#include "upper_body_canfd_hardware/canfd_system.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "upper_body_canfd_hardware/protocol.hpp"

namespace upper_body_canfd_hardware {
class SocketCan {
public:
  explicit SocketCan(const std::string &interface) {
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd_ < 0)
      throw std::runtime_error("cannot create SocketCAN socket: " +
                               std::string(strerror(errno)));
    int enable_fd = 1;
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
      close(fd_);
      throw std::runtime_error("cannot enable CAN-FD");
    }
    ifreq request{};
    std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
      close(fd_);
      throw std::runtime_error("unknown CAN interface: " + interface);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) <
        0) {
      close(fd_);
      throw std::runtime_error("cannot bind " + interface);
    }
  }
  ~SocketCan() {
    if (fd_ >= 0)
      close(fd_);
  }
  bool send(const uint32_t id, const uint8_t *data, const size_t size,
            const bool force_fd) {
    if (size > CANFD_MAX_DLEN)
      return false;
    if (force_fd || size > CAN_MAX_DLEN) {
      canfd_frame f{};
      f.can_id = id;
      f.len = size;
      f.flags = CANFD_BRS;
      std::memcpy(f.data, data, size);
      return ::write(fd_, &f, sizeof(f)) == sizeof(f);
    }
    can_frame f{};
    f.can_id = id;
    f.can_dlc = size;
    std::memcpy(f.data, data, size);
    return ::write(fd_, &f, sizeof(f)) == sizeof(f);
  }
  bool receive(canfd_frame &frame, bool &is_fd) {
    const auto n = ::read(fd_, &frame, sizeof(frame));
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return false;
    is_fd = n == CANFD_MTU;
    return n == CAN_MTU || n == CANFD_MTU;
  }

private:
  int fd_{-1};
};

namespace {
double parameter(const hardware_interface::HardwareInfo &info,
                 const std::string &name, const double fallback) {
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback
                                              : std::stod(it->second);
}
} // namespace

hardware_interface::CallbackReturn
CanFdSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  if (info_.joints.size() != 14) {
    RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"),
                 "exactly 14 ordered joints are required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::array<std::string, 14> expected_joints{
      {"left_shoulder_yaw_joint", "left_shoulder_roll_joint",
       "left_shoulder_pitch_joint", "left_elbow_pitch_joint",
       "left_wrist_yaw_joint", "left_wrist_pitch_joint",
       "left_wrist_roll_joint", "right_shoulder_yaw_joint",
       "right_shoulder_roll_joint", "right_shoulder_pitch_joint",
       "right_elbow_pitch_joint", "right_wrist_yaw_joint",
       "right_wrist_pitch_joint", "right_wrist_roll_joint"}};
  for (size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (joint.name != expected_joints[index]) {
      RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"),
                   "joint %zu must be %s, got %s", index,
                   expected_joints[index].c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"),
                   "joint %s must expose position command and "
                   "position/velocity/effort states",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  auto get_string = [&](const char *key, const char *fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return it == info_.hardware_parameters.end() ? std::string(fallback)
                                                 : it->second;
  };
  interfaces_ = {get_string("left_interface", "can0"),
                 get_string("right_interface", "can1")};
  rp_kp_ = parameter(info_, "rp_kp", 30.0);
  rp_kd_ = parameter(info_, "rp_kd", 1.0);
  wrist_kp_ = parameter(info_, "wrist_kp", 0.008);
  wrist_kd_ = parameter(info_, "wrist_kd", 1.0);
  wrist_torque_constant_ =
      parameter(info_, "wrist_torque_constant_nm_per_amp", 0.35);
  feedback_timeout_s_ = parameter(info_, "feedback_timeout_s", 0.1);
  if (rp_kp_ < 0.0 || rp_kd_ < 0.0 || wrist_kp_ < 0.0 || wrist_kd_ < 0.0 ||
      wrist_torque_constant_ <= 0.0 || feedback_timeout_s_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"),
                 "invalid negative/zero hardware parameter");
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_.assign(14, std::numeric_limits<double>::quiet_NaN());
  velocity_.assign(14, 0.0);
  effort_.assign(14, 0.0);
  command_.assign(14, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CanFdSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < 14; ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                     &position_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                     &velocity_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                     &effort_[i]);
  }
  return out;
}
std::vector<hardware_interface::CommandInterface>
CanFdSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < 14; ++i)
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                     &command_[i]);
  return out;
}

hardware_interface::CallbackReturn
CanFdSystem::on_activate(const rclcpp_lifecycle::State &) {
  try {
    buses_[0] = std::make_unique<SocketCan>(interfaces_[0]);
    buses_[1] = std::make_unique<SocketCan>(interfaces_[1]);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"), "%s", e.what());
    buses_ = {};
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Query all drives while they are still disabled. Never invent a zero pose:
  // enabling against an unknown encoder position can cause an immediate jump.
  command_.assign(14, 0.0);
  if (!send_commands()) {
    buses_ = {};
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(feedback_timeout_s_);
  do {
    receive_feedback();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  for (size_t i = 0; i < 14; ++i) {
    if (!std::isfinite(position_[i])) {
      RCLCPP_ERROR(rclcpp::get_logger("CanFdSystem"),
                   "no initial feedback from joint %s",
                   info_.joints[i].name.c_str());
      buses_ = {};
      return hardware_interface::CallbackReturn::ERROR;
    }
    command_[i] = position_[i];
  }
  if (!send_enable(true)) {
    buses_ = {};
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
CanFdSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  send_enable(false);
  buses_ = {};
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CanFdSystem::send_enable(const bool enabled) {
  const std::array<uint8_t, 8> rp{
      {static_cast<uint8_t>(enabled), 0, 0, 0, 0, 0, 0, 0}};
  const std::array<uint8_t, 8> wrist{
      {static_cast<uint8_t>(enabled), 0, 0, 0, 0, 0, 0, 0}};
  bool ok = true;
  for (auto &bus : buses_)
    if (bus) {
      ok &= bus->send(protocol::kRpEnableId, rp.data(), rp.size(), false);
      for (unsigned n = 1; n <= 3; ++n)
        ok &= bus->send(protocol::wrist_control_id(n), wrist.data(),
                        wrist.size(), false);
    }
  return ok;
}

hardware_interface::return_type CanFdSystem::write(const rclcpp::Time &,
                                                   const rclcpp::Duration &) {
  return active_ && send_commands() ? hardware_interface::return_type::OK
                                    : hardware_interface::return_type::ERROR;
}
bool CanFdSystem::send_commands() {
  bool ok = true;
  for (size_t arm = 0; arm < 2; ++arm) {
    std::array<uint8_t, 64> frame{};
    for (size_t slot = 0; slot < 8; ++slot) {
      protocol::JointCommand c;
      if (slot < 4) {
        c.position = command_[arm * 7 + slot];
        c.kp = rp_kp_;
        c.kd = rp_kd_;
      }
      const auto packed = protocol::pack_rp_slot(c);
      std::copy(packed.begin(), packed.end(), frame.begin() + slot * 8);
    }
    ok &= buses_[arm]->send(protocol::kRpCommandId, frame.data(), frame.size(),
                            true);
    for (unsigned node = 1; node <= 3; ++node) {
      protocol::JointCommand c;
      c.position = command_[arm * 7 + 3 + node];
      c.kp = wrist_kp_;
      c.kd = wrist_kd_;
      const auto packed = protocol::pack_wrist_command(c);
      ok &= buses_[arm]->send(protocol::wrist_command_id(node), packed.data(),
                              packed.size(), true);
    }
  }
  return ok;
}

hardware_interface::return_type CanFdSystem::read(const rclcpp::Time &,
                                                  const rclcpp::Duration &) {
  if (!active_)
    return hardware_interface::return_type::ERROR;
  receive_feedback();
  const auto now = std::chrono::steady_clock::now();
  for (size_t a = 0; a < 2; ++a)
    for (size_t j = 0; j < 7; ++j)
      if (std::chrono::duration<double>(now - last_feedback_[a][j]).count() >
          feedback_timeout_s_)
        return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}
void CanFdSystem::receive_feedback() {
  for (size_t arm = 0; arm < 2; ++arm) {
    if (!buses_[arm])
      continue;
    canfd_frame f{};
    bool fd = false;
    while (buses_[arm]->receive(f, fd)) {
      static_cast<void>(fd);
      try {
        size_t joint = 99;
        protocol::JointState s;
        const auto id = f.can_id & CAN_SFF_MASK;
        if (id >= 1 && id <= 4 && f.len == 12) {
          joint = id - 1;
          s = protocol::unpack_rp_state(f.data, f.len);
        } else
          for (unsigned n = 1; n <= 3; ++n)
            if ((id == protocol::wrist_reply_id(n) && f.len == 12) ||
                (id == protocol::wrist_status_id(n) && f.len == 20)) {
              joint = 3 + n;
              s = protocol::unpack_wrist_feedback(f.data, f.len);
              s.effort *= wrist_torque_constant_;
              break;
            }
        if (joint < 7) {
          const auto i = arm * 7 + joint;
          position_[i] = s.position;
          velocity_[i] = s.velocity;
          effort_[i] = s.effort;
          last_feedback_[arm][joint] = std::chrono::steady_clock::now();
        }
      } catch (const std::exception &) {
      }
    }
  }
}
} // namespace upper_body_canfd_hardware

PLUGINLIB_EXPORT_CLASS(upper_body_canfd_hardware::CanFdSystem,
                       hardware_interface::SystemInterface)
