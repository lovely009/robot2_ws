#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/logger.hpp"

#include "motor_drive/motor_driver.hpp"
#include "motor_drive/motor_protocol.hpp"
#include "motor_drive/protocol.hpp"
#include "motor_drive/socketcan.hpp"
#include "motor_canfd_sim/plant_model.hpp"
#include "motor_canfd_sim/sim_frame_decode.hpp"

namespace motor_canfd_sim {

namespace drive = motor_drive;

// 一条模拟总线：配置、SocketCAN 通道与挂在这条总线上的电机下标。
struct SimBus {
  drive::BusConfig config;
  std::unique_ptr<drive::SocketCan> socket;
  std::vector<std::size_t> motor_indices;
};

// 传给家族 handler 的运行上下文：只读配置、可变运行状态与回包通道。
struct SimHandlerContext {
  const drive::DriverConfig &driver_config;
  std::vector<SimBus> &buses;
  const std::vector<std::size_t> &motor_to_bus;
  std::vector<SimMotorState> &motor_states;
  std::vector<JointSample> &samples;
  MotorPlantModel &plant;
  rclcpp::Logger logger;
};

// 电机家族仿真处理器：识别本家族的 CAN 帧、更新电机状态并构造反馈帧。
// 与 motor_drive::MotorProtocol 一一对应；新增家族时在 registry 注册
// 一个 handler，节点本身不出现任何家族分支。
class SimMotorHandler {
public:
  virtual ~SimMotorHandler() = default;

  // 处理 bus 上收到的一帧；返回 true 表示该帧属于本家族并被消费。
  virtual bool handle_frame(const SimHandlerContext &ctx, std::size_t bus,
                            const drive::CanFrame &frame, Clock::time_point now) = 0;

  // 周期性状态帧发布；无状态帧的家族保持空实现。
  virtual void status_tick(const SimHandlerContext &ctx) = 0;
};

// RP 家族：使能广播、64 Byte 槽位命令与 12 Byte 状态反馈。
class RpSimMotorHandler final : public SimMotorHandler {
public:
  bool handle_frame(const SimHandlerContext &ctx, std::size_t bus,
                    const drive::CanFrame &frame, Clock::time_point now) override;
  void status_tick(const SimHandlerContext &ctx) override;

private:
  void send_feedback(const SimHandlerContext &ctx, std::size_t bus) const;
};

// 25/30 mm 家族：按节点 ID 的命令/控制帧与反馈/状态帧。
class Motor2530SimMotorHandler final : public SimMotorHandler {
public:
  bool handle_frame(const SimHandlerContext &ctx, std::size_t bus,
                    const drive::CanFrame &frame, Clock::time_point now) override;
  void status_tick(const SimHandlerContext &ctx) override;

private:
  void send_reply(const SimHandlerContext &ctx, std::size_t motor) const;
};

// 家族注册表：与 motor_drive::MotorProtocolRegistry 对称。
class SimMotorHandlerRegistry {
public:
  using Factory = std::function<std::unique_ptr<SimMotorHandler>()>;

  void register_factory(std::string type, Factory factory);
  bool contains(const std::string &type) const noexcept;
  std::unique_ptr<SimMotorHandler> create(const std::string &type) const;

  static SimMotorHandlerRegistry with_builtin_handlers();

private:
  std::unordered_map<std::string, Factory> factories_;
};

} // namespace motor_canfd_sim
