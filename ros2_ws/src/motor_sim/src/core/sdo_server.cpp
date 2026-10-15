#include "motor_sim/core/sdo_server.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace motor_sim::core {
namespace {

bool is_known_index(const uint16_t index) {
  return std::find(kCerbObjectIndices.begin(), kCerbObjectIndices.end(),
                   index) != kCerbObjectIndices.end();
}

uint32_t little_endian_unsigned(const SdoFrame &frame) {
  return static_cast<uint32_t>(frame[4]) |
         (static_cast<uint32_t>(frame[5]) << 8) |
         (static_cast<uint32_t>(frame[6]) << 16) |
         (static_cast<uint32_t>(frame[7]) << 24);
}

void put_abort(const uint16_t index, const uint8_t sub, const uint32_t code,
               SdoFrame &out) {
  out = SdoFrame{};
  out[0] = 0x80;
  out[1] = static_cast<uint8_t>(index & 0xFFU);
  out[2] = static_cast<uint8_t>((index >> 8) & 0xFFU);
  out[3] = sub;
  out[4] = static_cast<uint8_t>(code & 0xFFU);
  out[5] = static_cast<uint8_t>((code >> 8) & 0xFFU);
  out[6] = static_cast<uint8_t>((code >> 16) & 0xFFU);
  out[7] = static_cast<uint8_t>((code >> 24) & 0xFFU);
}

void put_read_reply(const SdoFrame &request, const int32_t value,
                    SdoFrame &out) {
  const uint32_t raw = static_cast<uint32_t>(value);
  out = SdoFrame{};
  out[0] = 0x43;  // 快速上传：4 字节有效
  out[1] = request[1];
  out[2] = request[2];
  out[3] = request[3];
  out[4] = static_cast<uint8_t>(raw & 0xFFU);
  out[5] = static_cast<uint8_t>((raw >> 8) & 0xFFU);
  out[6] = static_cast<uint8_t>((raw >> 16) & 0xFFU);
  out[7] = static_cast<uint8_t>((raw >> 24) & 0xFFU);
}

}  // namespace

std::map<SdoObjectKey, int32_t> cerb_sdo_initial_objects(
    const MotorConfig &motor) {
  return {
      // 位置限位出厂值：±32768 cnt（±180°）。
      {{0x607D, 0x01}, -32768},
      {{0x607D, 0x02}, 32768},
      // 设备端限值：最大转速是模块侧共用值，最大电流逐型号。
      {{0x6080, 0x00}, motor.device_max_speed_rpm},
      {{0x2538, 0x00}, motor.device_max_current_ma},
      // 易失对象：上电回初值。
      {{0x2650, 0x00},
       static_cast<int32_t>(motor.sdo_overrides.protection_enabled)},
      {{0x2541, 0x00}, static_cast<int32_t>(motor.sdo_overrides.direction)},
      // 负载端实际位置当前值；设零后应为 0。
      {{0x6064, 0x00}, 0},
  };
}

void CerbSdoServer::add_node(unsigned node_id, std::string identity,
                             std::map<SdoObjectKey, int32_t> objects) {
  Node node;
  node.identity = std::move(identity);
  node.objects = std::move(objects);
  nodes_[node_id] = std::move(node);
}

bool CerbSdoServer::handle(unsigned node_id, const SdoFrame &request,
                           SdoFrame &response) {
  const auto found = nodes_.find(node_id);
  if (found == nodes_.end()) {
    return false;
  }
  Node &node = found->second;
  const uint8_t command = request[0];
  const uint16_t index =
      static_cast<uint16_t>(request[1] | (request[2] << 8));
  const uint8_t sub = request[3];
  const SdoObjectKey key{index, sub};

  if (command == 0x23) {  // 4 字节快速写：存起来并回 ack
    const auto object = node.objects.find(key);
    if (object == node.objects.end()) {
      put_abort(index, sub,
                is_known_index(index) ? kCerbSdoAbortNoSubindex
                                      : kCerbSdoAbortNoObject,
                response);
      return true;
    }
    object->second = static_cast<int32_t>(little_endian_unsigned(request));
    response = SdoFrame{};
    response[0] = 0x60;
    response[1] = request[1];
    response[2] = request[2];
    response[3] = sub;
    return true;
  }

  if (command == 0x40) {  // 上传发起
    const auto object = node.objects.find(key);
    if (object != node.objects.end()) {
      put_read_reply(request, object->second, response);
      return true;
    }
    put_abort(index, sub,
              is_known_index(index) ? kCerbSdoAbortNoSubindex
                                    : kCerbSdoAbortNoObject,
              response);
    return true;
  }

  return false;  // 分段上传的请求由 Task 3 处理
}

}  // namespace motor_sim::core
