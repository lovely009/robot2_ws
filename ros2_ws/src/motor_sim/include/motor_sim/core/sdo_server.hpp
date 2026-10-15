#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "motor_sim/core/config.hpp"

namespace motor_sim::core {

// CERB SDO 中止码：设备的字典是**分层**答的——索引不在字典里回"对象不存在"，
// 索引在而子索引没有回"子索引不存在"（2026-10-15 真机 node 1 实测）。
inline constexpr uint32_t kCerbSdoAbortNoObject = 0x06020000U;
inline constexpr uint32_t kCerbSdoAbortNoSubindex = 0x06090011U;

// 设备对象字典里**存在**的主索引（厂家 PR0002/TN0002 + 实机实测）。只用来挑上面
// 两个中止码，不表示本模拟器实现了它们——没实现的对象按"只有这些子索引"答。
inline constexpr std::array<uint16_t, 25> kCerbObjectIndices{
    0x1000, 0x1008, 0x1009, 0x100A, 0x1011, 0x1017, 0x1018, 0x1800,
    0x2530, 0x2531, 0x2538, 0x2539, 0x2540, 0x2541, 0x2650, 0x2654,
    0x3100, 0x4500, 0x603F, 0x6041, 0x6060, 0x6061, 0x6064, 0x607D,
    0x6080};

using SdoObjectKey = std::pair<uint16_t, uint8_t>;
// SDO 应答恒为单帧 8 字节（FD/BRS）。
using SdoFrame = std::array<uint8_t, 8>;

// 一台 CERB 的 SDO 从站。与传输无关：调用方把 0x600+id 的请求喂进来，
// 把应答发到 0x580+id。
class CerbSdoServer {
 public:
  void add_node(unsigned node_id, std::string identity,
                std::map<SdoObjectKey, int32_t> objects);

  // 返回 false = 本从站不该应答（非本从站节点）。是 SDO 命名空间但命令/对象
  // 不认的，仍回一条 abort 并返回 true。
  bool handle(unsigned node_id, const SdoFrame &request, SdoFrame &response);

 private:
  struct Node {
    std::string identity;
    std::map<SdoObjectKey, int32_t> objects;
    std::size_t segment_offset{0U};
  };

  // 身份串的分段上传：initiate 回总长，随后 0x60/0x70 交替取段。
  void put_identity_initiate(Node &node, SdoFrame &response);
  void put_identity_segment(Node &node, uint8_t command, SdoFrame &response);

  std::map<unsigned, Node> nodes_;
};

// 按 MotorConfig 造一台 CERB 的对象字典初值。位置限位与负载端位置是固定出厂值；
// 设备端限值来自型号库；保护开关与方向来自 sdo_overrides（真机这两个对象易失）。
std::map<SdoObjectKey, int32_t> cerb_sdo_initial_objects(const MotorConfig &motor);

}  // namespace motor_sim::core
