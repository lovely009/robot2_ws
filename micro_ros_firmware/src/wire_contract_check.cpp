// 线上契约的编译期门禁。本文件**没有运行期代码**，只有 static_assert：
// 尺寸越界在这里让 pio run 直接失败，而不是等台架上丢帧才发现。
//
// 尺寸算术用的是 CDR 序列化布局（每个有界序列 4 B 长度前缀 + 元素），不是
// sizeof()——后者含 data/size/capacity 三元组，比线上多 32 B，拿它当门禁没有余量。

#include <can_bridge_msgs/msg/detail/config_request__struct.h>
#include <can_bridge_msgs/msg/detail/low_command__struct.h>
#include <can_bridge_msgs/msg/detail/low_state__struct.h>
#include <can_bridge_msgs/msg/detail/motor_status__struct.h>
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>

#include "wire_codec.h"

namespace {

// CDR：uint64 + uint32 + uint8 + 3 B 对齐填充，然后 4 个有界序列。
constexpr size_t kLowCommandWire =
    8 + 4 + 1 + 3
    + (4 + can_bridge_msgs__msg__LowCommand__motor_ids__MAX_SIZE)
    + 3 * (4 + 2 * can_bridge_msgs__msg__LowCommand__position_q__MAX_SIZE);

constexpr size_t kLowStateWire =
    8 + 4 + 1 + 3
    + (4 + can_bridge_msgs__msg__LowState__motor_ids__MAX_SIZE)
    + 3 * (4 + 2 * can_bridge_msgs__msg__LowState__position_q__MAX_SIZE);

// best-effort 可用载荷：UXR_CONFIG_CUSTOM_TRANSPORT_MTU(512) 扣掉 XRCE 头。
constexpr size_t kBestEffortPayloadMax = 490;
// CDR 封装头（representation identifier + options）4 B。上面两个只算消息体，
// 线上实际占用要加上它——漏算这 4 B 会让门禁比真实余量宽。
constexpr size_t kCdrEncapsulation = 4;

static_assert(kLowCommandWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_cmd 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");
static_assert(kLowStateWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_state 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");

// 枚举常量是整型常量表达式，可以直接断言。
static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == -32768,
    "哨兵值变了：定点编解码两端都会错");
static_assert(can_bridge_msgs__msg__WireContract__MOTOR_COUNT_MAX == 64,
    "电机数上界与内核 MOTOR_CONTROL_MAX_MOTORS 脱钩了");

// wire_codec 的哨兵是**受控的第二份**（该库刻意不依赖 ROS，host 可直编，拿不到生成常量）。
// 宏与枚举都是整型常量表达式，所以这条能进 static_assert —— 两份漂移在这里就编不过。
static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == WIRE_SENTINEL_MISSING,
    "wire_codec 的哨兵与 WireContract 漂移了：两端定点解码会错位");

// 量程与话题名在这里**故意不设断言**：rosidl 给 C 侧生成的是 `static const float` 与
// `static const char * const`，两者都不是 C++ 常量表达式（[expr.const] 只放行 constexpr
// 与 const 整型/枚举），进不了 static_assert。它们由主机侧契约用例按精确值钉住
// （micro_ros_bridge 的 Contract.ScalesAndSentinel / TopicNamesAreAbsoluteAndUnderMicroRos）。

// motor_status 不设尺寸门禁：它允许分片，而精确 CDR 长度要靠 40 多项字段算，
// 写成 static_assert 既冗长又容易与消息定义漂移。它的门禁是**运行期测量**
// （序列化一条 count=64 的 MotorStatus 看长度），等 status_bridge 存在之后。

}  // namespace
