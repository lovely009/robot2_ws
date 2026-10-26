// 线协议编译期校验；尺寸按 CDR 布局计算，不使用含 data/size/capacity 的 sizeof。

#include <can_bridge_msgs/msg/detail/config_request__struct.h>
#include <can_bridge_msgs/msg/detail/low_command__struct.h>
#include <can_bridge_msgs/msg/detail/low_state__struct.h>
#include <can_bridge_msgs/msg/detail/motor_status__struct.h>
#include <can_bridge_msgs/msg/detail/system_command__struct.h>
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>

#include "firmware_config.h"
#include "wire_codec.h"

namespace {

// CDR：uint64 + uint32 + uint8 + 3 B 对齐填充，然后 4 个有界序列。
constexpr size_t kLowCommandWire =
    8 + 4 + 4 + 1 + 3
    + (4 + can_bridge_msgs__msg__LowCommand__motor_ids__MAX_SIZE)
    + 3 * (4 + 2 * can_bridge_msgs__msg__LowCommand__position_q__MAX_SIZE);

// 32 B 固定头、6 个序列长度前缀与每槽10 B，保守计入4 B额外对齐。
constexpr size_t kLowStateWire = 32 + 6 * 4 + 10 *
    can_bridge_msgs__msg__LowState__motor_ids__MAX_SIZE + 4;

// best-effort 可用载荷：UXR_CONFIG_CUSTOM_TRANSPORT_MTU(512) 扣掉 XRCE 头。
constexpr size_t kBestEffortPayloadMax = 490;
// CDR 封装头 4 B，须加到消息体尺寸上。
constexpr size_t kCdrEncapsulation = 4;

static_assert(kLowCommandWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_cmd 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");
static_assert(kLowStateWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_state 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");

static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == -32768,
    "哨兵值变了：定点编解码两端都会错");
static_assert(can_bridge_msgs__msg__WireContract__MOTOR_COUNT_MAX >= FIRMWARE_MAX_MOTORS,
    "线协议电机数上界必须覆盖固件注册容量");

// wire_codec 不依赖 ROS；在此校验其哨兵与生成常量一致。
static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == WIRE_SENTINEL_MISSING,
    "wire_codec 的哨兵与 WireContract 漂移了：两端定点解码会错位");

static_assert(WIRE_CONFIG_KIND_PARAM_SET ==
    can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_PARAM_SET,
    "wire_codec 与 ConfigRequest 的 PARAM_SET 常量漂移了");
static_assert(WIRE_CONFIG_KIND_SYSTEM_COMMAND ==
    can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_SYSTEM_COMMAND,
    "wire_codec 与 ConfigRequest 的 SYSTEM_COMMAND 常量漂移了");
static_assert(WIRE_CONFIG_COMMAND_SAVE ==
    can_bridge_msgs__msg__SystemCommand__COMMAND_SAVE,
    "wire_codec 的 COMMAND_SAVE 与 SystemCommand 漂移了");
static_assert(WIRE_MOTOR_ID_NONE ==
    can_bridge_msgs__msg__SystemCommand__MOTOR_ID_NONE,
    "wire_codec 的 MOTOR_ID_NONE 与 SystemCommand 漂移了");

// rosidl 生成的浮点与字符串常量不支持 static_assert；由主机侧契约测试校验。

// motor_status 允许分片，不设单帧尺寸断言。

}  // namespace
