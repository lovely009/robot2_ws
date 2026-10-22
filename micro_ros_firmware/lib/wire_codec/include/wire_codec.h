// 定点编解码：物理量 ↔ 线协议 int16 字段。**固件侧唯一换算点**
// （主机侧对应 micro_ros_bridge 的 micro_ros_codec）。
//
// 单位边界：本库是协议单位与 SI 单位的边界，调用方传进来的必须是 rad / rad/s / N·m。
// 量程常量不在这里定义（唯一来源是 WireContract.msg 的 SCALE_* 常量）。
#ifndef WIRE_CODEC_H
#define WIRE_CODEC_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 本拍无有效测量。**值必须与 WireContract.msg 的 SENTINEL_MISSING 一致**：
// 本库刻意不依赖 ROS（host 可直编），拿不到生成常量，所以这里是**受控的第二份**，
// 由 src/wire_contract_check.cpp 的 static_assert 把两份钉在一起。
#define WIRE_SENTINEL_MISSING ((int16_t)-32768)

typedef struct {
  int16_t q;
  bool saturated;  // 被量程钳位（上层据此判断量程常量是否选错）
} wire_quantize_result_t;

// 定点：q = clamp(round(value * 32767 / scale), -32767, 32767)。
// 前置条件：value 有限——非有限输入（NaN/Inf）返回哨兵值，不做未定义行为，
// 但**命令路径的 NaN 必须在调用前被拒**（整条拒收语义，不在这里兜）。
wire_quantize_result_t wire_quantize(double value, double scale);

// 反定点。q == WIRE_SENTINEL_MISSING → NAN。
double wire_dequantize(int16_t q, double scale);

// MotorStatus 健康/电气字段的编码。缺失与饱和分离：年龄 0xFFFF = 从未收到、
// 0xFFFE = 真实年龄饱和；电压 0xFFFF = 缺失；int16 字段复用 WIRE_SENTINEL_MISSING。
// 入参沿用内核快照语义：年龄 0xFFFFFFFF = 从未测量，电气量 NaN = 缺失。
#define WIRE_AGE_MS_NEVER 0xFFFFU
#define WIRE_AGE_MS_SATURATED 0xFFFEU
#define WIRE_VOLTAGE_MV_MISSING 0xFFFFU

typedef struct {
  bool online;
  bool device_protect_known;
  bool device_protect_enabled;
  bool device_direction_known;
  uint8_t device_direction;  // 0=CW，非零=CCW，原值直通
} wire_status_flags_input_t;

// 以下函数先钳位后取整，非有限输入给缺失哨兵（不进入浮点转整数 UB）。
uint16_t wire_encode_state_age(uint32_t age_ms);
int16_t wire_encode_temperature_dc(double temperature_c);
int16_t wire_encode_current_ma(double current_a);
uint16_t wire_encode_voltage_mv(double voltage_v);
// 位序：bit0 online，bit1 protect_known，bit2 protect_enabled，
// bit3 direction_known，bit4 direction。
uint8_t wire_encode_status_flags(wire_status_flags_input_t in);


// 循环面槽位形状：count 必须等于四个序列的声明长度，且不超过内核容量。
// 只校验「同一帧内的自洽性」，不解释内容——值的合法性归 mailbox。
bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots);

// 配置面两组字段互斥（ConfigRequest.msg）。不适用组的哨兵值按**实际编码端**取：
// micro_ros_bridge 的 encode_config(ParamSetArray) 写 command = COMMAND_SAVE(0)、
// motor_id = MOTOR_ID_NONE(65535)。kind/command/motor_id 哨兵是本库的**受控第二份**，
// 由 src/wire_contract_check.cpp 的 static_assert 钉到生成常量上。
#define WIRE_CONFIG_KIND_PARAM_SET 0U
#define WIRE_CONFIG_KIND_SYSTEM_COMMAND 1U
#define WIRE_CONFIG_COMMAND_SAVE 0U
#define WIRE_MOTOR_ID_NONE 65535U

bool wire_config_groups_exclusive(uint8_t kind, uint16_t count,
    size_t targets_len, uint8_t command, uint16_t motor_id);

#ifdef __cplusplus
}
#endif

#endif // WIRE_CODEC_H
