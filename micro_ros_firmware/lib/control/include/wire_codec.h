// 定点编解码：调用方输入 rad/rad/s/N·m，量程由 WireContract.msg 的 SCALE_* 提供。
// 固件统一在此换算，主机侧对应 micro_ros_codec。
#ifndef WIRE_CODEC_H
#define WIRE_CODEC_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 缺失测量哨兵；与 WireContract.msg 一致性由 wire_contract_check.cpp 静态断言校验。
#define WIRE_SENTINEL_MISSING ((int16_t)-32768)

typedef struct {
  int16_t q;
  bool saturated;  // 被量程钳位（上层据此判断量程常量是否选错）
} wire_quantize_result_t;

typedef struct {
  uint64_t session_id;
  uint32_t sequence;
  bool have_sequence;
} wire_command_guard_t;

void wire_command_guard_reset(wire_command_guard_t *guard, uint64_t session_id);
bool wire_command_guard_allows(const wire_command_guard_t *guard,
    uint64_t session_id, uint32_t sequence);
void wire_command_guard_commit(wire_command_guard_t *guard, uint32_t sequence);
bool wire_command_deadline_valid(uint32_t now_ms, uint32_t deadline_ms);
uint64_t wire_session_from_epoch_ns(uint64_t epoch_ns, uint64_t previous_session);

// 定点：q = clamp(round(value * 32767 / scale), -32767, 32767)。
// 前置条件：value 有限——非有限输入（NaN/Inf）返回哨兵值，不做未定义行为，
// 但**命令路径的 NaN 必须在调用前被拒**（整条拒收语义，不在这里兜）。
wire_quantize_result_t wire_quantize(double value, double scale);

// 反定点。q == WIRE_SENTINEL_MISSING → NAN。
double wire_dequantize(int16_t q, double scale);

// MotorStatus 健康/电气字段的编码。缺失与饱和分离：年龄 0xFFFF = 从未收到、
// 0xFFFE = 真实年龄饱和；温度复用 WIRE_SENTINEL_MISSING，电流/电压缺失为 INT32_MIN/UINT32_MAX。
// 入参沿用内核快照语义：年龄 0xFFFFFFFF = 从未测量，电气量 NaN = 缺失。
#define WIRE_AGE_MS_NEVER 0xFFFFU
#define WIRE_AGE_MS_SATURATED 0xFFFEU

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
int32_t wire_encode_current_ma_i32(double current_a);
uint32_t wire_encode_voltage_mv_u32(double voltage_v);
// 位序：bit0 online，bit1 protect_known，bit2 protect_enabled，
// bit3 direction_known，bit4 direction。
uint8_t wire_encode_status_flags(wire_status_flags_input_t in);


// 循环面槽位形状：count 必须等于四个序列的声明长度，且不超过内核容量。
// 只校验「同一帧内的自洽性」，不解释内容——值的合法性归 mailbox。
bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots);

#ifdef __cplusplus
}
#endif

#endif // WIRE_CODEC_H
