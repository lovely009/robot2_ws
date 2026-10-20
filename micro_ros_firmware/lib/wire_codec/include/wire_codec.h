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

// 循环面槽位形状：count 必须等于四个序列的声明长度，且不超过内核容量。
// 只校验「同一帧内的自洽性」，不解释内容——值的合法性归 mailbox。
bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots);

#ifdef __cplusplus
}
#endif

#endif // WIRE_CODEC_H
