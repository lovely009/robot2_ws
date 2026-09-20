// 参数服务器的对外单位与内核 SI 的换算（纯函数，host 可直测）。
//
// 口径（2026-09-20 改；此前是"对外一律工程单位 deg/deg/s/A"，2026-09-17 定）：
// 参数服务器是**上层边界**，与 ROS 侧同单位——位置 rad、速度 rad/s（内核就是
// 这套，直通、不换算），只有力矩列对外是 **A**。理由两条：
//   - 上层（web 上位机等）拿到的位置/速度要和 `motion_states`、URDF 对得上，
//     曾经"参数看度数、状态看弧度"要人脑两套换算，且页面还得知道 π。
//   - 力矩列用 A 而不是 N·m 的理由仍然成立：设备侧的权威对象是 0x2538（电机
//     最大电流，mA，「建议用默认」），折成 N·m 要过一次 Kt×减速比×效率，而该
//     系数本身有未决争议（见 docs/architecture/motor-runtime-decision-log.md
//     §6.4/§6.6 TBD-PROFILE-02）。读出来的是设备里存的值，写下去的是设备收的值。
//
// 换算只在 param_bridge 边界做一次（读出 SI→A、写入 A→SI），前后端都不再各自
// 换算，避免"页面换算一次、桥再换算一次"的重复缩放。
#ifndef PARAM_UNITS_H
#define PARAM_UNITS_H

#include <math.h>

// 力矩 ↔ 电流：nm_per_amp = Kt × 减速比 × 效率（关节侧 N·m/A）。
// 系数缺失（未绑定型号 / 非正值 / NaN）时不换算——返回 NaN，调用方据此保持
// SI 语义（宁可显示 N·m，也不拿一个瞎猜的系数除）。
static inline float param_units_nm_to_amp(float nm, float nm_per_amp) {
  if (!(nm_per_amp > 0.0f) || !isfinite(nm)) {
    return NAN;
  }
  return nm / nm_per_amp;
}
static inline float param_units_amp_to_nm(float amp, float nm_per_amp) {
  if (!(nm_per_amp > 0.0f) || !isfinite(amp)) {
    return NAN;
  }
  return amp * nm_per_amp;
}

#endif // PARAM_UNITS_H
