// 参数面对外单位与内核 SI 的换算（纯函数，host 可直测）。
//
// 参数面是**上层边界**，与 ROS 侧同单位：位置 rad、速度 rad/s 直通不换算，只有力矩列对外是 **A**。
// 力矩用 A 而非 N·m 的理由：设备侧权威对象是 0x2538（电机最大电流，mA），折成 N·m 要过一次关节侧
// N·m/A 系数，而该系数本身有未决争议（TBD-PROFILE-02）；读出来的是设备存的值、写下去的是设备收的值。
// 换算只在 param_bridge 边界做一次（读出 SI→A、写入 A→SI），前后端不再各自换算。
#ifndef PARAM_UNITS_H
#define PARAM_UNITS_H

#include <math.h>

// 力矩 ↔ 电流：nm_per_amp = 关节侧 N·m/A，**由调用方给**，本文件不假定其来源。
// 生产路径传的是 motor_protocol_profile_limit_nm_per_amp()：有规格书峰值对就
// 优先用它，否则退 Kt × 减速比 × 效率（反馈估计走 Kt 链，是另一个量）。
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
