// 参数服务器的对外工程单位 ↔ 内核 SI 的换算（纯函数，host 可直测）。
//
// 口径（2026-09-17 定）：参数服务器是"人看的边界"——现场手里只有**角度、
// 角度/秒、安培**（设备里的对象本来就是 cnt / RPM / mA），rad / rad/s / N·m
// 只留在内核与 W25 记录里。换算只在 param_bridge 的边界做一次：
//   - 读出：内核 SI → 参数（deg / deg/s / A）
//   - 写入：参数（deg / deg/s / A）→ 内核 SI
// 前后端都不再各自换算，避免"页面换算一次、桥再换算一次"的重复缩放。
//
// 力矩列用 A 而不是 N·m 的理由：设备侧的权威对象是 0x2538（电机最大电流，
// mA），把手里的电流值乘 Kt×减速比×效率折成 N·m 只会引入一次系数争议
// （见 docs/architecture/motor-runtime-decision-log.md §6.4）。
#ifndef PARAM_UNITS_H
#define PARAM_UNITS_H

#include <math.h>

// π 不依赖 M_PI（-std=c11 下未必定义）。
#define PARAM_UNITS_PI 3.14159265358979323846
#define PARAM_UNITS_RAD_TO_DEG (180.0 / PARAM_UNITS_PI)

// 角度：rad ↔ deg。
static inline float param_units_rad_to_deg(float rad) {
  return (float)((double)rad * PARAM_UNITS_RAD_TO_DEG);
}
static inline float param_units_deg_to_rad(float deg) {
  return (float)((double)deg / PARAM_UNITS_RAD_TO_DEG);
}

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
