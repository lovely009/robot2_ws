// 参数面对外单位与内核 SI 的换算（纯函数，host 可直测）。
//
// 对外位置 rad、速度 rad/s 直通，力矩列用 A；仅 param_bridge 做 A ↔ N·m 换算。
#ifndef PARAM_UNITS_H
#define PARAM_UNITS_H

#include <math.h>

// nm_per_amp 为调用方提供的关节侧 N·m/A；生产优先规格书峰值对，
// 缺失时回退 Kt×减速比×效率，与反馈估计系数区分。
// 非正/NaN 系数或非有限输入返回 NaN，调用方保持 SI 语义或拒绝下发。
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
