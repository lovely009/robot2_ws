#include "wire_codec.h"

#include <math.h>

// 用 32767 而非 32768：满量程落在 int16 内，-32768 整个留给哨兵。
#define WIRE_Q_MAX 32767.0

wire_quantize_result_t wire_quantize(double value, double scale) {
  wire_quantize_result_t r = {0, false};
  if (!isfinite(value)) {
    r.q = WIRE_SENTINEL_MISSING;
    return r;
  }
  const double raw = value * WIRE_Q_MAX / scale;
  // 严格不等：**恰好**落在 ±full-scale 是可精确表示的，不算越界、不置饱和位。
  // 写成 >= 会让「满量程」这种正常命令在页面上显示成饱和（测试 test_saturation
  // 的 `!r.saturated` 正是钉这一点）。
  if (raw > WIRE_Q_MAX) {
    r.q = (int16_t)32767;
    r.saturated = true;
  } else if (raw < -WIRE_Q_MAX) {
    r.q = (int16_t)-32767;
    r.saturated = true;
  } else {
    r.q = (int16_t)lround(raw);
  }
  return r;
}

double wire_dequantize(int16_t q, double scale) {
  if (q == WIRE_SENTINEL_MISSING) {
    return NAN;
  }
  return (double)q * scale / WIRE_Q_MAX;
}
