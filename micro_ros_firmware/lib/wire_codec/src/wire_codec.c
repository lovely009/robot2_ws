#include "wire_codec.h"

#include <math.h>

// 用 32767 而非 32768：满量程落在 int16 内，-32768 整个留给哨兵。
#define WIRE_Q_MAX 32767.0

void wire_command_guard_reset(wire_command_guard_t *guard, uint64_t session_id) {
  guard->session_id = session_id;
  guard->sequence = 0U;
  guard->have_sequence = false;
}

bool wire_command_guard_allows(const wire_command_guard_t *guard,
    uint64_t session_id, uint32_t sequence) {
  if (session_id == 0U || session_id != guard->session_id) return false;
  // 半区间比较允许 uint32 回绕，重复与旧序号不刷新接收时基。
  const uint32_t distance = sequence - guard->sequence;
  return !guard->have_sequence || (distance != 0U && distance < 0x80000000U);
}

void wire_command_guard_commit(wire_command_guard_t *guard, uint32_t sequence) {
  guard->sequence = sequence;
  guard->have_sequence = true;
}

bool wire_command_deadline_valid(uint32_t now_ms, uint32_t deadline_ms) {
  const uint32_t remaining = deadline_ms - now_ms;
  return remaining != 0U && remaining < 0x80000000U;
}

uint64_t wire_session_from_epoch_ns(uint64_t epoch_ns, uint64_t previous_session) {
  const uint64_t milliseconds = epoch_ns / 1000000ULL;
  if (milliseconds == 0U) return 0U;
  // 标识按 epoch 毫秒分配，保持 rosbridge/JavaScript 整数精确；不作执行时钟。
  const uint64_t session = milliseconds > previous_session ? milliseconds : previous_session + 1U;
  return session <= 9007199254740991ULL ? session : 0U;
}

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

uint16_t wire_encode_state_age(uint32_t age_ms) {
  if (age_ms == 0xFFFFFFFFu) {
    return WIRE_AGE_MS_NEVER;
  }
  if (age_ms >= WIRE_AGE_MS_SATURATED) {
    return WIRE_AGE_MS_SATURATED;
  }
  return (uint16_t)age_ms;
}

static int16_t encode_i16_scaled(double value, double scale) {
  if (!isfinite(value)) {
    return WIRE_SENTINEL_MISSING;
  }
  const double scaled = value * scale;
  if (scaled <= -32767.0) {
    return (int16_t)-32767;
  }
  if (scaled >= 32767.0) {
    return (int16_t)32767;
  }
  return (int16_t)lround(scaled);
}

int16_t wire_encode_temperature_dc(double temperature_c) {
  return encode_i16_scaled(temperature_c, 10.0);
}

int16_t wire_encode_current_ma(double current_a) {
  return encode_i16_scaled(current_a, 1000.0);
}

uint16_t wire_encode_voltage_mv(double voltage_v) {
  if (!isfinite(voltage_v)) {
    return WIRE_VOLTAGE_MV_MISSING;
  }
  const double scaled = voltage_v * 1000.0;
  if (scaled <= 0.0) {
    return 0U;
  }
  if (scaled >= 65534.0) {
    return 0xFFFEu;
  }
  return (uint16_t)lround(scaled);
}

int32_t wire_encode_current_ma_i32(double current_a) {
  if (!isfinite(current_a)) return INT32_MIN;
  const double value = current_a * 1000.0;
  if (value >= INT32_MAX) return INT32_MAX;
  if (value <= -2147483647.0) return -2147483647;
  return (int32_t)llround(value);
}

uint32_t wire_encode_voltage_mv_u32(double voltage_v) {
  if (!isfinite(voltage_v)) return UINT32_MAX;
  const double value = voltage_v * 1000.0;
  if (value <= 0.0) return 0U;
  if (value >= 4294967294.0) return UINT32_MAX - 1U;
  return (uint32_t)llround(value);
}

uint8_t wire_encode_status_flags(wire_status_flags_input_t in) {
  uint8_t flags = 0U;
  if (in.online) {
    flags |= 0x01u;
  }
  if (in.device_protect_known) {
    flags |= 0x02u;
  }
  if (in.device_protect_enabled) {
    flags |= 0x04u;
  }
  if (in.device_direction_known) {
    flags |= 0x08u;
  }
  if (in.device_direction != 0U) {
    flags |= 0x10u;
  }
  return flags;
}



bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots) {
  return count <= max_slots &&
         motor_ids_len == count && position_q_len == count &&
         velocity_q_len == count && effort_q_len == count;
}

bool wire_config_groups_exclusive(uint8_t kind, uint16_t count,
    size_t targets_len, uint8_t command, uint16_t motor_id) {
  if (count != targets_len) {
    return false;
  }
  if (kind == WIRE_CONFIG_KIND_PARAM_SET) {
    return command == WIRE_CONFIG_COMMAND_SAVE &&
           motor_id == WIRE_MOTOR_ID_NONE;
  }
  if (kind == WIRE_CONFIG_KIND_SYSTEM_COMMAND) {
    return count == 0U;
  }
  return false;
}
