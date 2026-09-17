// 电机协议编解码（vaxor 25/30mm + CERB CAN FD/MIT）。

#include "motor_protocol.h"

#include <math.h>

static int64_t clamp_i64(int64_t value, int64_t low, int64_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static double clamp_double(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static void write_le(uint8_t *destination, uint64_t value, size_t width) {
  for (size_t index = 0; index < width; ++index) {
    destination[index] = (uint8_t)(value >> (8U * index));
  }
}

static uint32_t read_le(const uint8_t *data, size_t width) {
  uint32_t value = 0U;
  for (size_t index = 0; index < width; ++index) {
    value |= (uint32_t)data[index] << (8U * index);
  }
  return value;
}

static int32_t read_be_i16(const uint8_t *data) {
  uint16_t raw = ((uint16_t)data[0] << 8U) | data[1];
  return raw <= INT16_MAX ? (int32_t)raw : (int32_t)raw - 65536;
}

uint32_t motor_protocol_float_to_uint(double value, double low, double high,
                                      unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    return 0U;
  }
  value = clamp_double(value, low, high);
  uint32_t maximum = (UINT32_C(1) << bits) - 1U;
  return (uint32_t)((value - low) * maximum / (high - low));
}

static uint32_t cerb_symmetric_float_to_uint(double value, double limit,
                                               unsigned bits) {
  double normalized = (clamp_double(value / limit, -1.0, 1.0) + 1.0) * 0.5;
  uint32_t maximum = (UINT32_C(1) << bits) - 1U;
  return (uint32_t)(normalized * maximum);
}

static uint16_t vaxor_velocity_to_code(double velocity_rad_s) {
  int32_t rpm10 =
      (int32_t)llround(velocity_rad_s * 60.0 / (2.0 * MOTOR_PROTOCOL_PI) * 10.0);
  rpm10 = clamp_i32(rpm10, -MOTOR_PROTOCOL_VAXOR_VELOCITY_CODE_LIMIT,
                    MOTOR_PROTOCOL_VAXOR_VELOCITY_CODE_LIMIT);
  return (uint16_t)(rpm10 + 0x8000);
}

static double vaxor_code_to_velocity(uint16_t code) {
  int32_t raw = (int32_t)code - 0x8000;
  return raw / 10.0 * 2.0 * MOTOR_PROTOCOL_PI / 60.0;
}

static uint32_t vaxor_position_to_code(double position_rad) {
  int64_t ticks = llround(position_rad / (2.0 * MOTOR_PROTOCOL_PI) *
                          (double)MOTOR_PROTOCOL_VAXOR_TICKS_PER_REV);
  ticks = clamp_i64(ticks, -MOTOR_PROTOCOL_VAXOR_POSITION_TICK_LIMIT,
                    MOTOR_PROTOCOL_VAXOR_POSITION_TICK_LIMIT);
  return (uint32_t)(ticks + INT64_C(0x80000000));
}

static double vaxor_code_to_position(uint32_t code) {
  return ((int64_t)code - INT64_C(0x80000000)) /
         (double)MOTOR_PROTOCOL_VAXOR_TICKS_PER_REV * 2.0 * MOTOR_PROTOCOL_PI;
}

static uint16_t vaxor_current_to_code(double current_a) {
  int32_t raw =
      (int32_t)llround(current_a / MOTOR_PROTOCOL_VAXOR_CURRENT_SCALE_A);
  raw = clamp_i32(raw, -32768, 32767);
  return (uint16_t)(raw + 0x8000);
}

static double vaxor_code_to_current(uint16_t code) {
  return ((int32_t)code - 0x8000) * MOTOR_PROTOCOL_VAXOR_CURRENT_SCALE_A;
}

uint32_t motor_protocol_vaxor_command_id(unsigned node) {
  return MOTOR_PROTOCOL_VAXOR_COMMAND_ID_BASE +
         MOTOR_PROTOCOL_VAXOR_NODE_STRIDE * node;
}

uint32_t motor_protocol_vaxor_control_id(unsigned node) {
  return MOTOR_PROTOCOL_VAXOR_CONTROL_ID_BASE +
         MOTOR_PROTOCOL_VAXOR_NODE_STRIDE * node;
}

uint32_t motor_protocol_vaxor_reply_id(unsigned node) {
  return MOTOR_PROTOCOL_VAXOR_REPLY_ID_BASE +
         MOTOR_PROTOCOL_VAXOR_NODE_STRIDE * node;
}

uint32_t motor_protocol_vaxor_status_id(unsigned node) {
  return MOTOR_PROTOCOL_VAXOR_STATUS_ID_BASE +
         MOTOR_PROTOCOL_VAXOR_NODE_STRIDE * node;
}

void motor_protocol_pack_vaxor_command(const motor_protocol_joint_command_t *command,
                                       uint8_t out[12]) {
  uint32_t position = vaxor_position_to_code(command->position);
  uint16_t velocity = vaxor_velocity_to_code(command->velocity);
  uint16_t kp = (uint16_t)clamp_i64(
      llround(command->kp / MOTOR_PROTOCOL_VAXOR_KP_SCALE), 0, 0xffff);
  uint16_t kd = (uint16_t)clamp_i64(
      llround(command->kd / MOTOR_PROTOCOL_VAXOR_KD_SCALE), 0, 0xffff);
  uint16_t current = vaxor_current_to_code(command->feedforward);
  write_le(out, position, 4U);
  write_le(out + 4U, velocity, 2U);
  write_le(out + 6U, kp, 2U);
  write_le(out + 8U, kd, 2U);
  write_le(out + 10U, current, 2U);
}

bool motor_protocol_unpack_vaxor_command(const uint8_t *data, size_t size,
                                         motor_protocol_joint_command_t *command) {
  if (data == NULL || size != 12U || command == NULL) {
    return false;
  }
  uint32_t position = read_le(data, 4U);
  uint16_t velocity = (uint16_t)read_le(data + 4U, 2U);
  uint16_t kp = (uint16_t)read_le(data + 6U, 2U);
  uint16_t kd = (uint16_t)read_le(data + 8U, 2U);
  uint16_t current = (uint16_t)read_le(data + 10U, 2U);
  command->position = vaxor_code_to_position(position);
  command->velocity = vaxor_code_to_velocity(velocity);
  command->kp = kp * MOTOR_PROTOCOL_VAXOR_KP_SCALE;
  command->kd = kd * MOTOR_PROTOCOL_VAXOR_KD_SCALE;
  command->feedforward = vaxor_code_to_current(current);
  return true;
}

void motor_protocol_pack_vaxor_control(const motor_protocol_vaxor_control_t *control,
                                       uint8_t out[8]) {
  out[0] = control->run ? 1U : 0U;
  out[1] = control->set_zero ? 1U : 0U;
  out[2] = control->mode;
  out[3] = 0U;
  out[4] = 0U;
  out[5] = 0U;
  out[6] = 0U;
  out[7] = 0U;
}

bool motor_protocol_unpack_vaxor_control(const uint8_t *data, size_t size,
                                         motor_protocol_vaxor_control_t *control) {
  if (data == NULL || size != 8U || control == NULL) {
    return false;
  }
  if (data[2] > (uint8_t)MOTOR_PROTOCOL_MODE_POSITION) {
    return false;
  }
  control->run = data[0] != 0U;
  control->set_zero = data[1] != 0U;
  control->mode = data[2];
  return true;
}

void motor_protocol_pack_vaxor_reply(unsigned node,
                                     const motor_protocol_joint_state_t *state,
                                     uint8_t out[12]) {
  write_le(out, motor_protocol_vaxor_command_id(node), 2U);
  write_le(out + 2U, vaxor_position_to_code(state->position), 4U);
  write_le(out + 6U, vaxor_velocity_to_code(state->velocity), 2U);
  write_le(out + 8U, vaxor_current_to_code(state->effort), 2U);
}

void motor_protocol_pack_vaxor_status(unsigned node,
                                      const motor_protocol_joint_state_t *state,
                                      double voltage_v, uint32_t fault,
                                      uint8_t motor_state,
                                      uint8_t calibration_state, uint8_t mode,
                                      uint8_t motor_type, uint8_t out[20]) {
  write_le(out, motor_protocol_vaxor_command_id(node), 2U);
  write_le(out + 2U, vaxor_position_to_code(state->position), 4U);
  write_le(out + 6U, vaxor_velocity_to_code(state->velocity), 2U);
  write_le(out + 8U, vaxor_current_to_code(state->effort), 2U);
  int32_t voltage_code =
      clamp_i32((int32_t)llround(voltage_v * 10.0), -32768, 32767);
  write_le(out + 10U, (uint16_t)(int16_t)voltage_code, 2U);
  write_le(out + 12U, fault, 4U);
  out[16] = motor_state;
  out[17] = calibration_state;
  out[18] = mode;
  out[19] = motor_type;
}

bool motor_protocol_unpack_vaxor_feedback(const uint8_t *data, size_t size,
                                          motor_protocol_joint_state_t *state) {
  if (data == NULL || (size != 12U && size != 20U) || state == NULL) {
    return false;
  }
  uint32_t position = read_le(data + 2U, 4U);
  uint16_t velocity = (uint16_t)read_le(data + 6U, 2U);
  uint16_t current = (uint16_t)read_le(data + 8U, 2U);
  state->position = vaxor_code_to_position(position);
  state->velocity = vaxor_code_to_velocity(velocity);
  state->effort = vaxor_code_to_current(current);
  return true;
}

bool motor_protocol_unpack_vaxor_status(
    const uint8_t *data, size_t size, motor_protocol_vaxor_status_t *status) {
  if (data == NULL || size != 20U || status == NULL) {
    return false;
  }
  int16_t voltage_code = (int16_t)read_le(data + 10U, 2U);
  status->voltage_v = voltage_code / 10.0;
  status->fault = read_le(data + 12U, 4U);
  status->motor_state = data[16];
  status->calibration_state = data[17];
  status->mode = data[18];
  status->motor_type = data[19];
  return true;
}

bool motor_protocol_cerb_node_valid(unsigned node) {
  return node >= 1U && node <= 127U;
}

bool motor_protocol_cerb_command_id(unsigned node, uint32_t *id) {
  if (!motor_protocol_cerb_node_valid(node) || id == NULL) {
    return false;
  }
  *id = MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE + node;
  return true;
}

bool motor_protocol_cerb_feedback_id(unsigned node, uint32_t *id) {
  if (!motor_protocol_cerb_node_valid(node) || id == NULL) {
    return false;
  }
  *id = MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE + node;
  return true;
}

// 0x607D：cnt → rad。正向换算见 cerb_adapter 的 cerb_rad_to_limit_cnt，
// 两侧共用 65536/360 标度（本函数是其逆函数）。
double motor_protocol_cerb_limit_cnt_to_rad(int32_t cnt) {
  const double degrees = (double)cnt * 360.0 / 65536.0;
  return degrees * MOTOR_PROTOCOL_PI / 180.0;
}

// 0x6080：电机端 RPM → 关节侧 rad/s。
double motor_protocol_cerb_motor_rpm_to_joint_rad_s(double gear_ratio,
                                                    uint32_t rpm) {
  if (!(gear_ratio > 0.0)) {
    return NAN;
  }
  return (double)rpm * (2.0 * MOTOR_PROTOCOL_PI / 60.0) / gear_ratio;
}

// 0x2538：电机端 mA → 关节侧 N·m（τ = Iq × Kt × gear × efficiency）。
double motor_protocol_limit_ma_to_joint_nm(double joint_nm_per_amp,
                                           uint32_t milliamp) {
  if (!(joint_nm_per_amp > 0.0)) {
    return 0.0;
  }
  return ((double)milliamp / 1000.0) * joint_nm_per_amp;
}

uint32_t motor_protocol_limit_joint_nm_to_ma(double joint_nm_per_amp,
                                             double joint_nm) {
  if (!(joint_nm_per_amp > 0.0) || !(joint_nm > 0.0)) {
    return 0U;
  }
  return (uint32_t)llround(joint_nm / joint_nm_per_amp * 1000.0);
}

double motor_protocol_cerb_motor_ma_to_joint_nm(double torque_constant,
                                                double gear_ratio,
                                                double gear_efficiency,
                                                uint32_t milliamp) {
  return ((double)milliamp / 1000.0) * torque_constant * gear_ratio *
         gear_efficiency;
}

bool motor_protocol_pack_cerb_mit_frame(
    const motor_protocol_joint_command_t *command,
    const motor_protocol_cerb_limits_t *limits, bool enabled,
    uint8_t out[12]) {
  if (command == NULL || limits == NULL || out == NULL ||
      !isfinite(limits->position_rad) ||
      !isfinite(limits->velocity_rad_s) || !isfinite(limits->torque_nm) ||
      !(limits->position_rad > 0.0) || !(limits->velocity_rad_s > 0.0) ||
      !(limits->torque_nm > 0.0) ||
      !isfinite(2.0 * limits->position_rad) ||
      !isfinite(2.0 * limits->velocity_rad_s) ||
      !isfinite(2.0 * limits->torque_nm) || !isfinite(command->position) ||
      !isfinite(command->velocity) || !isfinite(command->kp) ||
      !isfinite(command->kd) || !isfinite(command->feedforward)) {
    return false;
  }

  uint32_t position = cerb_symmetric_float_to_uint(
      command->position, limits->position_rad, 16U);
  uint32_t velocity = cerb_symmetric_float_to_uint(
      command->velocity, limits->velocity_rad_s, 12U);
  uint32_t kp = motor_protocol_float_to_uint(command->kp, 0.0, 500.0, 12U);
  uint32_t kd = motor_protocol_float_to_uint(command->kd, 0.0, 5.0, 12U);
  uint32_t torque = cerb_symmetric_float_to_uint(
      command->feedforward, limits->torque_nm, 12U);

  // bit[7]=使能，bit[6]=抱闸释放，bit[4:1]=0x06（MIT）。
  out[0] = (uint8_t)((enabled ? 0xc0U : 0x00U) | 0x0cU);
  out[1] = (uint8_t)(position >> 8U);
  out[2] = (uint8_t)position;
  out[3] = (uint8_t)(velocity >> 4U);
  out[4] = (uint8_t)((velocity << 4U) | (kp >> 8U));
  out[5] = (uint8_t)kp;
  out[6] = (uint8_t)(kd >> 4U);
  out[7] = (uint8_t)((kd << 4U) | (torque >> 8U));
  out[8] = (uint8_t)torque;
  out[9] = 0U;
  out[10] = 0U;
  out[11] = 0U;
  return true;
}

bool motor_protocol_unpack_cerb_feedback(
    const uint8_t *data, size_t size, motor_protocol_joint_state_t *state) {
  if (data == NULL || size != 12U || state == NULL ||
      (data[11] & 0x20U) != 0U) {
    return false;
  }
  state->position = read_be_i16(data) / 182.0 * MOTOR_PROTOCOL_PI / 180.0;
  state->velocity = read_be_i16(data + 2U) * 2.0 * MOTOR_PROTOCOL_PI / 60.0;
  state->effort = read_be_i16(data + 4U) / 1000.0;
  return true;
}
