// 电机协议编解码（vaxor 25/30mm 唯一支持协议）。
// RP 协议已弃用，固件库中不再保留 RP 编解码实现。

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

uint32_t motor_protocol_float_to_uint(double value, double low, double high,
                                      unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    return 0U;
  }
  value = clamp_double(value, low, high);
  uint32_t maximum = (UINT32_C(1) << bits) - 1U;
  return (uint32_t)((value - low) * maximum / (high - low));
}

double motor_protocol_uint_to_float(uint32_t value, double low, double high,
                                    unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    return 0.0;
  }
  uint32_t maximum = (UINT32_C(1) << bits) - 1U;
  return value * (high - low) / maximum + low;
}

static uint16_t wrist_velocity_to_code(double velocity_rad_s) {
  int32_t rpm10 =
      (int32_t)llround(velocity_rad_s * 60.0 / (2.0 * MOTOR_PROTOCOL_PI) * 10.0);
  rpm10 = clamp_i32(rpm10, -MOTOR_PROTOCOL_WRIST_VELOCITY_CODE_LIMIT,
                    MOTOR_PROTOCOL_WRIST_VELOCITY_CODE_LIMIT);
  return (uint16_t)(rpm10 + 0x8000);
}

static double wrist_code_to_velocity(uint16_t code) {
  int32_t raw = (int32_t)code - 0x8000;
  return raw / 10.0 * 2.0 * MOTOR_PROTOCOL_PI / 60.0;
}

static uint32_t wrist_position_to_code(double position_rad) {
  int64_t ticks = llround(position_rad / (2.0 * MOTOR_PROTOCOL_PI) *
                          (double)MOTOR_PROTOCOL_WRIST_TICKS_PER_REV);
  ticks = clamp_i64(ticks, -MOTOR_PROTOCOL_WRIST_POSITION_TICK_LIMIT,
                    MOTOR_PROTOCOL_WRIST_POSITION_TICK_LIMIT);
  return (uint32_t)(ticks + INT64_C(0x80000000));
}

static double wrist_code_to_position(uint32_t code) {
  return ((int64_t)code - INT64_C(0x80000000)) /
         (double)MOTOR_PROTOCOL_WRIST_TICKS_PER_REV * 2.0 * MOTOR_PROTOCOL_PI;
}

static uint16_t wrist_current_to_code(double current_a) {
  int32_t raw =
      (int32_t)llround(current_a / MOTOR_PROTOCOL_WRIST_CURRENT_SCALE_A);
  raw = clamp_i32(raw, -32768, 32767);
  return (uint16_t)(raw + 0x8000);
}

static double wrist_code_to_current(uint16_t code) {
  return ((int32_t)code - 0x8000) * MOTOR_PROTOCOL_WRIST_CURRENT_SCALE_A;
}

bool motor_protocol_wrist_node_valid(unsigned node) {
  return node >= 1U && node <= 60U;
}

uint32_t motor_protocol_wrist_command_id(unsigned node) {
  return 0x501U + 4U * node;
}

uint32_t motor_protocol_wrist_control_id(unsigned node) {
  return 0x502U + 4U * node;
}

uint32_t motor_protocol_wrist_reply_id(unsigned node) {
  return 0x701U + 4U * node;
}

uint32_t motor_protocol_wrist_status_id(unsigned node) {
  return 0x702U + 4U * node;
}

void motor_protocol_pack_wrist_command(const motor_protocol_joint_command_t *command,
                                       uint8_t out[12]) {
  uint32_t position = wrist_position_to_code(command->position);
  uint16_t velocity = wrist_velocity_to_code(command->velocity);
  uint16_t kp = (uint16_t)clamp_i64(
      llround(command->kp / MOTOR_PROTOCOL_WRIST_KP_SCALE), 0, 0xffff);
  uint16_t kd = (uint16_t)clamp_i64(
      llround(command->kd / MOTOR_PROTOCOL_WRIST_KD_SCALE), 0, 0xffff);
  uint16_t current = wrist_current_to_code(command->feedforward);
  write_le(out, position, 4U);
  write_le(out + 4U, velocity, 2U);
  write_le(out + 6U, kp, 2U);
  write_le(out + 8U, kd, 2U);
  write_le(out + 10U, current, 2U);
}

bool motor_protocol_unpack_wrist_command(const uint8_t *data, size_t size,
                                         motor_protocol_joint_command_t *command) {
  if (data == NULL || size != 12U || command == NULL) {
    return false;
  }
  uint32_t position = read_le(data, 4U);
  uint16_t velocity = (uint16_t)read_le(data + 4U, 2U);
  uint16_t kp = (uint16_t)read_le(data + 6U, 2U);
  uint16_t kd = (uint16_t)read_le(data + 8U, 2U);
  uint16_t current = (uint16_t)read_le(data + 10U, 2U);
  command->position = wrist_code_to_position(position);
  command->velocity = wrist_code_to_velocity(velocity);
  command->kp = kp * MOTOR_PROTOCOL_WRIST_KP_SCALE;
  command->kd = kd * MOTOR_PROTOCOL_WRIST_KD_SCALE;
  command->feedforward = wrist_code_to_current(current);
  return true;
}

void motor_protocol_pack_wrist_control(const motor_protocol_wrist_control_t *control,
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

bool motor_protocol_unpack_wrist_control(const uint8_t *data, size_t size,
                                         motor_protocol_wrist_control_t *control) {
  if (data == NULL || size != 8U || control == NULL) {
    return false;
  }
  if (data[2] > (uint8_t)MOTOR_PROTOCOL_WRIST_MODE_POSITION) {
    return false;
  }
  control->run = data[0] != 0U;
  control->set_zero = data[1] != 0U;
  control->mode = data[2];
  return true;
}

void motor_protocol_pack_wrist_reply(unsigned node,
                                     const motor_protocol_joint_state_t *state,
                                     uint8_t out[12]) {
  write_le(out, motor_protocol_wrist_command_id(node), 2U);
  write_le(out + 2U, wrist_position_to_code(state->position), 4U);
  write_le(out + 6U, wrist_velocity_to_code(state->velocity), 2U);
  write_le(out + 8U, wrist_current_to_code(state->effort), 2U);
}

void motor_protocol_pack_wrist_status(unsigned node,
                                      const motor_protocol_joint_state_t *state,
                                      double voltage_v, uint32_t fault,
                                      uint8_t motor_state,
                                      uint8_t calibration_state, uint8_t mode,
                                      uint8_t motor_type, uint8_t out[20]) {
  write_le(out, motor_protocol_wrist_command_id(node), 2U);
  write_le(out + 2U, wrist_position_to_code(state->position), 4U);
  write_le(out + 6U, wrist_velocity_to_code(state->velocity), 2U);
  write_le(out + 8U, wrist_current_to_code(state->effort), 2U);
  int32_t voltage_code =
      clamp_i32((int32_t)llround(voltage_v * 10.0), -32768, 32767);
  write_le(out + 10U, (uint16_t)(voltage_code + 0x8000), 2U);
  write_le(out + 12U, fault, 4U);
  out[16] = motor_state;
  out[17] = calibration_state;
  out[18] = mode;
  out[19] = motor_type;
}

bool motor_protocol_unpack_wrist_feedback(const uint8_t *data, size_t size,
                                          motor_protocol_joint_state_t *state) {
  if (data == NULL || (size != 12U && size != 20U) || state == NULL) {
    return false;
  }
  uint32_t position = read_le(data + 2U, 4U);
  uint16_t velocity = (uint16_t)read_le(data + 6U, 2U);
  uint16_t current = (uint16_t)read_le(data + 8U, 2U);
  state->position = wrist_code_to_position(position);
  state->velocity = wrist_code_to_velocity(velocity);
  state->effort = wrist_code_to_current(current);
  return true;
}

double motor_protocol_wrist_kp_to_current_gain(double protocol_kp) {
  return protocol_kp / 0.001115;
}

double motor_protocol_wrist_kd_to_current_gain(double protocol_kd) {
  return protocol_kd / 85.636;
}
