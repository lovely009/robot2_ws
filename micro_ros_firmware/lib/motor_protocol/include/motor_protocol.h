// 电机协议编解码（vaxor 25/30mm + CERB CAN FD/MIT）。

#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PROTOCOL_PI 3.14159265358979323846

// ---------------- vaxor（25/30mm）协议常量 ----------------
// 码值比例依据《25-30mm 驱控一体使用说明手册》V20260204 §3
#define MOTOR_PROTOCOL_VAXOR_TICKS_PER_REV 2097152LL
#define MOTOR_PROTOCOL_VAXOR_POSITION_TICK_LIMIT 0x7cffffffL
#define MOTOR_PROTOCOL_VAXOR_VELOCITY_CODE_LIMIT 30000
#define MOTOR_PROTOCOL_VAXOR_CURRENT_SCALE_A (11.0 / 4096.0)
#define MOTOR_PROTOCOL_VAXOR_KP_SCALE 0.00001
#define MOTOR_PROTOCOL_VAXOR_KD_SCALE 0.01

typedef enum {
  MOTOR_PROTOCOL_MODE_MIT = 0,
  MOTOR_PROTOCOL_MODE_VELOCITY = 1,
  MOTOR_PROTOCOL_MODE_POSITION = 2,
} motor_protocol_mode_t;

typedef struct {
  double position;
  double velocity;
  double kp;
  double kd;
  double feedforward;
} motor_protocol_joint_command_t;

typedef struct {
  double position;
  double velocity;
  double effort;
} motor_protocol_joint_state_t;

typedef struct {
  double voltage_v;
  uint32_t fault;
  uint8_t motor_state;
  uint8_t calibration_state;
  uint8_t mode;
  uint8_t motor_type;
} motor_protocol_vaxor_status_t;

typedef struct {
  double position_rad;
  double velocity_rad_s;
  double torque_nm;
} motor_protocol_cerb_limits_t;

typedef struct {
  bool run;
  bool set_zero;
  uint8_t mode;
} motor_protocol_vaxor_control_t;

uint32_t motor_protocol_float_to_uint(double value, double low, double high,
                                      unsigned bits);
double motor_protocol_uint_to_float(uint32_t value, double low, double high,
                                    unsigned bits);

// 节点 ID：0x501/0x502/0x701/0x702 + 4*node（node 合法范围 1..60）
bool motor_protocol_vaxor_node_valid(unsigned node);
uint32_t motor_protocol_vaxor_command_id(unsigned node);
uint32_t motor_protocol_vaxor_control_id(unsigned node);
uint32_t motor_protocol_vaxor_reply_id(unsigned node);
uint32_t motor_protocol_vaxor_status_id(unsigned node);

void motor_protocol_pack_vaxor_command(const motor_protocol_joint_command_t *command,
                                       uint8_t out[12]);
bool motor_protocol_unpack_vaxor_command(const uint8_t *data, size_t size,
                                         motor_protocol_joint_command_t *command);
void motor_protocol_pack_vaxor_control(const motor_protocol_vaxor_control_t *control,
                                       uint8_t out[8]);
bool motor_protocol_unpack_vaxor_control(const uint8_t *data, size_t size,
                                         motor_protocol_vaxor_control_t *control);
void motor_protocol_pack_vaxor_reply(unsigned node,
                                     const motor_protocol_joint_state_t *state,
                                     uint8_t out[12]);
void motor_protocol_pack_vaxor_status(unsigned node,
                                      const motor_protocol_joint_state_t *state,
                                      double voltage_v, uint32_t fault,
                                      uint8_t motor_state,
                                      uint8_t calibration_state, uint8_t mode,
                                      uint8_t motor_type, uint8_t out[20]);
bool motor_protocol_unpack_vaxor_feedback(const uint8_t *data, size_t size,
                                          motor_protocol_joint_state_t *state);
bool motor_protocol_unpack_vaxor_status(
    const uint8_t *data, size_t size, motor_protocol_vaxor_status_t *status);

// ---------------- CERB 自定义 PDO / MIT ----------------
// 新版协议中的 ID 基址按“加节点号”解释，避免与 Dev_ID 位重叠时产生碰撞。
bool motor_protocol_cerb_node_valid(unsigned node);
bool motor_protocol_cerb_command_id(unsigned node, uint32_t *id);
bool motor_protocol_cerb_feedback_id(unsigned node, uint32_t *id);

// 输出 CAN FD DLC=9 对应的 12 Byte 载荷：控制字 + 8 Byte MIT + 3 Byte 零填充。
bool motor_protocol_pack_cerb_mit_frame(
    const motor_protocol_joint_command_t *command,
    const motor_protocol_cerb_limits_t *limits, bool enabled,
    uint8_t out[12]);

// 解码位置(rad)、电机端速度(rad/s)和 Iq(A)；故障状态反馈返回 false。
bool motor_protocol_unpack_cerb_feedback(
    const uint8_t *data, size_t size, motor_protocol_joint_state_t *state);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_PROTOCOL_H
