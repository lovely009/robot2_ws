// 电机协议编解码（从 ros2_ws/src/motor_drive 的 protocol.hpp/cpp 移植）。
//
// 单个文件内按两部分组织，便于阅读与维护：
//   1) RP 协议：8 Byte MIT 槽位、12 Byte 状态、64 Byte 广播命令（CAN ID 0x80）
//   2) vaxor（25/30mm）协议：12 Byte MIT 命令、8 Byte 控制、12/20 Byte 反馈
// 原实现是 C++（std::array + 异常）；本工程用 -fno-exceptions，故改为 C 接口，
// 无效输入返回 false / 0，字节布局与数值范围保持逐字节一致。

#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PROTOCOL_PI 3.14159265358979323846

// ---------------- RP 协议常量 ----------------
#define MOTOR_PROTOCOL_RP_ENABLE_ID 0x00U
#define MOTOR_PROTOCOL_RP_COMMAND_ID 0x80U
#define MOTOR_PROTOCOL_RP_BROADCAST_SLOTS 8U

// ---------------- vaxor（25/30mm）协议常量 ----------------
// 码值比例依据《25-30mm 驱控一体使用说明手册》V20260204 §3
#define MOTOR_PROTOCOL_WRIST_TICKS_PER_REV 2097152LL
#define MOTOR_PROTOCOL_WRIST_POSITION_TICK_LIMIT 0x7cffffffL
#define MOTOR_PROTOCOL_WRIST_VELOCITY_CODE_LIMIT 30000
#define MOTOR_PROTOCOL_WRIST_CURRENT_SCALE_A (11.0 / 4096.0)
#define MOTOR_PROTOCOL_WRIST_KP_SCALE 0.00001
#define MOTOR_PROTOCOL_WRIST_KD_SCALE 0.01

typedef enum {
  MOTOR_PROTOCOL_WRIST_MODE_MIT = 0,
  MOTOR_PROTOCOL_WRIST_MODE_VELOCITY = 1,
  MOTOR_PROTOCOL_WRIST_MODE_POSITION = 2,
} motor_protocol_wrist_mode_t;

// 统一关节量：rad、rad/s、A/Nm（与 ros2_ws protocol.hpp 同语义）
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
  bool run;
  bool set_zero;
  uint8_t mode;
} motor_protocol_wrist_control_t;

// 公共：RP MIT 定点映射（内部限幅后线性映射到指定 bit 宽度）
uint32_t motor_protocol_float_to_uint(double value, double low, double high,
                                      unsigned bits);
double motor_protocol_uint_to_float(uint32_t value, double low, double high,
                                    unsigned bits);

// ---------------- RP 协议 ----------------
void motor_protocol_pack_rp_slot(const motor_protocol_joint_command_t *command,
                                 uint8_t out[8]);
bool motor_protocol_unpack_rp_slot(const uint8_t *data, size_t size,
                                   motor_protocol_joint_command_t *command);
void motor_protocol_pack_rp_state(const motor_protocol_joint_state_t *state,
                                  double mos_temperature_c, uint16_t status,
                                  double voltage_v, double coil_temperature_c,
                                  uint8_t out[12]);
bool motor_protocol_unpack_rp_state(const uint8_t *data, size_t size,
                                    motor_protocol_joint_state_t *state);

// ---------------- vaxor（25/30mm）协议 ----------------
// 节点 ID：0x501/0x502/0x701/0x702 + 4*node（node 合法范围 1..60）
bool motor_protocol_wrist_node_valid(unsigned node);
uint32_t motor_protocol_wrist_command_id(unsigned node);
uint32_t motor_protocol_wrist_control_id(unsigned node);
uint32_t motor_protocol_wrist_reply_id(unsigned node);
uint32_t motor_protocol_wrist_status_id(unsigned node);

void motor_protocol_pack_wrist_command(const motor_protocol_joint_command_t *command,
                                       uint8_t out[12]);
bool motor_protocol_unpack_wrist_command(const uint8_t *data, size_t size,
                                         motor_protocol_joint_command_t *command);
void motor_protocol_pack_wrist_control(const motor_protocol_wrist_control_t *control,
                                       uint8_t out[8]);
bool motor_protocol_unpack_wrist_control(const uint8_t *data, size_t size,
                                         motor_protocol_wrist_control_t *control);
void motor_protocol_pack_wrist_reply(unsigned node,
                                     const motor_protocol_joint_state_t *state,
                                     uint8_t out[12]);
void motor_protocol_pack_wrist_status(unsigned node,
                                      const motor_protocol_joint_state_t *state,
                                      double voltage_v, uint32_t fault,
                                      uint8_t motor_state,
                                      uint8_t calibration_state, uint8_t mode,
                                      uint8_t motor_type, uint8_t out[20]);
bool motor_protocol_unpack_wrist_feedback(const uint8_t *data, size_t size,
                                          motor_protocol_joint_state_t *state);

// vaxor Kp/Kd 协议码 -> 电流增益换算
double motor_protocol_wrist_kp_to_current_gain(double protocol_kp);
double motor_protocol_wrist_kd_to_current_gain(double protocol_kd);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_PROTOCOL_H
