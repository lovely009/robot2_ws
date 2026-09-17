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

// 节点 ID：base + stride*node（node 合法范围 1..60）。基址即 node 0 槽位，
// 因 node 0 非法而不冲突。
//
// 以下常量是 vaxor CAN-ID 推导的唯一来源：本文件的 *_id() 与 adapter 的
// 接收匹配均须引用，勿在别处重新定义字面量（历史上 vaxor_adapter.c 曾
// 自带一份 0x701/0x702 副本并与取模算术重复实现）。
#define MOTOR_PROTOCOL_VAXOR_COMMAND_ID_BASE 0x501U
#define MOTOR_PROTOCOL_VAXOR_CONTROL_ID_BASE 0x502U
#define MOTOR_PROTOCOL_VAXOR_REPLY_ID_BASE 0x701U
#define MOTOR_PROTOCOL_VAXOR_STATUS_ID_BASE 0x702U
#define MOTOR_PROTOCOL_VAXOR_NODE_STRIDE 4U

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
// 与 vaxor 同理：以下常量是 CERB CAN-ID 推导的唯一来源（历史上
// cerb_adapter.c 曾自带一份 0x300/0x110 副本并内联重算命令 ID）。
#define MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE 0x110U
#define MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE 0x300U
#define MOTOR_PROTOCOL_CERB_SYNC_ID 0x080U
// CANopen SDO（《CANopen-V0.9 字典表》§SDO 协议内容详解）：
//   请求 0x600|node，响应 0x580|node，DLC 恒 8，CANFD 加速标准帧
//   （说明书 §3.1：1M 仲裁段 / 5M 数据段）。
//   管理对象均 ≤4 字节，走快速（expedited）SDO；设备身份对象 0x1008 超
//   4 字节，走分段（segmented）上传。
#define MOTOR_PROTOCOL_CERB_SDO_REQUEST_ID_BASE 0x600U
#define MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE 0x580U

bool motor_protocol_cerb_node_valid(unsigned node);
bool motor_protocol_cerb_command_id(unsigned node, uint32_t *id);
bool motor_protocol_cerb_feedback_id(unsigned node, uint32_t *id);

// SDO 管理对象反向换算（设备影子回读，REFRESH_PARAMS 事务用）：
// 0x607D cnt（负载端，65536 cnt/360°）→ 关节侧 rad；
// 0x6080 电机端 RPM → 关节侧 rad/s（÷gear）；0x2538 电机端 mA → 关节侧 N·m。
double motor_protocol_cerb_limit_cnt_to_rad(int32_t cnt);
double motor_protocol_cerb_motor_rpm_to_joint_rad_s(double gear_ratio,
                                                    uint32_t rpm);
// 限值对象专用换算（0x2538 电流上限 ↔ 关节侧扭矩），系数 = 关节侧 N·m/A，
// 由 motor_protocol_profile_limit_nm_per_amp() 给出（规格书峰值对优先）。
// 与反馈换算分开：反馈是"电流 → 实测扭矩估计"（走 Kt 链），限值是"规格书
// 峰值对的两端映射"，规格书自身额定/峰值两点隐含效率不同，不能共用一个常数。
double motor_protocol_limit_ma_to_joint_nm(double joint_nm_per_amp,
                                           uint32_t milliamp);
uint32_t motor_protocol_limit_joint_nm_to_ma(double joint_nm_per_amp,
                                             double joint_nm);

double motor_protocol_cerb_motor_ma_to_joint_nm(double torque_constant,
                                                double gear_ratio,
                                                double gear_efficiency,
                                                uint32_t milliamp);

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
