// 控制业务任务接口（官方 CtrBoard-H7_FDCAN 例程移植）。
// 当前承载三路 FDCAN 初始化与 JointState 下发；固件只内置两个默认测试电机
// （RP + vaxor），完整拓扑后续通过参数服务下发，不在此扩展。

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// vaxor 控制模式（与 motor_protocol 一致）
#define CONTROL_MOTOR_MODE_MIT 0
#define CONTROL_MOTOR_MODE_VELOCITY 1
#define CONTROL_MOTOR_MODE_POSITION 2

#ifdef __cplusplus
extern "C" {
#endif

// 创建控制任务（只能调用一次；重复调用返回 true 且不重复建任务）
bool control_task_start(void);

// 单关节命令（由 sensor_msgs/JointState 展开；kp/kd 用默认测试电机参数）
// mode：只发 position -> 位置模式；只发 velocity -> 速度模式；只发 effort -> MIT
typedef struct {
  char joint_name[32];
  uint8_t mode;
  double position;
  double velocity;
  double effort;  // RP: Nm 前馈；vaxor: 电流 A 前馈
} control_motor_command_t;

// 按 joint_name 匹配默认测试电机并下发协议帧；未使能时先自动使能。
// 返回是否找到该电机
bool control_motor_command(const control_motor_command_t *command);

// 关节反馈（sensor_msgs/JointState 同语义）
typedef struct {
  char joint_name[32];
  double position;
  double velocity;
  double effort;
} control_joint_state_t;

// 尝试把一帧 CAN 反馈解析为默认测试电机状态；返回是否匹配
bool control_decode_feedback(uint32_t id, const uint8_t *data, size_t size,
                             control_joint_state_t *out);

// 调试计数：回调收到 JointState 次数 / 匹配到默认电机次数（GDB 可读）
extern volatile uint32_t g_control_motor_cmd_count;
extern volatile uint32_t g_control_motor_found_count;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_TASK_H
