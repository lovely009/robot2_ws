// 控制业务任务（官方 CtrBoard-H7_FDCAN 例程移植）。
//
// 职责：
//   - 调度器启动后初始化三路 FDCAN；
//   - 接收 ROS 侧 JointState，匹配默认测试电机（RP + vaxor）后用
//     motor_protocol 编码下发；
//   - 官方 0x520 测试帧循环保留但默认关闭（ENABLE_CANFD_DEMO_LOOP=0），
//     避免与真实 CAN 命令互相干扰。
//   - 固件暂不维护完整拓扑：kDefaultMotors 只放当前两台测试电机，
//     后续拓扑由参数服务下发后替换。

#include "control_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <string.h>

#include "can_bsp.h"
#include "motor_protocol.h"

#if !defined(ENABLE_CANFD_DEMO_LOOP)
#define ENABLE_CANFD_DEMO_LOOP 0
#endif

// 任务参数：栈 1024 字（4KB），与 sensor 同档，低于 micro_ros 收发任务
static const uint32_t kControlTaskStack = 1024;
static const UBaseType_t kControlTaskPrio = 3;
static const uint16_t kControlStartDelayMs = 500;

enum class MotorProtocolType { kRp, kVaxor };

// 临时默认测试电机：RP 占位 id=1；CAN1 当前实际接入一台 vaxor，节点 ID=2。
// 完整拓扑后续由参数服务下发，这里不扩展为全量关节表。
typedef struct {
  const char *joint_name;
  MotorProtocolType protocol;
  uint8_t id;
  FDCAN_HandleTypeDef *hfdcan;
  double gear_ratio;
  double gear_efficiency;      // vaxor 反馈 effort 关节侧折算；RP 不使用
  double torque_constant_nm_per_amp;  // vaxor 电流→Nm；RP 不使用
  double kp;
  double kd;
  bool enabled;
  uint8_t mode;  // 0=MIT, 1=velocity, 2=position（vaxor）
  motor_protocol_joint_command_t last_command;
  bool has_command;
} control_motor_t;

static control_motor_t kDefaultMotors[] = {
    {"left_shoulder_yaw_joint", MotorProtocolType::kRp, 1, &hfdcan1, 1.0, 1.0, 0.0, 30.0, 1.0, false, CONTROL_MOTOR_MODE_MIT, {}, false},
    {"left_wrist_yaw_joint", MotorProtocolType::kVaxor, 2, &hfdcan1, 50.0, 0.65, 0.042, 0.008, 1.0, false, CONTROL_MOTOR_MODE_MIT, {}, false},
};
static const size_t kDefaultMotorCount =
    sizeof(kDefaultMotors) / sizeof(kDefaultMotors[0]);

volatile uint32_t g_control_motor_cmd_count = 0;
volatile uint32_t g_control_motor_found_count = 0;

static control_motor_t *find_motor(const char *joint_name) {
  for (size_t i = 0; i < kDefaultMotorCount; ++i) {
    if (strcmp(kDefaultMotors[i].joint_name, joint_name) == 0) {
      return &kDefaultMotors[i];
    }
  }
  return nullptr;
}

static bool send_rp_enable(FDCAN_HandleTypeDef *hfdcan, bool enabled) {
  uint8_t data[8] = {0};
  data[0] = enabled ? 1U : 0U;
  return fdcanx_send_frame(hfdcan, MOTOR_PROTOCOL_RP_ENABLE_ID, data, 8,
                           false, false) == 0;
}

static bool send_rp_command(control_motor_t *target,
                            const motor_protocol_joint_command_t *command) {
  uint8_t payload[64];
  motor_protocol_joint_command_t neutral = {};
  uint8_t slot[8];
  motor_protocol_pack_rp_slot(&neutral, slot);
  for (int i = 0; i < MOTOR_PROTOCOL_RP_BROADCAST_SLOTS; ++i) {
    memcpy(payload + i * 8, slot, 8);
  }
  motor_protocol_pack_rp_slot(command, slot);
  memcpy(payload + (target->id - 1U) * 8, slot, 8);
  if (fdcanx_send_frame(target->hfdcan, MOTOR_PROTOCOL_RP_COMMAND_ID, payload,
                        64, false, true) != 0) {
    return false;
  }
  target->last_command = *command;
  target->has_command = true;
  return true;
}

static bool send_vaxor_control(control_motor_t *motor, bool run, uint8_t mode) {
  motor_protocol_wrist_control_t control = {};
  control.run = run;
  control.mode = mode;
  uint8_t data[8];
  motor_protocol_pack_wrist_control(&control, data);
  return fdcanx_send_frame(motor->hfdcan,
                           motor_protocol_wrist_control_id(motor->id), data, 8,
                           false, false) == 0;
}

static bool send_vaxor_command(control_motor_t *motor,
                               const motor_protocol_joint_command_t *command) {
  uint8_t data[12];
  motor_protocol_pack_wrist_command(command, data);
  return fdcanx_send_frame(motor->hfdcan,
                           motor_protocol_wrist_command_id(motor->id), data, 12,
                           false, true) == 0;
}

bool control_motor_command(const control_motor_command_t *command) {
  g_control_motor_cmd_count++;
  if (command == nullptr || command->joint_name[0] == '\0') {
    return false;
  }
  control_motor_t *motor = find_motor(command->joint_name);
  if (motor == nullptr) {
    return false;
  }
  g_control_motor_found_count++;

  if (motor->protocol == MotorProtocolType::kRp) {
    // RP 使能是总线广播；模式字段不适用
    if (!motor->enabled && !send_rp_enable(motor->hfdcan, true)) {
      return false;
    }
    motor->enabled = true;
  } else if (!motor->enabled || motor->mode != command->mode) {
    // vaxor：模式切换先停机（run=0），再按新模式启动
    if (motor->enabled) {
      send_vaxor_control(motor, false, motor->mode);
    }
    if (!send_vaxor_control(motor, true, command->mode)) {
      return false;
    }
    motor->enabled = true;
    motor->mode = command->mode;
  }

  // vaxor：关节侧位置/速度乘减速比换算为电机侧；kp/kd/前馈不变
  motor_protocol_joint_command_t frame_command = {};
  frame_command.position = command->position;
  frame_command.velocity = command->velocity;
  frame_command.kp = motor->kp;
  frame_command.kd = motor->kd;
  frame_command.feedforward = command->effort;
  if (motor->protocol == MotorProtocolType::kVaxor) {
    frame_command.position *= motor->gear_ratio;
    frame_command.velocity *= motor->gear_ratio;
    return send_vaxor_command(motor, &frame_command);
  }
  return send_rp_command(motor, &frame_command);
}

bool control_decode_feedback(uint32_t id, const uint8_t *data, size_t size,
                             control_joint_state_t *out) {
  if (data == NULL || out == NULL) {
    return false;
  }
  for (size_t i = 0; i < kDefaultMotorCount; ++i) {
    control_motor_t *motor = &kDefaultMotors[i];
    motor_protocol_joint_state_t state = {};
    bool matched = false;

    if (motor->protocol == MotorProtocolType::kRp) {
      // RP：标准 ID = 电机 ID，12 Byte 状态，量纲已是关节侧
      if (id == motor->id && size == 12U) {
        matched = motor_protocol_unpack_rp_state(data, size, &state);
      }
    } else {
      // vaxor：reply（12B）/ status（20B）反馈，反馈为电机侧电流 A
      if ((id == motor_protocol_wrist_reply_id(motor->id) && size == 12U) ||
          (id == motor_protocol_wrist_status_id(motor->id) && size == 20U)) {
        matched = motor_protocol_unpack_wrist_feedback(data, size, &state);
        state.position /= motor->gear_ratio;
        state.velocity /= motor->gear_ratio;
        state.effort *= motor->torque_constant_nm_per_amp * motor->gear_ratio *
                        motor->gear_efficiency;
      }
    }

    if (matched) {
      size_t len = strlen(motor->joint_name);
      if (len > sizeof(out->joint_name) - 1U) {
        len = sizeof(out->joint_name) - 1U;
      }
      memcpy(out->joint_name, motor->joint_name, len);
      out->joint_name[len] = '\0';
      out->position = state.position;
      out->velocity = state.velocity;
      out->effort = state.effort;
      return true;
    }
  }
  return false;
}

static void control_task_fn(void *arg) {
  (void)arg;

  // 与官方任务编排一致：调度稳定后再初始化外设
  vTaskDelay(pdMS_TO_TICKS(kControlStartDelayMs));
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  bsp_can_init();

#if ENABLE_CANFD_DEMO_LOOP
  // 官方 main 循环：0x520 + {0..7}，三路各间隔 100ms（默认关闭）
  static const uint16_t kControlTestId = 0x520;
  static const uint8_t kControlTestData[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  while (1) {
    fdcanx_send_data(&hfdcan1, kControlTestId, kControlTestData,
                     sizeof(kControlTestData));
    vTaskDelay(pdMS_TO_TICKS(100));
    fdcanx_send_data(&hfdcan2, kControlTestId, kControlTestData,
                     sizeof(kControlTestData));
    vTaskDelay(pdMS_TO_TICKS(100));
    fdcanx_send_data(&hfdcan3, kControlTestId, kControlTestData,
                     sizeof(kControlTestData));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
#else
  // 电机命令由 micro_ros 任务回调 control_motor_command() 触发，此处保活
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
#endif
}

bool control_task_start(void) {
  return xTaskCreate(control_task_fn, "control", kControlTaskStack, nullptr,
                     kControlTaskPrio, nullptr) == pdPASS;
}
