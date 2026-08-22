// 控制业务任务（官方 CtrBoard-H7_FDCAN 例程移植）。
//
// control task 是正常电机命令的唯一发送者：ROS 回调只覆盖静态 mailbox，
// 本任务按 10 ms 绝对周期消费命令、推进 motor_control 并下发 FDCAN action。
// 固件暂不维护完整拓扑，只保留当前两台默认测试电机。

#include "control_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <string.h>

#include "can_bsp.h"
#include "motor_protocol.h"

#if !defined(ENABLE_CANFD_DEMO_LOOP)
#define ENABLE_CANFD_DEMO_LOOP 0
#endif

// 任务参数：栈 1024 字（4KB），与 sensor 同档，低于 micro_ros 收发任务。
static const uint32_t kControlTaskStack = 1024;
static const UBaseType_t kControlTaskPrio = 3;
static const uint16_t kControlStartDelayMs = 500;
static const uint16_t kControlPeriodMs = 10;

// 临时默认测试电机：RP 占位 id=1；CAN1 当前实际接入 vaxor 节点 ID=2。
// 数值保持与原 control_task 电机表一致，完整拓扑后续由参数服务替换。
static const motor_control_motor_config_t kDefaultMotors[] = {
    {"left_shoulder_yaw_joint", MOTOR_CONTROL_RP, 1, 0,
     1.0, 1.0, 0.0, 30.0, 1.0},
    {"left_wrist_yaw_joint", MOTOR_CONTROL_VAXOR, 2, 0,
     50.0, 0.65, 0.042, 0.008, 1.0},
};
static const size_t kDefaultMotorCount =
    sizeof(kDefaultMotors) / sizeof(kDefaultMotors[0]);
static_assert(kDefaultMotorCount <= MOTOR_CONTROL_MAX_MOTORS,
              "默认电机数量超过 motor_control 固定容量");

static FDCAN_HandleTypeDef *const kCanHandles[] = {
    &hfdcan1,
    &hfdcan2,
    &hfdcan3,
};
static const size_t kCanHandleCount =
    sizeof(kCanHandles) / sizeof(kCanHandles[0]);

typedef struct {
  motor_control_command_t command;
  uint32_t received_ms;
  uint32_t sequence;
  bool pending;
} command_mailbox_t;

static motor_control_context_t g_motor_control;
static command_mailbox_t g_command_mailboxes[kDefaultMotorCount];
static control_joint_state_t g_joint_states[kDefaultMotorCount];
static bool g_joint_state_valid[kDefaultMotorCount];
static uint32_t g_command_sequence = 0;
static uint32_t g_last_can1_rx_sequence = 0;

// ROS/link 任务只覆盖请求；motor_control_context_t 始终由 control task 修改。
static bool g_authorization_requested = true;
static bool g_authorization_pending = false;
static bool g_authorization_disable_pending = false;
static bool g_agent_state_seen = false;
static bool g_agent_connected_requested = false;
static bool g_agent_state_pending = false;
static bool g_agent_disconnect_pending = false;
static bool g_task_started = false;

volatile uint32_t g_control_motor_cmd_count = 0;
volatile uint32_t g_control_motor_found_count = 0;
static volatile uint32_t g_control_tx_ok_count = 0;
static volatile uint32_t g_control_tx_error_count = 0;

static int find_motor_index(const char *joint_name) {
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    if (strcmp(kDefaultMotors[index].joint_name, joint_name) == 0) {
      return (int)index;
    }
  }
  return -1;
}

static void clear_pending_commands_locked() {
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    g_command_mailboxes[index].pending = false;
  }
}

bool control_task_submit_command(const motor_control_command_t *command,
                                 uint32_t received_ms) {
  ++g_control_motor_cmd_count;
  if (command == nullptr || command->joint_name[0] == '\0' ||
      memchr(command->joint_name, '\0', sizeof(command->joint_name)) ==
          nullptr) {
    return false;
  }
  int index = find_motor_index(command->joint_name);
  if (index < 0) {
    return false;
  }

  taskENTER_CRITICAL();
  // 禁用/断连期间不积压命令，保证重新授权后必须收到新命令。
  bool accepting = g_authorization_requested &&
                   (!g_agent_state_seen || g_agent_connected_requested);
  if (accepting) {
    command_mailbox_t *mailbox = &g_command_mailboxes[index];
    mailbox->command = *command;
    mailbox->received_ms = received_ms;
    mailbox->sequence = ++g_command_sequence;
    mailbox->pending = true;
    ++g_control_motor_found_count;
  }
  taskEXIT_CRITICAL();
  return accepting;
}

void control_task_set_authorization(bool authorized) {
  taskENTER_CRITICAL();
  g_authorization_requested = authorized;
  g_authorization_pending = true;
  if (!authorized) {
    g_authorization_disable_pending = true;
    clear_pending_commands_locked();
  }
  taskEXIT_CRITICAL();
}

void control_task_set_agent_connected(bool connected) {
  taskENTER_CRITICAL();
  bool disconnected_after_connection =
      !connected && g_agent_state_seen && g_agent_connected_requested;
  g_agent_state_seen = true;
  g_agent_connected_requested = connected;
  g_agent_state_pending = true;
  if (!connected) {
    g_agent_disconnect_pending = true;
    clear_pending_commands_locked();
    if (disconnected_after_connection) {
      // 与 motor_control 断连语义对齐：重连后必须显式重新授权。
      g_authorization_requested = false;
    }
  }
  taskEXIT_CRITICAL();
}

bool control_task_get_joint_state(size_t index, control_joint_state_t *out) {
  if (index >= kDefaultMotorCount || out == nullptr) {
    return false;
  }
  taskENTER_CRITICAL();
  bool valid = g_joint_state_valid[index];
  if (valid) {
    *out = g_joint_states[index];
  }
  taskEXIT_CRITICAL();
  return valid;
}

static void drain_control_requests() {
  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_pending;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_pending;

  taskENTER_CRITICAL();
  authorization_requested = g_authorization_requested;
  authorization_pending = g_authorization_pending;
  authorization_disable_pending = g_authorization_disable_pending;
  agent_connected_requested = g_agent_connected_requested;
  agent_state_pending = g_agent_state_pending;
  agent_disconnect_pending = g_agent_disconnect_pending;
  g_authorization_pending = false;
  g_authorization_disable_pending = false;
  g_agent_state_pending = false;
  g_agent_disconnect_pending = false;
  taskEXIT_CRITICAL();

  // 快速 false->true 转换也必须先执行失效动作，不能遗漏断连/禁用边沿。
  if (agent_disconnect_pending) {
    motor_control_set_agent_connected(&g_motor_control, false);
  }
  if (agent_state_pending && agent_connected_requested) {
    motor_control_set_agent_connected(&g_motor_control, true);
  }
  if (authorization_disable_pending) {
    (void)motor_control_set_authorization(&g_motor_control, false);
  }
  if (authorization_pending && authorization_requested) {
    (void)motor_control_set_authorization(&g_motor_control, true);
  }
}

static void drain_command_mailboxes() {
  command_mailbox_t pending[kDefaultMotorCount] = {};

  taskENTER_CRITICAL();
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    if (g_command_mailboxes[index].pending) {
      pending[index] = g_command_mailboxes[index];
      g_command_mailboxes[index].pending = false;
    }
  }
  taskEXIT_CRITICAL();

  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    if (pending[index].pending) {
      (void)motor_control_accept_command(&g_motor_control,
                                         &pending[index].command,
                                         pending[index].received_ms);
    }
  }
}

static bool decode_feedback(uint32_t id, const uint8_t *data, size_t size,
                            size_t *motor_index,
                            control_joint_state_t *out) {
  if (data == nullptr || motor_index == nullptr || out == nullptr) {
    return false;
  }
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    const motor_control_motor_config_t *motor = &kDefaultMotors[index];
    motor_protocol_joint_state_t state = {};
    bool matched = false;

    if (motor->protocol == MOTOR_CONTROL_RP) {
      if (id == motor->id && size == 12U) {
        matched = motor_protocol_unpack_rp_state(data, size, &state);
      }
    } else if ((id == motor_protocol_wrist_reply_id(motor->id) &&
                size == 12U) ||
               (id == motor_protocol_wrist_status_id(motor->id) &&
                size == 20U)) {
      matched = motor_protocol_unpack_wrist_feedback(data, size, &state);
      state.position /= motor->gear_ratio;
      state.velocity /= motor->gear_ratio;
      state.effort *= motor->torque_constant_nm_per_amp * motor->gear_ratio *
                      motor->gear_efficiency;
    }

    if (matched) {
      memset(out, 0, sizeof(*out));
      size_t name_size = strlen(motor->joint_name) + 1U;
      memcpy(out->joint_name, motor->joint_name, name_size);
      out->position = state.position;
      out->velocity = state.velocity;
      out->effort = state.effort;
      *motor_index = index;
      return true;
    }
  }
  return false;
}

// 当前 fdcan 层只提供 CAN1 最新快照；CAN RX queue 留给后续任务实现。
static void drain_can_rx_snapshot() {
  can_rx_frame_t frame = {};
  uint32_t sequence = can1_rx_snapshot(&frame);
  if (sequence == g_last_can1_rx_sequence) {
    return;
  }
  g_last_can1_rx_sequence = sequence;

  size_t motor_index = 0;
  control_joint_state_t state = {};
  if (!decode_feedback(frame.id, frame.data, frame.dlc, &motor_index, &state)) {
    return;
  }
  taskENTER_CRITICAL();
  g_joint_states[motor_index] = state;
  g_joint_state_valid[motor_index] = true;
  taskEXIT_CRITICAL();
}

static void apply_tx_actions(const motor_control_tx_action_t *actions,
                             size_t action_count) {
  for (size_t index = 0; index < action_count; ++index) {
    const motor_control_tx_action_t *action = &actions[index];
    if (action->bus_index >= kCanHandleCount || action->size > 64U) {
      ++g_control_tx_error_count;
      continue;
    }
    uint8_t rc = fdcanx_send_frame(
        kCanHandles[action->bus_index], action->can_id, action->data,
        action->size, action->is_extended, action->is_fd);
    if (rc == 0U) {
      ++g_control_tx_ok_count;
    } else {
      ++g_control_tx_error_count;
    }
  }
}

static void run_motor_control_cycle(uint32_t now_ms) {
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS] = {};
  size_t action_count = motor_control_step(
      &g_motor_control, now_ms, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  apply_tx_actions(actions, action_count);
}

#if ENABLE_CANFD_DEMO_LOOP
static void run_canfd_demo_cycle() {
  // 保留官方 0x520 调试模式：三路依次间隔 100 ms 发送。
  static const uint16_t kControlTestId = 0x520;
  static const uint8_t kControlTestData[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  static uint8_t bus_index = 0;
  static uint8_t period_count = 0;
  if (period_count++ < 10U) {
    return;
  }
  period_count = 0;
  (void)fdcanx_send_data(kCanHandles[bus_index], kControlTestId,
                         kControlTestData, sizeof(kControlTestData));
  bus_index = (uint8_t)((bus_index + 1U) % kCanHandleCount);
}
#endif

static void control_task_fn(void *arg) {
  (void)arg;

  // 与官方任务编排一致：调度稳定后再初始化外设。
  vTaskDelay(pdMS_TO_TICKS(kControlStartDelayMs));
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  bsp_can_init();

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    drain_control_requests();
    drain_command_mailboxes();
    drain_can_rx_snapshot();
    run_motor_control_cycle(millis());
#if ENABLE_CANFD_DEMO_LOOP
    run_canfd_demo_cycle();
#endif
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

bool control_task_start(void) {
  taskENTER_CRITICAL();
  if (g_task_started) {
    taskEXIT_CRITICAL();
    return true;
  }
  g_task_started = true;
  taskEXIT_CRITICAL();

  motor_control_init(&g_motor_control, kDefaultMotors, kDefaultMotorCount);
  if (xTaskCreate(control_task_fn, "control", kControlTaskStack, nullptr,
                  kControlTaskPrio, nullptr) == pdPASS) {
    return true;
  }

  taskENTER_CRITICAL();
  g_task_started = false;
  taskEXIT_CRITICAL();
  return false;
}
