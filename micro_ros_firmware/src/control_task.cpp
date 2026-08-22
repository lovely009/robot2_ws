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
#include "control_mailbox.h"
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

static motor_control_context_t g_motor_control;
static control_mailbox_t g_control_mailbox;
static control_joint_state_t g_joint_states[kDefaultMotorCount];
static bool g_joint_state_valid[kDefaultMotorCount];
static uint32_t g_last_can1_rx_sequence = 0;

// ROS/link 任务只覆盖请求；motor_control_context_t 始终由 control task 修改。
static bool g_mailbox_initialized = false;
static bool g_task_started = false;

volatile uint32_t g_control_motor_cmd_count = 0;
volatile uint32_t g_control_motor_found_count = 0;
volatile uint32_t g_control_debug_submit_count = 0;
volatile uint32_t g_control_debug_overwrite_count = 0;
volatile uint32_t g_control_debug_rejected_count = 0;
volatile uint32_t g_control_debug_sent_count = 0;
static volatile uint32_t g_control_tx_ok_count = 0;
static volatile uint32_t g_control_tx_error_count = 0;

static void ensure_mailbox_initialized_locked() {
  if (!g_mailbox_initialized) {
    control_mailbox_init(&g_control_mailbox, kDefaultMotors,
                         kDefaultMotorCount);
    g_mailbox_initialized = true;
  }
}

bool control_task_submit_command(const motor_control_command_t *command,
                                 uint32_t received_ms) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  ++g_control_motor_cmd_count;
  bool accepted = control_mailbox_submit_command(
      &g_control_mailbox, command, received_ms);
  if (accepted) {
    ++g_control_motor_found_count;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool control_task_submit_debug_frame(const motor_control_tx_action_t *frame) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  bool accepted =
      control_mailbox_submit_debug_frame(&g_control_mailbox, frame);
  g_control_debug_submit_count = g_control_mailbox.debug_submit_count;
  g_control_debug_overwrite_count = g_control_mailbox.debug_overwrite_count;
  if (!accepted) {
    ++g_control_debug_rejected_count;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void control_task_set_authorization(bool authorized) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  control_mailbox_request_authorization(&g_control_mailbox, authorized);
  taskEXIT_CRITICAL();
}

void control_task_set_agent_connected(bool connected) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  control_mailbox_request_agent_connected(&g_control_mailbox, connected);
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
  control_mailbox_events_t events = {};

  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  (void)control_mailbox_take_events(&g_control_mailbox, &events);
  taskEXIT_CRITICAL();

  // 快速 false->true 转换也必须先执行失效动作，不能遗漏断连/禁用边沿。
  if (events.agent_disconnect_edge) {
    motor_control_set_agent_connected(&g_motor_control, false);
  }
  if (events.agent_state_pending && events.agent_connected_requested) {
    motor_control_set_agent_connected(&g_motor_control, true);
  }
  if (events.authorization_disable_edge) {
    (void)motor_control_set_authorization(&g_motor_control, false);
  }
  if (events.authorization_pending && events.authorization_requested) {
    (void)motor_control_set_authorization(&g_motor_control, true);
  }
}

static void drain_command_mailboxes() {
  control_mailbox_command_slot_t pending[kDefaultMotorCount] = {};

  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    pending[index].pending = control_mailbox_take_command(
        &g_control_mailbox, index, &pending[index].command,
        &pending[index].received_ms, &pending[index].sequence);
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

static size_t apply_tx_actions(const motor_control_tx_action_t *actions,
                               size_t action_count) {
  size_t success_count = 0;
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
      ++success_count;
    } else {
      ++g_control_tx_error_count;
    }
  }
  return success_count;
}

static void run_motor_control_cycle(uint32_t now_ms) {
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS] = {};
  size_t action_count = motor_control_step(
      &g_motor_control, now_ms, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  (void)apply_tx_actions(actions, action_count);
}

static void run_debug_tx_cycle() {
  motor_control_tx_action_t frame = {};
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  bool pending = control_mailbox_take_debug_frame(
      &g_control_mailbox, &frame, nullptr);
  taskEXIT_CRITICAL();
  if (pending) {
    g_control_debug_sent_count += apply_tx_actions(&frame, 1U);
  }
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
    run_debug_tx_cycle();
#if ENABLE_CANFD_DEMO_LOOP
    run_canfd_demo_cycle();
#endif
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

bool control_task_start(void) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
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
