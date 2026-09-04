// 控制业务任务（官方 CtrBoard-H7_FDCAN 例程移植）。
//
// control task 是正常电机命令的唯一发送者：ROS 回调只覆盖静态 mailbox，
// 本任务按 10 ms 绝对周期消费命令、推进 motor_control 并下发 FDCAN action。
// 固件暂不维护完整拓扑，只保留当前四台默认测试电机。

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
// 每总线每周期最多处理 8 帧，总上限 24 帧；CAN1 积压留给后续周期，
// 保证 RX burst 不会无界占用 10 ms TX 周期。
static const size_t kCanRxDrainBudgetPerBus = 8U;

// 固定默认测试电机：FDCAN1 同时挂载 vaxor 25/30mm 与 CREB R58/R48。
// 拓扑固定为当前默认表，后续需扩展时应在更高层引入明确的配置入口，
// 不能把动态拓扑配置混进当前固件控制任务。
static const motor_control_motor_config_t kDefaultMotors[] = {
    {"left_wrist_yaw_joint", MOTOR_CONTROL_VAXOR, 1, 0,
     50.0, 0.65, 0.042, 0.008, 1.0},
    {"right_wrist_yaw_joint", MOTOR_CONTROL_VAXOR, 2, 0,
     50.0, 0.65, 0.042, 0.008, 1.0},
    {"left_shoulder_yaw_joint", MOTOR_CONTROL_CREB, 1, 0,
     101.0, 1.0, 0.086, 30.0, 1.0,
     MOTOR_PROTOCOL_PI, MOTOR_PROTOCOL_PI, 45.0},
    {"right_shoulder_yaw_joint", MOTOR_CONTROL_CREB, 2, 0,
     101.0, 1.0, 0.077, 30.0, 1.0,
     MOTOR_PROTOCOL_PI, MOTOR_PROTOCOL_PI, 11.0},
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
// ROS/link 任务只覆盖请求；motor_control_context_t 始终由 control task 修改。
static bool g_mailbox_initialized = false;
static bool g_task_started = false;
static volatile bool g_fdcan_ready = false;

volatile uint32_t g_control_motor_cmd_count = 0;
volatile uint32_t g_control_motor_found_count = 0;
volatile uint32_t g_control_can_rx_backlog_count[CAN_RX_BUS_COUNT] = {0};
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

bool control_task_submit_mode(const motor_control_mode_request_t *request) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  bool accepted = control_mailbox_submit_mode(&g_control_mailbox, request);
  taskEXIT_CRITICAL();
  return accepted;
}

bool control_task_submit_gains(const motor_control_gains_request_t *request) {
  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  bool accepted = control_mailbox_submit_gains(&g_control_mailbox, request);
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
  motor_control_joint_state_t state = {};
  bool valid = motor_control_get_joint_state(&g_motor_control, index, &state);
  if (valid) {
    memcpy(out->joint_name, state.joint_name, sizeof(out->joint_name));
    out->position = state.position;
    out->velocity = state.velocity;
    out->effort = state.effort;
  }
  taskEXIT_CRITICAL();
  return valid;
}

bool control_task_get_motor_status(size_t index, control_motor_status_t *out) {
  if (index >= kDefaultMotorCount || out == nullptr) {
    return false;
  }
  taskENTER_CRITICAL();
  motor_control_motor_status_t status = {};
  bool valid = motor_control_get_motor_status(&g_motor_control, index, &status);
  if (valid) {
    memcpy(out->joint_name, status.joint_name, sizeof(out->joint_name));
    out->sequence = status.sequence;
    out->voltage = static_cast<float>(status.voltage_v);
    out->fault = status.fault;
    out->motor_state = status.motor_state;
    out->calibration_state = status.calibration_state;
    out->mode = status.mode;
    out->motor_type = status.motor_type;
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

static void drain_motor_mailboxes() {
  control_mailbox_command_slot_t commands[kDefaultMotorCount] = {};
  control_mailbox_mode_slot_t modes[kDefaultMotorCount] = {};
  control_mailbox_gains_slot_t gains[kDefaultMotorCount] = {};

  taskENTER_CRITICAL();
  ensure_mailbox_initialized_locked();
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    bool configuration_pending =
        control_mailbox_has_configuration(&g_control_mailbox, index);
    modes[index].pending = control_mailbox_take_mode(
        &g_control_mailbox, index, &modes[index].request);
    gains[index].pending = control_mailbox_take_gains(
        &g_control_mailbox, index, &gains[index].request);
    if (!configuration_pending) {
      commands[index].pending = control_mailbox_take_command(
          &g_control_mailbox, index, &commands[index].command,
          &commands[index].received_ms, &commands[index].sequence);
    }
  }
  taskEXIT_CRITICAL();

  // 配置建立一个完整控制周期的屏障：先让 motor_control 发送停机/模式帧，
  // 配置之后到达的新命令留到下一周期，不能在同一 step 内直接 run。
  for (size_t index = 0; index < kDefaultMotorCount; ++index) {
    if (modes[index].pending) {
      (void)motor_control_set_mode(&g_motor_control,
                                   modes[index].request.joint_name,
                                   modes[index].request.mode);
    }
    if (gains[index].pending) {
      (void)motor_control_set_gains(&g_motor_control,
                                    gains[index].request.joint_name,
                                    gains[index].request.kp,
                                    gains[index].request.kd);
    }
    if (commands[index].pending) {
      (void)motor_control_accept_command(&g_motor_control,
                                         &commands[index].command,
                                         commands[index].received_ms);
    }
  }
}

static void accept_can_feedback(const can_rx_frame_t *frame) {
  motor_control_rx_frame_t feedback = {};
  feedback.bus_index = frame->bus_index;
  feedback.can_id = frame->id;
  feedback.size = frame->dlc;
  feedback.is_fd = frame->is_fd;
  feedback.is_extended = frame->is_extended;
  memcpy(feedback.data, frame->data, frame->dlc);
  (void)motor_control_accept_feedback(&g_motor_control, &feedback);
}

static void drain_can_rx_queues() {
  for (uint8_t bus_index = 0U; bus_index < CAN_RX_BUS_COUNT; ++bus_index) {
    size_t processed = 0U;
    can_rx_frame_t frame = {};
    while (processed < kCanRxDrainBudgetPerBus &&
           can_rx_queue_pop(bus_index, &frame)) {
      accept_can_feedback(&frame);
      ++processed;
    }
    if (processed == kCanRxDrainBudgetPerBus &&
        can_rx_queue_pending(bus_index) > 0U) {
      ++g_control_can_rx_backlog_count[bus_index];
    }
  }
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

static void control_task_fn(void *arg) {
  (void)arg;

  // 与官方任务编排一致：调度稳定后再初始化外设。
  vTaskDelay(pdMS_TO_TICKS(kControlStartDelayMs));
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  bsp_can_init();
  taskENTER_CRITICAL();
  g_fdcan_ready = true;
  taskEXIT_CRITICAL();

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    drain_control_requests();
    drain_motor_mailboxes();
    drain_can_rx_queues();
    run_motor_control_cycle(millis());
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
  taskENTER_CRITICAL();
  g_fdcan_ready = false;
  taskEXIT_CRITICAL();
  if (xTaskCreate(control_task_fn, "control", kControlTaskStack, nullptr,
                  kControlTaskPrio, nullptr) == pdPASS) {
    return true;
  }

  taskENTER_CRITICAL();
  g_task_started = false;
  taskEXIT_CRITICAL();
  return false;
}

bool control_task_fdcan_ready(void) {
  bool ready = false;
  taskENTER_CRITICAL();
  ready = g_fdcan_ready;
  taskEXIT_CRITICAL();
  return ready;
}
