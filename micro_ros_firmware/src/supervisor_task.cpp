#include "supervisor_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <string.h>

#include "control_task.h"
#include "sensor_task.h"
#include "supervisor_heartbeat.h"

static const uint32_t kSupervisorTaskStack = 512U;
static const UBaseType_t kSupervisorTaskPrio = 1U;
static const uint32_t kSupervisorPeriodMs = 100U;

// control 正常周期 10 ms；750 ms 仍能容忍启动延时和开发态的短暂调度抖动。
static const uint32_t kControlHeartbeatTimeoutMs = 750U;
// BMI088 正常 DRDY 为约 200 Hz；降级初始化每秒只会完成一次重试。
static const uint32_t kSensorHealthyHeartbeatTimeoutMs = 300U;
static const uint32_t kSensorDegradedHeartbeatTimeoutMs = 2500U;
// rmw_uros_ping_agent(200, 3) 合法阻塞约 600 ms，阈值留出调度余量。
static const uint32_t kMicroRosHeartbeatTimeoutMs = 1500U;

static volatile uint32_t g_heartbeats[SUPERVISOR_TASK_COUNT] = {};
static supervisor_diagnostics_t g_diagnostics = {};
static supervisor_heartbeat_state_t
    g_heartbeat_states[SUPERVISOR_TASK_COUNT] = {};
static bool g_task_started = false;

static void copy_heartbeats(uint32_t *out) {
  taskENTER_CRITICAL();
  for (size_t index = 0U; index < SUPERVISOR_TASK_COUNT; ++index) {
    out[index] = g_heartbeats[index];
  }
  taskEXIT_CRITICAL();
}

static void supervisor_task_fn(void *arg) {
  (void)arg;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    const uint32_t now_ms = millis();
    uint32_t heartbeats[SUPERVISOR_TASK_COUNT] = {};
    copy_heartbeats(heartbeats);

    const bool imu_healthy = sensor_task_imu_healthy();
    const uint32_t sensor_timeout_ms =
        imu_healthy ? kSensorHealthyHeartbeatTimeoutMs
                    : kSensorDegradedHeartbeatTimeoutMs;
    const bool control_stalled = supervisor_heartbeat_evaluate(
        &g_heartbeat_states[SUPERVISOR_TASK_CONTROL],
        heartbeats[SUPERVISOR_TASK_CONTROL], now_ms,
        kControlHeartbeatTimeoutMs);
    const bool sensor_stalled = supervisor_heartbeat_evaluate(
        &g_heartbeat_states[SUPERVISOR_TASK_SENSOR],
        heartbeats[SUPERVISOR_TASK_SENSOR], now_ms, sensor_timeout_ms);
    const bool micro_ros_stalled = supervisor_heartbeat_evaluate(
        &g_heartbeat_states[SUPERVISOR_TASK_MICRO_ROS],
        heartbeats[SUPERVISOR_TASK_MICRO_ROS], now_ms,
        kMicroRosHeartbeatTimeoutMs);

    uint32_t fault_bitmask = 0U;
    if (control_stalled) {
      fault_bitmask |= SUPERVISOR_FAULT_CONTROL_MISSED;
    }
    if (sensor_stalled) {
      fault_bitmask |= SUPERVISOR_FAULT_SENSOR_MISSED;
    }
    if (micro_ros_stalled) {
      fault_bitmask |= SUPERVISOR_FAULT_MICRO_ROS_STALLED;
    }
    if (!imu_healthy) {
      fault_bitmask |= SUPERVISOR_FAULT_IMU_DEGRADED;
    }
    if (!control_task_fdcan_ready()) {
      // 启动延时内也可见，但仅作诊断位；不改变电机状态也不触发复位。
      fault_bitmask |= SUPERVISOR_FAULT_FDCAN_NOT_READY;
    }

    taskENTER_CRITICAL();
    memcpy(g_diagnostics.heartbeat, heartbeats, sizeof(heartbeats));
    if (control_stalled) {
      ++g_diagnostics.miss_count[SUPERVISOR_TASK_CONTROL];
    }
    if (sensor_stalled) {
      ++g_diagnostics.miss_count[SUPERVISOR_TASK_SENSOR];
    }
    if (micro_ros_stalled) {
      ++g_diagnostics.miss_count[SUPERVISOR_TASK_MICRO_ROS];
    }
    g_diagnostics.fault_bitmask = fault_bitmask;
    g_diagnostics.last_check_ms = now_ms;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kSupervisorPeriodMs));
  }
}

bool supervisor_task_start(void) {
  taskENTER_CRITICAL();
  if (g_task_started) {
    taskEXIT_CRITICAL();
    return true;
  }
  g_task_started = true;
  taskEXIT_CRITICAL();

  if (xTaskCreate(supervisor_task_fn, "supervisor", kSupervisorTaskStack,
                  nullptr, kSupervisorTaskPrio, nullptr) == pdPASS) {
    return true;
  }

  taskENTER_CRITICAL();
  g_task_started = false;
  taskEXIT_CRITICAL();
  return false;
}

void supervisor_heartbeat(supervisor_task_id_t task) {
  if (task >= SUPERVISOR_TASK_COUNT) {
    return;
  }
  taskENTER_CRITICAL();
  ++g_heartbeats[task];
  taskEXIT_CRITICAL();
}

void supervisor_get_diagnostics(supervisor_diagnostics_t *out) {
  if (out == nullptr) {
    return;
  }
  taskENTER_CRITICAL();
  *out = g_diagnostics;
  taskEXIT_CRITICAL();
}
