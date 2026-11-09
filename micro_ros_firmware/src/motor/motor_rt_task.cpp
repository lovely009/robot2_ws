// 电机实时任务：1ms 服务、配置周期的控制边界。运行期不动态分配，
// 不等待 ROS/Agent/设备响应/W25Q64；RX/TX/配置均有每周期配额。

#include "motor_rt_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include <cstring>

#include "agent_control_gate.h"
#include "trajectory_transaction.hpp"
#include "time_utils.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "motor_config_contract.hpp"
#include "motor_protocol_adapter.h"
#include "motor_tx_router.h"
#include "session_sync.h"
#include "firmware_config.h"
#include "storage_boot.h"

// 各库电机容量须一致；RX ring 深度不是电机容量，不纳入此约束。
static_assert(MOTOR_CONTROL_MAX_MOTORS == CONTROL_COMMAND_MAILBOX_MAX_TARGETS,
    "命令快照容量必须与注册表容量一致");
static_assert(MOTOR_CONTROL_MAX_MOTORS == CAN_TX_P1_SLOTS,
    "P1 调度槽数必须与注册表容量一致");
static_assert(MOTOR_CONTROL_MAX_MOTORS == MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS,
    "内核快照目标容量必须与注册表容量一致");
static_assert(MOTOR_CONTROL_MAX_TRAJECTORY_POINTS ==
    CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS,
    "轨迹点数容量必须与 mailbox 容量一致");
static_assert(MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS ==
    CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS,
    "轨迹电机容量必须与 mailbox 容量一致");
static_assert(MOTOR_CONTROL_MAX_MOTORS == CONTROL_POLICY_MAILBOX_MAX_TARGETS,
    "策略容量必须与注册表容量一致");

// 栈 1024 字（4KB）；1ms 节拍的栈用量与 WCET 需台架验证。
static const uint32_t kMotorRtTaskStack = 1024U;
static const UBaseType_t kMotorRtTaskPrio = 3U;
static const TickType_t kServicePeriodTicks = pdMS_TO_TICKS(1U);
static_assert(pdMS_TO_TICKS(1U) > 0U, "1kHz 控制需要至少 1kHz 的 RTOS tick");
// 默认 5ms/200Hz、可配至 1ms/1kHz：轨迹插值与快照保持每台每边界一条命令。
// P1 槽 latest-wins；CERB 出队后聚合为每帧最多 6 轴的 0x210。
// 20ms/50Hz SYNC 与上行状态独立；总线负载和 WCET/jitter 需无负载台架验证。

// 每周期配额，需按台架数据验证。
static const uint32_t kRxDrainBudgetPerBus = 16U;   // 每 1ms 每总线最多解码帧数
static const uint32_t kRxPopBatch = 8U;             // 单次批量弹出（栈上界）
// 两次服务拍至少搬完注册容量内的控制帧，并给 P0/P2/SYNC 留余量。
static const uint32_t kTxCollectBudget =
    MOTOR_TX_ROUTER_LOGICAL_CAPACITY; // 每 1ms 最多取走内核待发帧数
static const uint32_t kConfigDrainPerBoundary = 4U; // 每个控制边界最多转发配置数
static const motor_control_stored_params_t *g_stored_params;
static size_t g_stored_count;

static const uint32_t kDiscoverySyncPeriodMs = FIRMWARE_MOTOR_REPORT_PERIOD_MS;
// SYNC 心跳维持 CERB 状态上报；VAXOR 自推周期由设备配置决定。

// 不透明内核单例（布局在 motor_control 库内私有；本任务只持指针）。
static motor_control_context_t *const g_motor_control =
    motor_control_instance();
static control_mailbox_t g_control_mailbox;
static agent_control_gate_t g_agent_gate;
static trajectory_transaction::Coordinator g_trajectory;
static bool g_clock_requested = false;

// boot 存储参数（start 时从 storage 任务快照；控制边界套用到内核）。
static TaskHandle_t g_task_handle = nullptr;
static bool g_task_started = false;

static motor_tx_router_t g_tx_router;

// ---------------------------------------------------------------------------
// mailbox 写入口（外部任务上下文；短临界区保护 mailbox）
// ---------------------------------------------------------------------------

bool motor_rt_submit_control_lease(uint64_t session_id, uint64_t lease_id,
    uint32_t sequence, uint32_t deadline, bool active) {
  taskENTER_CRITICAL();
  const bool ok = control_lease_submit(&g_agent_gate.authority, session_id,
      lease_id, sequence, deadline, active, millis());
  taskEXIT_CRITICAL();
  return ok;
}
void motor_rt_copy_control_lease(control_lease_state_t *out) {
  if (!out) return;
  taskENTER_CRITICAL();
  *out = g_agent_gate.authority;
  out->active = control_lease_allows(out, out->lease_id, millis());
  taskEXIT_CRITICAL();
}

bool motor_rt_submit_command(const control_command_snapshot_t *snapshot) {
  taskENTER_CRITICAL();
  const bool ok = agent_gate_submit_command(&g_agent_gate, &g_control_mailbox,
      snapshot, millis());
  taskEXIT_CRITICAL();
  return ok;
}

// 调用方已持短临界区；租约不能只看上一服务拍缓存的 bool。
static bool trajectory_gate_allowed(uint32_t now_ms) {
  const agent_link_t &link = g_agent_gate.link;
  return control_lease_allows(&g_agent_gate.authority, g_agent_gate.authority.lease_id, now_ms) &&
      link.connected && link.clock_valid && g_clock_requested &&
      !link.stop_pending && link.lease_valid && link.authorized &&
      g_agent_gate.kernel_authorized && link.have_command &&
      now_ms - link.command_tick_ms <= 30U;
}

bool motor_rt_submit_agent_trajectory(const agent_trajectory_request_t *request) {
  if (request == nullptr) return false;
  const uint32_t now_ms = millis();
  taskENTER_CRITICAL();
  bool ok = false;
  if (request->operation == AGENT_TRAJECTORY_ACK) {
    // ACK 不占请求号，也不刷新授权；互斥载荷仍须为空。
    if (request->request_id == 0U && request->trajectory_id == 0U &&
        request->segment_index == 0U && request->segment_count == 0U &&
        request->target_count == 0U && request->point_count == 0U) {
      ok = g_trajectory.ack(request->session_id, request->ack_request_id,
          request->ack_terminal_trajectory_id);
    }
  } else {
    using namespace trajectory_transaction;
    Receipt receipt{};
    const bool motion = request->operation == AGENT_TRAJECTORY_LOAD ||
        request->operation == AGENT_TRAJECTORY_COMMIT;
    // 已受理请求必须先走幂等路径：过期重发只读旧收据，不重新执行。
    const bool fresh = request->request_id > g_trajectory.status().request_high_water;
    if (motion && fresh && (!trajectory_gate_allowed(now_ms) ||
        !control_lease_allows(&g_agent_gate.authority, request->lease_id, now_ms))) {
      receipt = g_trajectory.reject(*request, REASON_GATE);
    } else if (motion && fresh && request->operation == AGENT_TRAJECTORY_LOAD &&
        request->target_count <= 48U &&
        !motor_control_trajectory_targets_healthy(g_motor_control,
            request->target_ids, request->target_count, now_ms)) {
      receipt = g_trajectory.reject(*request, REASON_MEASUREMENT);
    } else if (request->operation == AGENT_TRAJECTORY_LOAD) {
      receipt = g_trajectory.load(*request, now_ms);
    } else if (request->operation == AGENT_TRAJECTORY_COMMIT) {
      receipt = g_trajectory.commit(*request, now_ms);
    } else if (request->operation == AGENT_TRAJECTORY_CANCEL) {
      receipt = g_trajectory.cancel(*request);
    } else {
      receipt = g_trajectory.reject(*request, REASON_INVALID);
    }
    ok = receipt.state == AGENT_TRAJECTORY_PENDING ||
        receipt.state == AGENT_TRAJECTORY_ACCEPTED;
  }
  if (!ok) ++g_control_mailbox.trajectory_rejected_count;
  taskEXIT_CRITICAL();
  return ok;
}

void motor_rt_copy_agent_trajectory(agent_trajectory_status_t *out) {
  if (out == nullptr) return;
  taskENTER_CRITICAL();
  *out = g_trajectory.status();
  taskEXIT_CRITICAL();
}

bool motor_rt_agent_trajectory_owns(uint16_t motor_id) {
  taskENTER_CRITICAL();
  const bool owned = g_trajectory.owns(motor_id);
  taskEXIT_CRITICAL();
  return owned;
}

bool motor_rt_submit_policy(const control_policy_snapshot_t *snapshot) {
  taskENTER_CRITICAL();
  const bool ok = agent_gate_submit_policy(&g_agent_gate, &g_control_mailbox,
      snapshot, millis());
  taskEXIT_CRITICAL();
  return ok;
}

// mailbox 无内部锁，配置读写须共用临界区；校验与门禁由 mailbox/内核负责。
bool motor_rt_submit_config(
    const control_mailbox_config_request_t *request) {
  if (!request || (request->kind != CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS &&
      request->kind != CONTROL_MAILBOX_CONFIG_SET_ZERO &&
      request->kind != CONTROL_MAILBOX_CONFIG_CLEAR_FAULT)) return false;
  taskENTER_CRITICAL();
  const bool ok = control_mailbox_submit_config(&g_control_mailbox, request);
  taskEXIT_CRITICAL();
  return ok;
}

void motor_rt_set_agent_connected(bool connected) {
  taskENTER_CRITICAL();
  control_mailbox_request_agent_connected(&g_control_mailbox, connected);
  taskEXIT_CRITICAL();
}

void motor_rt_begin_agent_session(uint64_t session_id) {
  taskENTER_CRITICAL();
  agent_gate_begin_session(&g_agent_gate, &g_control_mailbox, session_id);
  g_trajectory.begin_session(session_id);
  taskEXIT_CRITICAL();
}

void motor_rt_request_disable(uint64_t motor_mask) {
  taskENTER_CRITICAL();
  agent_gate_request_disable(&g_agent_gate, motor_mask);
  taskEXIT_CRITICAL();
}

void motor_rt_request_stop_all(void) {
  taskENTER_CRITICAL();
  agent_gate_request_stop(&g_agent_gate);
  taskEXIT_CRITICAL();
}

void motor_rt_set_clock_valid(bool valid) {
  taskENTER_CRITICAL();
  if (g_clock_requested && !valid) agent_gate_request_stop(&g_agent_gate);
  g_clock_requested = valid;
  taskEXIT_CRITICAL();
}

void motor_rt_copy_link_status(motor_rt_link_status_t *out) {
  if (out == nullptr) return;
  taskENTER_CRITICAL();
  const agent_link_t *link = &g_agent_gate.link;
  out->command_sequence = link->command_sequence;
  out->stop_generation = link->stop_generation;
  out->required_policy_generation = link->barrier_generation;
  out->lease_valid = link->lease_valid;
  out->authorized = link->authorized && g_agent_gate.kernel_authorized;
  taskEXIT_CRITICAL();
}

// ---------------------------------------------------------------------------
// 只读状态副本（内核唯一写者是本任务；读侧临界区在单核上即可保证一致）
// ---------------------------------------------------------------------------

size_t motor_rt_copy_motion_states(motor_control_motion_state_t *out,
    size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  const uint32_t now_ms = millis();
  taskENTER_CRITICAL();
  const size_t count =
      motor_control_copy_motion_states(g_motor_control, out, capacity, now_ms);
  taskEXIT_CRITICAL();
  return count;
}

size_t motor_rt_copy_statuses(motor_control_status_t *out, size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  const uint32_t now_ms = millis();
  taskENTER_CRITICAL();
  const size_t count =
      motor_control_copy_statuses(g_motor_control, out, capacity, now_ms);
  taskEXIT_CRITICAL();
  return count;
}

size_t motor_rt_copy_param_views(motor_control_param_view_t *out,
    size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  taskENTER_CRITICAL();
  const size_t count =
      motor_control_copy_param_views(g_motor_control, out, capacity);
  taskEXIT_CRITICAL();
  return count;
}

// ---------------------------------------------------------------------------
// 内核事件环与拓扑副本
// ---------------------------------------------------------------------------
//
// 内核事件环由本任务生产、ROS 桥消费，两侧共用临界区；
// 满时丢弃最旧事件，计入 ctx.event_dropped_count。

size_t motor_rt_drain_events(motor_control_event_t *out, size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  taskENTER_CRITICAL();
  const size_t count =
      motor_control_drain_events(g_motor_control, out, capacity);
  taskEXIT_CRITICAL();
  return count;
}

void motor_rt_copy_trajectory_state(motor_control_trajectory_state_t *out) {
  if (out == nullptr) {
    return;
  }
  const uint32_t now_ms = millis();
  taskENTER_CRITICAL();
  motor_control_copy_trajectory_state(g_motor_control, out, now_ms);
  taskEXIT_CRITICAL();
}

void motor_rt_copy_diagnostics(motor_rt_diagnostics_t *out) {
  if (out == nullptr) {
    return;
  }
  taskENTER_CRITICAL();
  motor_control_copy_diagnostics(g_motor_control, &out->kernel);
  taskEXIT_CRITICAL();
  // ring 溢出在 lib/board 侧，堆余量只有本任务能问（内核不感知堆）。
  static_assert(MOTOR_CONTROL_MAX_BUSES == CAN_RX_BUS_COUNT,
      "总线数契约：内核与 fdcan 必须一致");
  for (uint8_t bus = 0U; bus < MOTOR_CONTROL_MAX_BUSES; ++bus) {
    can_rx_ring_t *ring = can_bsp_rx_ring(bus);
    out->ring_drop[bus] = ring != nullptr ? ring->overflow_count : 0U;
  }
  // 拒绝计数包含外层授权/会话拒绝与内层非法载荷，两者均未进入内核。
  out->mailbox_rejected_command = g_control_mailbox.command_rejected_count +
      g_control_mailbox.command.invalid_snapshot_count;
  out->mailbox_rejected_trajectory =
      g_control_mailbox.trajectory_rejected_count +
      g_control_mailbox.trajectory.invalid_trajectory_count;
  out->mailbox_rejected_policy = g_control_mailbox.policy_rejected_count +
      g_control_mailbox.policy.invalid_policy_count;
  for (uint8_t bus = 0U; bus < MOTOR_CONTROL_MAX_BUSES; ++bus) {
    out->rx_hw_lost[bus] = can_bsp_rx_hw_lost(bus);
    out->tx_frame_count[bus] = can_bsp_tx_frame_count(bus);
  }
  out->heap_free = (uint32_t)xPortGetFreeHeapSize();
}

size_t motor_rt_copy_topology(motor_rt_topology_entry_t *out, size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  taskENTER_CRITICAL();
  // 内核按 motor_id 升序产出拓扑，这里只做协议编号映射与字符串落地。
  motor_control_topology_entry_t topo_in[MOTOR_CONTROL_MAX_MOTORS];
  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  const size_t kernel_count =
      motor_control_copy_topology(g_motor_control, topo_in, capacity);
  for (size_t i = 0; i < kernel_count; ++i) {
    const motor_control_topology_entry_t *in = &topo_in[i];
    motor_rt_topology_entry_t *topo = &out[i];
    topo->motor_id = in->motor_id;
    topo->bus_index = in->bus_index;
    topo->protocol = 0xFFU; // 未登记协议（防御；当前两协议均可命中）
    for (size_t k = 0U; k < adapter_count; ++k) {
      if (adapters[k] == in->protocol) {
        topo->protocol = (uint8_t)k; // 注册顺序 = MotorTopologyEntry 协议枚举
        break;
      }
    }
    topo->model_name[0] = '\0';
    if (in->model_name != nullptr) {
      strncpy(topo->model_name, in->model_name,
          sizeof(topo->model_name) - 1U);
      topo->model_name[sizeof(topo->model_name) - 1U] = '\0';
    }
    topo->capability_flags = in->capability_flags;
    topo->online = in->online;
    topo->conflict = in->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
    topo->lifecycle = (uint8_t)in->lifecycle;
  }
  taskEXIT_CRITICAL();
  return kernel_count;
}

// ---------------------------------------------------------------------------
// 任务主体
// ---------------------------------------------------------------------------

// 1ms 服务之一：三路 RX ring 有界弹出 → 内核原始帧输入。
// can_rx_frame_t 与 motor_protocol_raw_frame_t 字段一一对应（fdcan 层与
// 协议层各自独立定义，任务层显式拷贝，不假设布局一致）。
static void drain_rx_rings(void) {
  static can_rx_frame_t batch[kRxPopBatch]; // 静态而非栈上：固定容量、可审计
  for (uint8_t bus = 0U; bus < CAN_RX_BUS_COUNT; ++bus) {
    can_rx_ring_t *ring = can_bsp_rx_ring(bus);
    if (ring == nullptr) {
      continue;
    }
    uint32_t drained = 0U;
    while (drained < kRxDrainBudgetPerBus) {
      const uint32_t popped =
          can_rx_ring_pop(ring, batch, kRxPopBatch);
      if (popped == 0U) {
        break;
      }
      for (uint32_t i = 0; i < popped; ++i) {
        motor_protocol_raw_frame_t frame = {0};
        frame.bus_index = batch[i].bus_index;
        frame.can_id = batch[i].can_id;
        frame.is_extended = batch[i].is_extended;
        frame.is_fd = batch[i].is_fd;
        frame.is_brs = batch[i].is_brs;
        frame.dlc = batch[i].dlc;
        frame.rx_tick_ms = batch[i].rx_tick_ms;
        memcpy(frame.data, batch[i].data, MOTOR_PROTOCOL_MAX_FRAME_BYTES);
        motor_control_on_raw_frame(g_motor_control, &frame);
      }
      drained += popped;
    }
  }
}

// CERB 总线每 20ms 发 0x080 SYNC，维持状态上报及发现/热插拔心跳；
// VAXOR 无 SYNC。此心跳不得用于触发控制。
static void sustain_discovery_sync(uint32_t now_ms) {
  static uint32_t last_sync_ms = 0U;
  if ((uint32_t)(now_ms - last_sync_ms) < kDiscoverySyncPeriodMs) {
    return;
  }
  last_sync_ms = now_ms;
  for (uint8_t bus = 0U; bus < CAN_TX_BUS_COUNT; ++bus) {
    (void)motor_control_probe_discovery(g_motor_control, bus);
  }
}

// 1ms 服务之二：内核待发队列 → 每总线调度器（P0 FIFO / P1 latest-wins 槽 /
// P2 FIFO）。tx_slot = 内核表项索引（表项只增不减故稳定，同总线电机槽互异）；
// motor_protocol_tx_frame_t 与 can_tx_frame_t 显式逐字段拷贝（不假设布局一致）。
static void route_kernel_tx(void) {
  // Apply the kernel's stop barrier before moving fresh frames. This removes
  // unsent enables and motion at both queue stages, including P0 enables.
  const uint64_t cancelled = motor_control_take_tx_cancellations(g_motor_control);
  const uint64_t zero_cancelled = motor_control_take_zero_tx_cancellations(g_motor_control);
  for (uint8_t bus = 0U; bus < CAN_TX_BUS_COUNT; ++bus) {
    can_tx_scheduler_cancel_slots(can_bsp_tx_scheduler(bus), cancelled);
    can_tx_scheduler_cancel_timed_p2(can_bsp_tx_scheduler(bus), zero_cancelled);
  }
  static motor_protocol_tx_frame_t batch[kTxCollectBudget]; // 静态：固定容量
  const size_t count =
      motor_control_collect_tx(g_motor_control, batch, kTxCollectBudget);
  for (size_t i = 0; i < count; ++i) {
    can_tx_scheduler_t *sched = can_bsp_tx_scheduler(batch[i].bus_index);
    if (sched == nullptr) {
      continue;
    }
    const size_t bytes = batch[i].size <= CAN_TX_FRAME_BYTES
                             ? batch[i].size
                             : CAN_TX_FRAME_BYTES; // 拷贝上界防御
    can_tx_frame_t frame = {};
    frame.can_id = batch[i].can_id;
    frame.tx_slot = batch[i].tx_slot;
    frame.is_extended = batch[i].is_extended;
    frame.is_fd = batch[i].is_fd;
    frame.is_brs = batch[i].is_brs;
    frame.size = (uint8_t)bytes;
    frame.node = batch[i].node;
    frame.aggregate_only = batch[i].aggregate_only;
    frame.valid_until_tick_ms = batch[i].valid_until_tick_ms;
    memcpy(frame.data, batch[i].data, bytes);
    switch (batch[i].tx_class) {
    case MOTOR_PROTOCOL_TX_CLASS_P0:
      // 调度器 FIFO 满时回告内核重入队，由 p0_pending 逐拍补发；
      // 安全帧不允许在搬运链路中静默丢弃。
      if (!can_tx_scheduler_submit_p0(sched, &frame)) {
        (void)motor_control_requeue_p0(g_motor_control, &batch[i]);
      }
      break;
    case MOTOR_PROTOCOL_TX_CLASS_P1:
      (void)can_tx_scheduler_submit_p1(sched, batch[i].tx_slot, &frame);
      break;
    case MOTOR_PROTOCOL_TX_CLASS_P2:
    default:
      (void)can_tx_scheduler_submit_p2(sched, &frame);
      break;
    }
  }
}

static bool submit_routed_tx(void *context, uint8_t bus_index,
    const can_tx_frame_t *frame) {
  (void)context;
  return can_bsp_submit_tx(bus_index, frame);
}

// 1ms 服务之三：每总线只调用一次路由器；HAL 提交仍由本任务独占。
static void pump_tx_schedulers(void) {
  for (uint8_t bus = 0U; bus < CAN_TX_BUS_COUNT; ++bus) {
    can_tx_scheduler_t *scheduler = can_bsp_tx_scheduler(bus);
    if (scheduler == nullptr) {
      continue;
    }
    // P0 积压可能令 P1 跨过原始截止；物理出队前取消失效槽，不能停机后补发旧目标。
    const uint32_t now_ms = millis();
    can_tx_scheduler_expire_p2(scheduler, now_ms);
    for (uint16_t slot = 0; slot < CAN_TX_P1_SLOTS; ++slot) {
      if (scheduler->p1.dirty[slot] &&
          !motor_control_target_valid(g_motor_control, slot, now_ms)) {
        scheduler->p1.dirty[slot] = false;
      }
    }
    (void)motor_tx_router_pump(&g_tx_router, scheduler, bus,
        submit_routed_tx, nullptr);
  }
}

// 控制边界：会话/授权边沿 → 内核；完整快照 → apply；配置 FIFO 有界转发。
//
// mailbox 类型与内核类型刻意互不依赖（同形镜像）；以下转换是任务层的
// 缝合职责，逐字段拷贝，不假设两侧布局一致。

// 两侧枚举类型独立；motor_config_contract.hpp 锁住完整数值映射。
static void convert_config(const control_mailbox_config_request_t *in,
    motor_protocol_config_request_t *out) {
  out->request_id = in->request_id;
  out->motor_id = in->motor_id;
  // kind 数值契约一致（1..6、8..14；7 是内核专用 READ_IDENTITY 不经
  // mailbox，上方 static_assert 编译期锁），两侧枚举类型独立故显式转换。
  out->kind = (motor_protocol_config_kind_t)in->kind;

}

static void control_boundary(uint32_t now_ms) {
  const bool clock_valid = time_clock_valid();
  taskENTER_CRITICAL();
  agent_gate_step(&g_agent_gate, &g_control_mailbox, g_motor_control, now_ms,
      clock_valid && g_clock_requested);
  // 只在控制边界消费 COMMIT/CANCEL；组装区直接借给内核，无全量转换副本。
  g_trajectory.boundary(g_motor_control, now_ms,
      clock_valid && trajectory_gate_allowed(now_ms));
  taskEXIT_CRITICAL();
  // 快照通道的边界保持重发：与 trajectory_step 同拍、覆盖互斥的电机集合，
  // 两者合计使每个边界每台恰好产生一帧 P1（见 motor_control.h 的说明）。
  motor_control_hold_step(g_motor_control, now_ms);

  // 配置 FIFO 有界转发（每边界至多 kConfigDrainPerBoundary 条，不阻塞节拍）。
  for (uint32_t i = 0; i < kConfigDrainPerBoundary; ++i) {
    control_mailbox_config_request_t request_in = {0};
    bool got_config = false;
    taskENTER_CRITICAL();
    got_config = control_mailbox_take_config(&g_control_mailbox, &request_in);
    taskEXIT_CRITICAL();
    if (!got_config) {
      break;
    }
    const uint32_t remaining = request_in.valid_until_tick_ms - now_ms;
    if (remaining == 0U || remaining > 1000U) {
      motor_control_report_config_result(g_motor_control, request_in.request_id,
          request_in.motor_id, MOTOR_PROTOCOL_CONFIG_INVALID, now_ms);
      continue;
    }
    motor_protocol_config_request_t request = {0};
    convert_config(&request_in, &request);
    (void)motor_control_submit_config(g_motor_control, &request, now_ms);
  }

  // boot 存储参数：Profile 未绑定的电机保留待套用位，后续边界自动重试。
  // 无记录（count=0）时也必须调用：内核据此把"记录已就位（哪怕是没有记录）"
  // 告知每个表项，上电设备参数回读才会发起（见 motor_control_config_step 0b）。
  motor_control_apply_stored_params(g_motor_control, g_stored_params, g_stored_count, now_ms);

  // 写事务只更新内存记录与脏位；COMMAND_SAVE 显式触发设备 Flash 和 W25 保存。
}

// IWDG 仅由 motor_rt 任务喂：安全任务停摆会超时复位至 WAIT_AGENT 和授权
// 清零的初始态；micro_ros 任务停摆时，motor_rt 仍能执行快照超时失能。
// 调试器断点期间冻结计数，避免误复位。
#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32h7xx_hal.h"

static IWDG_HandleTypeDef g_iwdg;

static void watchdog_start(void) {
  __HAL_RCC_LSI_ENABLE();
  while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) {
  }
  g_iwdg.Instance = IWDG1; // H7 实例名为 IWDG1
  g_iwdg.Init.Prescaler = IWDG_PRESCALER_32; // LSI 32kHz / 32 = 1kHz
  g_iwdg.Init.Reload = 500U;                 // ~500ms 超时
  g_iwdg.Init.Window = IWDG_WINDOW_DISABLE;
  (void)HAL_IWDG_Init(&g_iwdg);
#ifdef __HAL_DBGMCU_FREEZE_IWDG
  __HAL_DBGMCU_FREEZE_IWDG();
#endif
}

static void watchdog_feed(void) {
  (void)HAL_IWDG_Refresh(&g_iwdg);
}
#else
static void watchdog_start(void) {
}
static void watchdog_feed(void) {
}
#endif

static void motor_rt_task_fn(void *pv_parameters) {
  (void)pv_parameters;

  // FDCAN 三路硬件初始化归 lib/board 门卫（bsp_can_init 建 ring/调度器/
  // 过滤器并启动硬件）；任务层不直接触碰 HAL 句柄。
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  bsp_can_init();

  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_control_ms = millis();

  for (;;) {
    watchdog_feed(); // 活性监督：本循环停摆 ≥ 超时即复位
    vTaskDelayUntil(&last_wake, kServicePeriodTicks);
    const uint32_t now_ms = millis();

    // 1ms 服务：有界 RX → 内核运行时服务 → SYNC → 待发帧路由。
    drain_rx_rings();
    const bool clock_valid = time_clock_valid();
    taskENTER_CRITICAL();
    const uint8_t safety_action = agent_gate_service(&g_agent_gate,
        &g_control_mailbox, g_motor_control, now_ms,
        clock_valid && g_clock_requested);
    if (safety_action != 0U)
      g_trajectory.abort(trajectory_transaction::REASON_STOPPED);
    g_trajectory.poll(now_ms);
    agent_gate_admit_command(&g_agent_gate, &g_control_mailbox, g_motor_control, now_ms);
    taskEXIT_CRITICAL();
    // 配置周期的控制边界：按 millis 差触发，不补发过期边界。
    if ((uint32_t)(now_ms - last_control_ms) >=
        FIRMWARE_MOTOR_COMMAND_PERIOD_MS) {
      last_control_ms = now_ms;
      control_boundary(now_ms);
    }

    // Consume an already queued, fresh replacement at this same boundary
    // before judging the previous snapshot's absolute deadline. Ownership
    // revocation above still wins. No cached target or deadline is extended.
    motor_control_service(g_motor_control, now_ms);
    sustain_discovery_sync(now_ms);

    // Collect once after the boundary: a newly applied snapshot and its hold
    // output share the kernel's latest-wins slot before the bounded TX drain.
    route_kernel_tx();
    // 1ms 服务：每总线 P0/P1/P2 出队并提交硬件。
    pump_tx_schedulers();
  }
}

bool motor_rt_task_start(void) {
  if (g_task_started) {
    return true;
  }
  // mailbox 与内核先于任务初始化；内核 adapter 列表构造注入。
  motor_tx_router_init(&g_tx_router);
  control_mailbox_init(&g_control_mailbox);
  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  motor_control_init(g_motor_control, adapters, adapter_count);
  g_stored_params = storage_boot_params(&g_stored_count);
  // 型号绑定不需要注入：内核在发现建档/重连时向设备读自报身份并查型号
  // 目录（协议 adapter 的 READ_IDENTITY 事务），无身份读对象的协议回退到
  // "协议内唯一型号"。表外设备保持未绑定不可控。
  // IWDG 先于任务创建就绪：首拍即开始喂狗，任务创建后 500ms 内必有第一次
  // 喂狗。复位后果是回到 WAIT_AGENT + 授权清零的初始态（安全侧）。
  watchdog_start();

  if (xTaskCreate(motor_rt_task_fn, "motor_rt", kMotorRtTaskStack, nullptr,
      kMotorRtTaskPrio, &g_task_handle) != pdPASS) {
    // 任务创建失败是致命配置错误：停在这里，禁止系统带残缺实时层运行。
    Error_Handler();
    return false;
  }
  g_task_started = true;
  return true;
}
