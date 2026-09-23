// 电机实时任务实现（ARD-0007 §11 双节拍 / §51.2-51.3）。
//
// 有界性（ARD §11.2）：本任务不等待 ROS/Agent/设备响应/W25Q64，不动态分配；
// RX/TX/配置每周期配额为具名常量（TBD-CAN-01 占位值，台架数据后收敛）。

#include "motor_rt_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include <cstring>

#include "can_bsp.h"
#include "fdcan.h"
#include "motor_config_contract.hpp"
#include "motor_protocol_adapter.h"
#include "motor_tx_router.h"
#include "session_sync.h"
#include "storage_task.h"

// 跨库容量契约：64 这一物理容量在各库独立定义，单侧改小会静默截断，
// 编译期锁齐。RX ring 的 64 是环深语义（非设备数），刻意不纳入。
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

// 任务参数：栈 1024 字（4KB）。1ms 节拍任务的栈/WCET 需按台架数据复测
// （TBD-RT-01），不得沿用 10ms 旧任务的假设。
static const uint32_t kMotorRtTaskStack = 1024U;
static const UBaseType_t kMotorRtTaskPrio = 3U;
static const TickType_t kServicePeriodTicks = pdMS_TO_TICKS(1U);
// 控制边界 2ms/500Hz —— 它同时**决定 CAN 命令速率**：轨迹机组由
// trajectory_step 逐边界插值下发，快照机组由 hold_step 逐边界保持重发，两条通道
// 合计每台每个边界恰好一条命令，P1 调度槽 latest-wins 覆盖同一拍内的重投递；
// CERB 命令在出队后合成每帧最多 6 轴的 0x210。
// 目标命令率为 500Hz；20ms/50Hz SYNC 与上行状态节拍独立。总线负载、
// motor_rt WCET/jitter 需要在无负载台架复测，不能沿用 200Hz 时的结论。
static const uint32_t kControlPeriodMs = 2U;

// 每周期配额（TBD-CAN-01 占位；均为具名可调常量）。
static const uint32_t kRxDrainBudgetPerBus = 16U;   // 每 1ms 每总线最多解码帧数
static const uint32_t kRxPopBatch = 8U;             // 单次批量弹出（栈上界）
// 两次服务拍至少搬完 64 台的控制帧，并给 P0/P2/SYNC 留余量。
static const uint32_t kTxCollectBudget =
    MOTOR_TX_ROUTER_LOGICAL_CAPACITY; // 每 1ms 最多取走内核待发帧数
static const uint32_t kConfigDrainPerBoundary = 4U; // 每个控制边界最多转发配置数
static const uint32_t kDiscoverySyncPeriodMs = 20U; // SYNC 心跳周期（维持设备
                                                    // 20ms 状态上报，与 VAXOR
                                                    // 20ms 自推统一 50Hz）

// 不透明内核单例（布局在 motor_control 库内私有；本任务只持指针）。
static motor_control_context_t *const g_motor_control =
    motor_control_instance();
static control_mailbox_t g_control_mailbox;

// boot 存储参数（start 时从 storage 任务快照；控制边界套用到内核）。
static motor_control_stored_params_t
    g_stored_params[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_stored_count = 0U;
static TaskHandle_t g_task_handle = nullptr;
static bool g_task_started = false;

static motor_tx_router_t g_tx_router;

// ---------------------------------------------------------------------------
// mailbox 写入口（外部任务上下文；短临界区保护 mailbox）
// ---------------------------------------------------------------------------

bool motor_rt_submit_command(const control_command_snapshot_t *snapshot) {
  taskENTER_CRITICAL();
  const bool ok = control_mailbox_submit_command(&g_control_mailbox, snapshot);
  taskEXIT_CRITICAL();
  return ok;
}

bool motor_rt_submit_trajectory(const control_trajectory_t *trajectory) {
  taskENTER_CRITICAL();
  const bool ok =
      control_mailbox_submit_trajectory(&g_control_mailbox, trajectory);
  taskEXIT_CRITICAL();
  return ok;
}

bool motor_rt_submit_policy(const control_policy_snapshot_t *snapshot) {
  taskENTER_CRITICAL();
  const bool ok = control_mailbox_submit_policy(&g_control_mailbox, snapshot);
  taskEXIT_CRITICAL();
  return ok;
}

// 低频配置入口（当前无 ROS 生产者）。与命令同款短
// 临界区：mailbox 无内部锁，读写两侧必须共用同一互斥域。
// 校验与门禁全在 mailbox/内核：request_id==0、字段非法 → config_invalid_count；
// 断链期 → config_session_count；FIFO 满 → config_overflow_count。
bool motor_rt_submit_config(
    const control_mailbox_config_request_t *request) {
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
// 内核事件环（motor_control 内，深度 MOTOR_CONTROL_MAX_EVENTS）即为桥的
// 消费源：本任务生产、ROS 桥在轮询里取走。此处不再经中间环搬运——两侧
// 本就共用同一临界区，二次解耦没有带来并发隔离，反而多一层搬运与一份
// 需要单独维护的丢弃计数。丢弃计数保留在内核（ctx.event_dropped_count）。

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
  // ring 溢出在 lib/fdcan 侧，堆余量只有本任务能问（内核不感知堆）。
  static_assert(MOTOR_CONTROL_MAX_BUSES == CAN_RX_BUS_COUNT,
      "总线数契约：内核与 fdcan 必须一致");
  for (uint8_t bus = 0U; bus < MOTOR_CONTROL_MAX_BUSES; ++bus) {
    can_rx_ring_t *ring = can_bsp_rx_ring(bus);
    out->ring_drop[bus] = ring != nullptr ? ring->overflow_count : 0U;
  }
  // mailbox 层拒绝（整条消息在进入内核前就被拦下）。两半都要算：
  //   ① 外层授权门拒绝（*_rejected_count）
  //   ② **载荷本身非法**（内层 invalid_*_count）——NaN 快照 / 时间递减轨迹 /
  //      越权电机都落在这半。此前它只记在**内层**结构里、从没有任何导出路径，
  //      而调用方又把 submit 的返回值 (void) 掉了 → 台架上表现为「发布成功、
  //      既不计数也不出事件」，哪儿都查不到。
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

// 1ms 服务之一（前置）：SYNC 心跳——对 SYNC 能力协议（CERB）的总线周期发
// 0x080，维持设备 20ms 状态上报（上报帧即总线检测心跳，2026-09-05 协议核实）。
// 发现/热插拔窗口复用同一路径；vaxor 无 SYNC（adapter 返回 false，零开销）。
// 禁止升级为控制触发（ARD §23.4）。
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
    frame.is_extended = batch[i].is_extended;
    frame.is_fd = batch[i].is_fd;
    frame.is_brs = batch[i].is_brs;
    frame.size = (uint8_t)bytes;
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
    (void)motor_tx_router_pump(&g_tx_router, scheduler, bus,
        submit_routed_tx, nullptr);
  }
}

// 2ms/500Hz 控制边界：会话/授权边沿 → 内核；完整快照 → apply；配置 FIFO 有界转发。
//
// mailbox 类型与内核类型刻意互不依赖（同形镜像）；以下转换是任务层的
// 缝合职责，逐字段拷贝，不假设两侧布局一致。

static void convert_snapshot(const control_command_snapshot_t *in,
    motor_control_command_snapshot_t *out) {
  out->sequence = in->sequence;
  out->received_tick_ms = in->received_tick_ms;
  out->target_count = in->target_count;
  for (uint8_t i = 0; i < in->target_count && i < MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS;
      ++i) {
    out->targets[i].motor_id = in->targets[i].motor_id;
    out->targets[i].position_rad = in->targets[i].position;
    out->targets[i].velocity_rad_s = in->targets[i].velocity;
    out->targets[i].effort_nm = in->targets[i].effort;
  }
}

static void convert_policy(const control_policy_snapshot_t *in,
    motor_control_policy_snapshot_t *out) {
  out->received_tick_ms = in->received_tick_ms;
  out->request_id = in->request_id;
  out->policy_generation = in->policy_generation;
  out->target_count = in->target_count;
  for (uint8_t i = 0; i < in->target_count && i < MOTOR_CONTROL_MAX_MOTORS;
      ++i) {
    out->targets[i].motor_id = in->targets[i].motor_id;
    out->targets[i].enabled = in->targets[i].enabled;
    out->targets[i].kp = in->targets[i].kp;
    out->targets[i].kd = in->targets[i].kd;
    out->targets[i].confirm_unprotected = in->targets[i].confirm_unprotected;
  }
}

static void convert_trajectory(const control_trajectory_t *in,
    motor_control_trajectory_t *out) {
  out->sequence = in->sequence;
  out->received_tick_ms = in->received_tick_ms;
  out->trajectory_id = in->trajectory_id;
  out->point_count = in->point_count;
  out->target_count = in->target_count;
  for (uint16_t i = 0;
      i < in->target_count && i < MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS; ++i) {
    out->target_ids[i] = in->target_ids[i];
  }
  for (uint16_t i = 0;
      i < in->point_count && i < MOTOR_CONTROL_MAX_TRAJECTORY_POINTS; ++i) {
    out->points[i].time_from_start_ms = in->points[i].time_from_start_ms;
    out->points[i].motor_id = in->points[i].motor_id;
    out->points[i].position_rad = in->points[i].position;
    out->points[i].velocity_rad_s = in->points[i].velocity;
    out->points[i].effort_nm = in->points[i].effort;
  }
}

// 两侧枚举类型独立；motor_config_contract.hpp 锁住完整数值映射。
static void convert_config(const control_mailbox_config_request_t *in,
    motor_protocol_config_request_t *out) {
  out->request_id = in->request_id;
  out->motor_id = in->motor_id;
  // kind 数值契约一致（1..6、8..14；7 是内核专用 READ_IDENTITY 不经
  // mailbox，上方 static_assert 编译期锁），两侧枚举类型独立故显式转换。
  out->kind = (motor_protocol_config_kind_t)in->kind;
  switch (in->kind) {
  case CONTROL_MAILBOX_CONFIG_SET_MODE:
    out->mode = in->value.mode;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_GAINS:
    out->kp = in->value.gains.kp;
    out->kd = in->value.gains.kd;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS:
    out->position_limits.min_position_rad =
        in->value.position_limits.min_position_rad;
    out->position_limits.max_position_rad =
        in->value.position_limits.max_position_rad;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_ENABLE:
    out->enable = in->value.enable;
    out->confirm_unprotected = in->value.confirm_unprotected;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT:
    out->max_velocity_rad_s = in->value.max_velocity_rad_s;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT:
    out->max_torque_nm = in->value.max_torque_nm;
    break;
  case CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS:
    break;   // 无载荷：只凭 motor_id 发起 4 个 SDO upload
  case CONTROL_MAILBOX_CONFIG_SET_ZERO:
    break;   // 无载荷：只凭 motor_id 写 0x2531（设零位，仅失能态）
  case CONTROL_MAILBOX_CONFIG_CLEAR_FAULT:
    break;   // 无载荷：只凭 motor_id 发厂家清错帧
  case CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT:
    out->safety_protect_raw = in->value.safety_protect_raw;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_DIRECTION:
    out->direction_raw = in->value.direction_raw;
    break;
  case CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH:
    out->save_slots = in->value.save_slots;
    break;
  case CONTROL_MAILBOX_CONFIG_SET_POLICY:
    out->enable = in->value.policy.enable;
    out->kp = in->value.policy.kp;
    out->kd = in->value.policy.kd;
    out->policy_generation = in->value.policy.generation;
    out->confirm_unprotected = in->value.policy.confirm_unprotected;
    break;
  default:
    break;
  }
}

static void control_boundary(void) {
  // 会话/授权同步（纯逻辑在 session_sync）：临界区内一次取走边沿并读
  // 会话电平（未知=引导豁免），内核状态变更在临界区外执行。
  control_mailbox_events_t events = {0};
  bool session_authorized = false;
  taskENTER_CRITICAL();
  (void)control_mailbox_take_events(&g_control_mailbox, &events);
  session_authorized =
      control_mailbox_session_authorized(&g_control_mailbox);
  taskEXIT_CRITICAL();
  const uint32_t now_ms = millis();
  session_sync_apply(g_motor_control, &events, session_authorized, now_ms);

  // 完整命令快照（mailbox 无新快照时本边界不产生运动帧）。
  // 两个快照各约 1KB，改静态避免压栈（本函数仅 motor_rt 任务单上下文
  // 进入）：take 成功时整体覆盖、convert 只写 target_count 条、内核只读
  // target_count 条，静态尾部残留不可达。
  static control_command_snapshot_t snapshot_in;
  static motor_control_command_snapshot_t snapshot;
  bool got_snapshot = false;
  taskENTER_CRITICAL();
  got_snapshot = control_mailbox_take_command(&g_control_mailbox, &snapshot_in);
  taskEXIT_CRITICAL();
  if (got_snapshot) {
    convert_snapshot(&snapshot_in, &snapshot);
    (void)motor_control_apply_command_snapshot(g_motor_control, &snapshot,
        now_ms);
  }

  // 多点轨迹：先受理（校验在 mailbox 与内核各做一次），再按时间线插值
  // 下发。两份轨迹各约 5KB，改静态避免压栈（与快照同因）。无新轨迹时
  // take 失败；step 在无活动轨迹时 O(1) 返回。
  static control_trajectory_t trajectory_in;
  static motor_control_trajectory_t trajectory;
  bool got_trajectory = false;
  taskENTER_CRITICAL();
  got_trajectory =
      control_mailbox_take_trajectory(&g_control_mailbox, &trajectory_in);
  taskEXIT_CRITICAL();
  if (got_trajectory) {
    convert_trajectory(&trajectory_in, &trajectory);
    (void)motor_control_apply_trajectory(g_motor_control, &trajectory, now_ms);
  }
  motor_control_trajectory_step(g_motor_control, now_ms);
  // 快照通道的边界保持重发：与 trajectory_step 同拍、覆盖互斥的电机集合，
  // 两者合计使每个边界每台恰好产生一帧 P1（见 motor_control.h 的说明）。
  motor_control_hold_step(g_motor_control, now_ms);

  // 逐电机策略：批量、latest-wins。稀疏语义——只影响列出的电机，未列出的
  // 保持原状。约 0.8KB，静态避免压栈。
  static control_policy_snapshot_t policy_in;
  static motor_control_policy_snapshot_t policy;
  bool got_policy = false;
  taskENTER_CRITICAL();
  got_policy = control_mailbox_take_policy(&g_control_mailbox, &policy_in);
  taskEXIT_CRITICAL();
  if (got_policy) {
    convert_policy(&policy_in, &policy);
    (void)motor_control_apply_policy(g_motor_control, &policy, now_ms);
  }

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
    motor_protocol_config_request_t request = {0};
    convert_config(&request_in, &request);
    (void)motor_control_submit_config(g_motor_control, &request, now_ms);
  }

  // boot 存储参数：Profile 未绑定的电机保留待套用位，后续边界自动重试。
  // 无记录（count=0）时也必须调用：内核据此把"记录已就位（哪怕是没有记录）"
  // 告知每个表项，上电设备参数回读才会发起（见 motor_control_config_step 0b）。
  motor_control_apply_stored_params(g_motor_control, g_stored_params,
      g_stored_count, now_ms);

  // 落盘**不再自动**：写事务只改内存记录 + 置脏位，"保存"由人工发 system_command
  // 的 COMMAND_SAVE 触发（param_bridge 发 SAVE_DEVICE_FLASH 事务 + 落 W25）。此前
  // 这里 500ms 去抖自动落盘，于是"只点了写"与"点了保存"无法区分——正是 owner
  // 要求消掉的行为（设计 §3.2）。
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

  // FDCAN 三路硬件初始化归 lib/fdcan 门卫（bsp_can_init 建 ring/调度器/
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

    // 1ms 服务：RX 有界解码 → 运行时服务（离线/新鲜度/事务步进）→
    // SYNC 心跳 → 内核待发帧路由进调度器 → 事件搬运。
    drain_rx_rings();
    motor_control_service(g_motor_control, now_ms);
    sustain_discovery_sync(now_ms);
    route_kernel_tx();

    // 2ms 控制边界（漂移自校正：按 millis 差触发；500 Hz）。
    if ((uint32_t)(now_ms - last_control_ms) >= kControlPeriodMs) {
      last_control_ms = now_ms;
      control_boundary();
    }

    // 1ms 服务：每总线 P0/P1/P2 出队并提交硬件。
    pump_tx_schedulers();
  }
}

bool motor_rt_task_start(void) {
  if (g_task_started) {
    return true;
  }
  // 注入参数快照提供者：storage 任务 save 时回调（打破 storage→motor_rt
  // 任务头反向依赖；临界区仍在 motor_rt_copy_param_views 内部）。
  storage_task_set_param_snapshot(motor_rt_copy_param_views);
  // mailbox 与内核先于任务初始化；内核 adapter 列表构造注入。
  motor_tx_router_init(&g_tx_router);
  control_mailbox_init(&g_control_mailbox);
  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  motor_control_init(g_motor_control, adapters, adapter_count);
  // boot 存储参数快照（storage 任务先于本任务启动；无有效 blob 时 count=0）。
  // 保存进静态副本：W25 的新值下次启动才生效，运行期不再引用 storage 任务。
  size_t stored_count = 0U;
  const motor_control_stored_params_t *stored =
      storage_boot_params(&stored_count);
  if (stored_count > MOTOR_CONTROL_MAX_MOTORS) {
    stored_count = MOTOR_CONTROL_MAX_MOTORS;
  }
  memcpy(g_stored_params, stored,
      stored_count * sizeof(motor_control_stored_params_t));
  g_stored_count = stored_count;
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
