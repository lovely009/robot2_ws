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
#include "motor_event_relay.h"
#include "motor_protocol_adapter.h"

// 任务参数：栈 1024 字（4KB）。1ms 节拍任务的栈/WCET 需按台架数据复测
// （TBD-RT-01），不得沿用 10ms 旧任务的假设。
static const uint32_t kMotorRtTaskStack = 1024U;
static const UBaseType_t kMotorRtTaskPrio = 3U;
static const TickType_t kServicePeriodTicks = pdMS_TO_TICKS(1U);
static const uint32_t kControlPeriodMs = 10U;

// 每周期配额（TBD-CAN-01 占位；均为具名可调常量）。
static const uint32_t kRxDrainBudgetPerBus = 16U;  // 每 1ms 每总线最多解码帧数
static const uint32_t kRxPopBatch = 8U;            // 单次批量弹出（栈上界）
static const uint32_t kTxCollectBudget = 16U;      // 每 1ms 最多取走内核待发帧数
static const uint32_t kTxPumpBudgetPerBus = 8U;    // 每 1ms 每总线最多提交帧数
static const uint32_t kConfigDrainPerBoundary = 4U; // 每 10ms 边界最多转发配置数
static const uint32_t kBindDrainPerBoundary = 4U;   // 每 10ms 边界最多转发绑定数
static const uint32_t kDiscoverySyncPeriodMs = 10U; // SYNC 心跳周期（维持设备
                                                    // 20ms 状态上报，总线检测）

static motor_control_context_t g_motor_control;
static control_mailbox_t g_control_mailbox;
static TaskHandle_t g_task_handle = nullptr;
static bool g_task_started = false;
static volatile bool g_fdcan_ready = false;

// 协议 adapter 注册表（构造注入内核；新增协议 = 加一项，内核零改动）。
static const motor_protocol_adapter_t *const kAdapters[2] = {
    &motor_protocol_vaxor_adapter,
    &motor_protocol_cerb_adapter,
};

// ---------------------------------------------------------------------------
// mailbox 写入口（外部任务上下文；短临界区保护 mailbox）
// ---------------------------------------------------------------------------

bool motor_rt_submit_command(const control_command_snapshot_t *snapshot) {
  taskENTER_CRITICAL();
  const bool ok = control_mailbox_submit_command(&g_control_mailbox, snapshot);
  taskEXIT_CRITICAL();
  return ok;
}

bool motor_rt_submit_config(const control_mailbox_config_request_t *request) {
  taskENTER_CRITICAL();
  const bool ok = control_mailbox_submit_config(&g_control_mailbox, request);
  taskEXIT_CRITICAL();
  return ok;
}

bool motor_rt_submit_bind(uint16_t motor_id, const char *model_name) {
  taskENTER_CRITICAL();
  const bool ok =
      control_mailbox_submit_bind(&g_control_mailbox, motor_id, model_name);
  taskEXIT_CRITICAL();
  return ok;
}

void motor_rt_request_authorization(bool authorized) {
  taskENTER_CRITICAL();
  control_mailbox_request_authorization(&g_control_mailbox, authorized);
  taskEXIT_CRITICAL();
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
      motor_control_copy_motion_states(&g_motor_control, out, capacity, now_ms);
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
      motor_control_copy_statuses(&g_motor_control, out, capacity, now_ms);
  taskEXIT_CRITICAL();
  return count;
}

bool motor_rt_task_started(void) { return g_task_started; }

bool motor_rt_task_fdcan_ready(void) { return g_fdcan_ready; }

// ---------------------------------------------------------------------------
// 内核事件中继（任务 1ms 搬运；桥在轮询里取走）与拓扑副本
// ---------------------------------------------------------------------------

// SPSC 中继环（lib/motor_event_relay）：本任务生产、ROS 桥任务消费。
static motor_event_relay_t g_event_relay;

static void forward_kernel_events(void) {
  // 内核事件环由本任务独占；共享中继环的每次 push 与桥侧 drain 使用同一
  // 临界区，确保生产与消费互斥。
  motor_control_event_t drained[8];
  size_t n;
  while ((n = motor_control_drain_events(&g_motor_control, drained, 8)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      taskENTER_CRITICAL();
      motor_event_relay_push(&g_event_relay, &drained[i]);
      taskEXIT_CRITICAL();
    }
  }
}

uint32_t motor_rt_event_dropped(void) {
  taskENTER_CRITICAL();
  const uint32_t dropped = motor_event_relay_dropped_count(&g_event_relay);
  taskEXIT_CRITICAL();
  return dropped;
}

size_t motor_rt_drain_events(motor_control_event_t *out, size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  taskENTER_CRITICAL();
  const size_t count = motor_event_relay_drain(&g_event_relay, out, capacity);
  taskEXIT_CRITICAL();
  return count;
}

size_t motor_rt_copy_topology(motor_rt_topology_entry_t *out, size_t capacity) {
  if (out == nullptr || capacity == 0U) {
    return 0U;
  }
  taskENTER_CRITICAL();
  // 发布顺序确定化：按 motor_id 升序（同内核 copy_motion_states 注释）。
  bool taken[MOTOR_CONTROL_MAX_MOTORS] = {};
  size_t count = 0U;
  for (;;) {
    if (count >= capacity) {
      break;
    }
    int best = -1;
    for (size_t i = 0; i < g_motor_control.entry_count; ++i) {
      const motor_control_entry_t *candidate = &g_motor_control.entries[i];
      if (!candidate->used || taken[i]) {
        continue;
      }
      if (best < 0 ||
          candidate->motor_id < g_motor_control.entries[best].motor_id) {
        best = (int)i;
      }
    }
    if (best < 0) {
      break;
    }
    taken[best] = true;
    const motor_control_entry_t *entry = &g_motor_control.entries[best];
    motor_rt_topology_entry_t *topo = &out[count++];
    topo->motor_id = entry->motor_id;
    topo->bus_index = entry->bus_index;
    topo->protocol = 0xFFU; // 未登记协议（防御；当前两协议均可命中）
    for (size_t k = 0; k < sizeof(kAdapters) / sizeof(kAdapters[0]); ++k) {
      if (kAdapters[k] == entry->adapter) {
        topo->protocol = (uint8_t)k; // 注册顺序 = MotorTopologyEntry 协议枚举
        break;
      }
    }
    topo->model_name[0] = '\0';
    if (entry->profile != nullptr && entry->profile->model_name != nullptr) {
      strncpy(topo->model_name, entry->profile->model_name,
              sizeof(topo->model_name) - 1U);
      topo->model_name[sizeof(topo->model_name) - 1U] = '\0';
    }
    topo->capability_flags =
        entry->profile != nullptr ? entry->profile->capability_flags : 0U;
    topo->online = entry->online;
    topo->conflict =
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
    topo->lifecycle = (uint8_t)entry->lifecycle;
  }
  taskEXIT_CRITICAL();
  return count;
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
        motor_control_on_raw_frame(&g_motor_control, &frame);
      }
      drained += popped;
    }
  }
}

// 1ms 服务之二（前置）：SYNC 心跳——对 SYNC 能力协议（CERB）的总线周期发
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
    (void)motor_control_probe_discovery(&g_motor_control, bus);
  }
}

// 1ms 服务之二：内核待发队列 → 每总线调度器（P0 FIFO / P1 latest-wins 槽 /
// P2 FIFO）。tx_slot = 内核表项索引（表项只增不减故稳定，同总线电机槽互异）；
// motor_protocol_tx_frame_t 与 can_tx_frame_t 显式逐字段拷贝（不假设布局一致）。
static void route_kernel_tx(void) {
  static motor_protocol_tx_frame_t batch[kTxCollectBudget]; // 静态：固定容量
  const size_t count =
      motor_control_collect_tx(&g_motor_control, batch, kTxCollectBudget);
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
          (void)motor_control_requeue_p0(&g_motor_control, &batch[i]);
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

// 1ms 服务之三：每总线 TX 调度器 pump → 硬件提交。
// 每周期恰好一次 pump：P2 配额按"每 pump"计，循环调用会叠加配额。
static void pump_tx_schedulers(void) {
  static can_tx_frame_t pump[kTxPumpBudgetPerBus]; // 静态：固定容量、可审计
  for (uint8_t bus = 0U; bus < CAN_TX_BUS_COUNT; ++bus) {
    can_tx_scheduler_t *sched = can_bsp_tx_scheduler(bus);
    if (sched == nullptr) {
      continue;
    }
    uint32_t p0_count = 0U;
    const uint32_t n =
        can_tx_scheduler_pump(sched, pump, kTxPumpBudgetPerBus, &p0_count);
    for (uint32_t i = 0; i < n; ++i) {
      if (!can_bsp_submit_tx(bus, &pump[i])) {
        // 回插 P0 队首，下一拍先于其他帧重试，确保离线电机的失能帧
        // 在总线恢复后落地。P1 由下一周期最新目标取代，P2 为管理流量；
        // 二者只保留拒收计数，不重试。
        if (i < p0_count) {
          (void)can_tx_scheduler_requeue_p0_front(sched, &pump[i]);
        }
      }
    }
  }
}

// 10ms 控制边界：会话/授权边沿 → 内核；完整快照 → apply；配置 FIFO 有界转发。
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

// mailbox 与内核协议层的配置 kind 枚举类型独立，但数值必须逐值相等；
// convert_config 的显式转换依赖此前提，以下断言防止单侧修改。
static_assert(static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_MODE) ==
                  static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_MODE),
              "mailbox/protocol config kind 契约不一致：SET_MODE");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_GAINS) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_GAINS),
    "mailbox/protocol config kind 契约不一致：SET_GAINS");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS),
    "mailbox/protocol config kind 契约不一致：SET_POSITION_LIMITS");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_ENABLE) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_ENABLE),
    "mailbox/protocol config kind 契约不一致：SET_ENABLE");

static void convert_config(const control_mailbox_config_request_t *in,
                           motor_protocol_config_request_t *out) {
  out->request_id = in->request_id;
  out->motor_id = in->motor_id;
  // kind 数值契约一致（1/2/3/4，上方 static_assert 编译期锁），两侧枚举
  // 类型独立故显式转换。
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
      break;
    default:
      break;
  }
}

static void control_boundary(void) {
  static bool session_synced = false;

  // mailbox 并发契约：读侧 take_* 与写侧 submit_* 必须互斥（库无内部锁）。
  control_mailbox_events_t events = {0};
  bool has_edges = false;
  taskENTER_CRITICAL();
  has_edges = control_mailbox_take_events(&g_control_mailbox, &events);
  taskEXIT_CRITICAL();
  const uint32_t now_ms = millis();
  // 启动首边界先同步一次电平（mailbox 引导豁免的授权电平必须镜像到内核）。
  if (has_edges || !session_synced) {
    if (events.agent_state_pending || !session_synced) {
      motor_control_set_agent_connected(&g_motor_control,
                                        events.agent_connected_requested, now_ms);
    }
    if (events.authorization_pending || !session_synced) {
      motor_control_set_authorization(&g_motor_control,
                                      events.authorization_requested, now_ms);
    }
    session_synced = true;
  }

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
    (void)motor_control_apply_command_snapshot(&g_motor_control, &snapshot,
                                               now_ms);
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
    (void)motor_control_submit_config(&g_motor_control, &request, now_ms);
  }

  // 身份绑定 FIFO 有界转发（TBD-PROFILE-01 过渡通道；断链不清，重连后
  // 绑定仍有效；结果经内核 PROFILE_BOUND/PROFILE_REJECTED 事件回告）。
  for (uint32_t i = 0; i < kBindDrainPerBoundary; ++i) {
    control_mailbox_bind_request_t bind_in = {0};
    bool got_bind = false;
    taskENTER_CRITICAL();
    got_bind = control_mailbox_take_bind(&g_control_mailbox, &bind_in);
    taskEXIT_CRITICAL();
    if (!got_bind) {
      break;
    }
    (void)motor_control_bind_profile_by_name(&g_motor_control, bind_in.motor_id,
                                             bind_in.model_name, now_ms);
  }
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

static void watchdog_feed(void) { (void)HAL_IWDG_Refresh(&g_iwdg); }
#else
static void watchdog_start(void) {}
static void watchdog_feed(void) {}
#endif

static void motor_rt_task_fn(void *pv_parameters) {
  (void)pv_parameters;

  // FDCAN 三路硬件初始化归 lib/fdcan 门卫（bsp_can_init 建 ring/调度器/
  // 过滤器并启动硬件）；任务层不直接触碰 HAL 句柄。
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  bsp_can_init();
  g_fdcan_ready = true;

  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_control_ms = millis();

  for (;;) {
    watchdog_feed(); // 活性监督：本循环停摆 ≥ 超时即复位
    vTaskDelayUntil(&last_wake, kServicePeriodTicks);
    const uint32_t now_ms = millis();

    // 1ms 服务：RX 有界解码 → 运行时服务（离线/新鲜度/事务步进）→
    // SYNC 心跳 → 内核待发帧路由进调度器 → 事件搬运。
    drain_rx_rings();
    motor_control_service(&g_motor_control, now_ms);
    sustain_discovery_sync(now_ms);
    route_kernel_tx();
    forward_kernel_events();

    // 10ms 控制边界（漂移自校正：按 millis 差触发）。
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
  // mailbox 与内核先于任务初始化；内核 adapter 列表构造注入。
  control_mailbox_init(&g_control_mailbox);
  motor_control_init(&g_motor_control, kAdapters,
                     sizeof(kAdapters) / sizeof(kAdapters[0]));
  motor_event_relay_init(&g_event_relay);
  watchdog_start(); // IWDG 就绪后再拉起实时任务，首拍即开始喂狗

  if (xTaskCreate(motor_rt_task_fn, "motor_rt", kMotorRtTaskStack, nullptr,
                  kMotorRtTaskPrio, &g_task_handle) != pdPASS) {
    // 任务创建失败是致命配置错误：停在这里，禁止系统带残缺实时层运行。
    Error_Handler();
    return false;
  }
  g_task_started = true;
  return true;
}
