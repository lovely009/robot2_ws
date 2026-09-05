#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ID 驱动的电机运行时内核（ARD-0007 §8/§9/§13/§20；HANDOFF 阶段 A）。
//
// 定位：纯 C 运行时内核——注册表、型号 Profile 目录、生命周期、安全、
// freshness、状态/事件快照。不依赖 ROS、HAL、FreeRTOS、control_mailbox，
// 也不感知 joint_name（关节语义在 upper 层，AR-003）。
// 厂家字节只经 motor_protocol 的 adapter 接口（构造时注入 adapter 列表，
// 新增协议不改内核——ARD §24.3）。
//
// 所有权：motor_rt_task（Phase B）是唯一调用者。1ms 服务节拍调
// on_raw_frame()/service()；10ms 控制边界调 apply_command_snapshot()；
// TX 调度器（lib/fdcan）消费 collect_tx() 输出的 P0/P1/P2 帧。
//
// 时基：一切 freshness/超时判定使用 MCU 单调时基毫秒（调用方传入 now_ms
// 与帧内 rx_tick_ms），绝不用 ROS 时间（ARD §12）。

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_protocol_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// 静态容量（ARD-0007 §8.1：注册表容量 ≥64 且编译期固定；禁止静默截断——
// 满表拒绝并计数/出事件）。
// ---------------------------------------------------------------------------
#define MOTOR_CONTROL_MAX_MOTORS 64U
#define MOTOR_CONTROL_MAX_BUSES 3U
#define MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS 64U
#define MOTOR_CONTROL_MAX_TX_PENDING 96U
#define MOTOR_CONTROL_MAX_EVENTS 32U

// 命令快照安全阈值（ARD-0007 §13.2；HANDOFF 阶段 A-6）。
#define MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS 30U
#define MOTOR_CONTROL_SNAPSHOT_DISABLE_MS 100U

// 生命周期（ARD-0007 §9.1：九态，只有 ACTIVE 允许运动帧）。
typedef enum {
  MOTOR_CONTROL_LIFECYCLE_ABSENT = 0U,
  MOTOR_CONTROL_LIFECYCLE_DISCOVERED = 1U,
  MOTOR_CONTROL_LIFECYCLE_IDENTIFYING = 2U,
  MOTOR_CONTROL_LIFECYCLE_READY_DISABLED = 3U,
  MOTOR_CONTROL_LIFECYCLE_ACTIVE = 4U,
  MOTOR_CONTROL_LIFECYCLE_SAFE_STOP = 5U,
  MOTOR_CONTROL_LIFECYCLE_FAULT = 6U,
  MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT = 7U,
  MOTOR_CONTROL_LIFECYCLE_OFFLINE = 8U
} motor_control_lifecycle_t;

// 统一故障码（厂家原始故障字另行透传，映射在 adapter——ARD §24）。
typedef enum {
  MOTOR_CONTROL_FAULT_NONE = 0U,
  MOTOR_CONTROL_FAULT_PROTOCOL_ERROR = 1U,
  MOTOR_CONTROL_FAULT_VENDOR = 2U,
  MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE = 3U,
  MOTOR_CONTROL_FAULT_CONFIG_CONFLICT = 4U,
  MOTOR_CONTROL_FAULT_OFFLINE = 5U
} motor_control_fault_code_t;

// 事件（ARD-0007 §10.4/§18.2：拓扑变化必须可见；固定容量环形缓冲，挤掉即计数）。
typedef enum {
  MOTOR_CONTROL_EVENT_DISCOVERED = 0U,
  MOTOR_CONTROL_EVENT_PROFILE_BOUND = 1U,
  MOTOR_CONTROL_EVENT_PROFILE_REJECTED = 2U,
  MOTOR_CONTROL_EVENT_READY = 3U,
  MOTOR_CONTROL_EVENT_ACTIVE = 4U,
  MOTOR_CONTROL_EVENT_SOFT_STOP = 5U,
  MOTOR_CONTROL_EVENT_DISABLE = 6U,
  MOTOR_CONTROL_EVENT_FAULT = 7U,
  MOTOR_CONTROL_EVENT_CONFIG_CONFLICT = 8U,
  MOTOR_CONTROL_EVENT_OFFLINE = 9U,
  MOTOR_CONTROL_EVENT_RECONNECTED = 10U,
  MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST = 11U,
  MOTOR_CONTROL_EVENT_SNAPSHOT_REJECTED = 12U,
  MOTOR_CONTROL_EVENT_CONFIG_RESULT = 13U,
  MOTOR_CONTROL_EVENT_REGISTRY_FULL = 14U
} motor_control_event_type_t;

typedef struct {
  motor_control_event_type_t type;
  uint32_t tick_ms;
  uint16_t motor_id;
  uint8_t bus_index;
  uint16_t reason;    // 结果码/失败码（motor_control_fault_code_t 或 config result）
  uint32_t request_id; // 仅 CONFIG_RESULT 携带
} motor_control_event_t;

// ---------------------------------------------------------------------------
// 输入：完整命令快照（mailbox control_command_snapshot_t 的同形镜像；
// 内核不 include mailbox 头，任务层负责拷贝转换）。
// 语义：完整快照（ARD §13.1），受控集合中缺席的电机必须软停，
// 禁止沿用上一周期目标（禁止项 8/13.1）。
// ---------------------------------------------------------------------------
typedef struct {
  uint16_t motor_id;
  float position_rad;
  float velocity_rad_s;
  float effort_nm;
} motor_control_snapshot_target_t;

typedef struct {
  uint32_t sequence;
  uint32_t received_tick_ms; // mailbox 记录的本地接收 tick（freshness 基准）
  uint8_t target_count;
  motor_control_snapshot_target_t targets[MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS];
} motor_control_command_snapshot_t;

// 输出：运动状态（100 Hz 发布用，ARD §16；缺失浮点 = NaN，state_age_ms
// 为 MCU 单调时基下最近有效运动测量距今毫秒数）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  motor_control_lifecycle_t lifecycle;
  bool online;
  float position_rad;
  float velocity_rad_s;
  float effort_nm;
  uint32_t state_age_ms;
} motor_control_motion_state_t;

// 输出：诊断状态（ARD §17；统一故障码 + 厂家故障字 + 公共位标志）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  motor_control_lifecycle_t lifecycle;
  bool online;
  float temperature_c;
  float current_a;
  float voltage_v;
  uint8_t drive_mode;
  uint16_t unified_fault_code;
  uint32_t vendor_fault_code;
  uint32_t protection_flags;
  uint32_t limit_flags;
  uint32_t state_age_ms;
  uint8_t transaction_state; // 0=空闲，1=配置事务进行中
  uint16_t last_failure_reason;
} motor_control_status_t;

// ---------------------------------------------------------------------------
// 运行时上下文（静态分配；字段仅供内核实现读写，调用方走函数接口）。
// ---------------------------------------------------------------------------
typedef struct {
  bool used;
  uint16_t motor_id; // 全局唯一（当前派生：= 厂家 node 号；冲突即 CONFIG_CONFLICT）
  uint8_t bus_index;
  uint16_t node;
  const motor_protocol_adapter_t *adapter;
  const motor_protocol_model_profile_t *profile; // NULL = 未绑定（不可控）
  motor_control_lifecycle_t lifecycle;
  bool online;

  uint32_t last_rx_tick_ms;       // 任意本机帧（OFFLINE 判定）
  uint32_t last_motion_rx_tick_ms; // 最近有效运动反馈（state_age）
  uint32_t last_commanded_tick_ms; // 最近 MIT/使能帧（CREB OFFLINE 判定）

  bool has_target;
  float target_position_rad;
  float target_velocity_rad_s;
  float target_effort_nm;

  // Policy（低频，不进入 100 Hz 命令——ARD §19.4）。
  bool policy_enable;   // 逐电机 Enable（MotorPolicy；OFFLINE 冻结为 false，
                        // 恢复需上层显式重发 SET_ENABLE(true)——ARD §36）
  bool has_kp_kd;
  float kp;
  float kd;
  bool has_soft_limits; // 仅 Vaxor（软件限位，打包前钳制——ARD §26.1）
  float soft_min_rad;
  float soft_max_rad;

  // 使能时序（vendor 整定门：adapter->enable_settle_ms）。
  bool motion_enabled;    // 已发使能帧且未被失能
  bool enable_pending;    // 整定窗口内，暂不发运动目标
  uint32_t enable_started_tick_ms;
  // 逐电机运动授权（ARD §36/§10.4：OFFLINE 即冻结，重连后由"完成重新发现流程
  // 后被新完整快照显式纳入"复立；正式的逐电机 Enable 等 Phase C 的 MotorPolicy）。
  bool motion_authorized;

  // P0 帧重试：推送失败（缓冲满且无可驱逐的 P1/P2）时置位，service 逐节拍重发；
  // 新使能/MIT 帧成功后作废（新命令取代过期的安全帧）。
  bool p0_pending;
  uint8_t p0_kind; // 0=无，1=软停，2=失能

  motor_protocol_measured_state_t measured;
  bool has_measured;

  // 诊断（ARD §8.1/§32.2）。
  uint16_t unified_fault_code;
  uint32_t vendor_fault_code;
  uint32_t protection_flags;
  uint32_t limit_flags;
  uint32_t protocol_error_count;
  uint16_t last_failure_reason;

  // 配置事务（每电机至多一个未决；新请求 BUSY——ARD §20.3）。
  bool tx_active;
  bool tx_awaiting;
  uint32_t tx_request_id;
  motor_protocol_config_kind_t tx_kind;
  motor_protocol_config_request_t tx_request; // 事务全程持有请求副本
  uint8_t tx_stage;
  uint8_t tx_retries;
  uint32_t tx_deadline_tick_ms;
} motor_control_entry_t;

typedef struct {
  // adapter 注册表（构造注入；测试注入 mock）。
  const motor_protocol_adapter_t *const *adapters;
  size_t adapter_count;

  motor_control_entry_t entries[MOTOR_CONTROL_MAX_MOTORS];
  size_t entry_count;

  // 会话与授权（ARD §13.2/§33：断链即协议级停机并清授权；
  // 恢复需要 新授权 + 新完整快照）。
  bool agent_connected;
  bool agent_ever_connected;
  bool authorized;
  bool require_new_snapshot;

  // 快照 freshness（单调时基）。
  bool have_snapshot;
  uint32_t last_snapshot_tick_ms;
  uint32_t last_snapshot_sequence;
  bool snapshot_soft_stop_active; // 30ms 级联已触发（去重）
  bool snapshot_disable_done;     // 100ms 级联已触发（去重）

  // 待发送帧缓冲（collect_tx 消费；满则丢弃并计数）。
  motor_protocol_tx_frame_t tx_pending[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t tx_count;
  uint32_t tx_dropped_count;
  uint32_t p0_retry_count; // P0 帧因缓冲全满而挂起重试的次数（诊断可见）

  // 事件环。
  motor_control_event_t events[MOTOR_CONTROL_MAX_EVENTS];
  size_t event_count;
  uint32_t event_dropped_count;

  // 诊断计数（ARD §32）。
  uint32_t rx_frame_count;
  uint32_t unknown_frame_count;
  uint32_t decode_error_count;
  uint32_t snapshot_accepted_count;
  uint32_t snapshot_rejected_count;
  uint32_t config_rejected_count;
} motor_control_context_t;

// ---------------------------------------------------------------------------
// 生命周期管理
// ---------------------------------------------------------------------------

// 初始化空注册表。adapter 列表由调用方提供（生命周期覆盖调用）。
// 注意：上电默认 *未* 授权（安全默认）——任务层启动时应把 mailbox 的
// 授权电平转发进来（mailbox 引导期默认 true）。
void motor_control_init(motor_control_context_t *ctx,
                        const motor_protocol_adapter_t *const *adapters,
                        size_t adapter_count);

// 会话边沿。断链（曾连接后断开）立即对全部可控电机发协议级停机/失能（P0）、
// 清运动授权与目标，不等待 100ms（ARD §13.2/§33）。
void motor_control_set_agent_connected(motor_control_context_t *ctx, bool connected,
                                       uint32_t now_ms);

// 授权门（≡ Enable，mailbox 冻结契约为全局授权；逐电机 Enable 等 Phase C
// 的 MotorPolicy）。撤销授权执行与断链相同的安全路径；
// 恢复授权后必须等新的完整快照才可能回 ACTIVE（require_new_snapshot）。
bool motor_control_set_authorization(motor_control_context_t *ctx, bool authorized,
                                     uint32_t now_ms);

// Profile 绑定（TBD-PROFILE-01 过渡：识别源关闭前由上层显式绑定型号；
// 型号目录不含 Vaxor-80 → 绑定被拒并出事件，该电机永不出运动帧）。
bool motor_control_bind_profile(motor_control_context_t *ctx, uint16_t motor_id,
                                size_t profile_index, uint32_t now_ms);

// 指针版绑定（深层接口）：供测试注入自定义 Profile，或未来接入目录之外的
// 识别来源。同样受协议一致性与生命周期门禁约束。
bool motor_control_bind_profile_ptr(motor_control_context_t *ctx, uint16_t motor_id,
                                    const motor_protocol_model_profile_t *profile,
                                    uint32_t now_ms);

// 低频配置入口（mailbox config FIFO 逐条转发）。SET_GAINS/SET_POSITION_LIMITS
// 在 Vaxor 上是本地 Policy；SET_MODE 与 CREB 限位走 request_id 事务。
// 结果统一经 CONFIG_RESULT 事件异步给出；本函数返回值仅表示"已受理/校验拒绝"。
bool motor_control_submit_config(motor_control_context_t *ctx,
                                 const motor_protocol_config_request_t *request,
                                 uint32_t now_ms);

// ---------------------------------------------------------------------------
// 实时路径（1ms 服务节拍）
// ---------------------------------------------------------------------------

// 原始帧输入（ISR→RX ring→任务 drain 后逐帧投递；帧内 rx_tick_ms 必填）。
void motor_control_on_raw_frame(motor_control_context_t *ctx,
                                const motor_protocol_raw_frame_t *frame);

// 1ms 服务：OFFLINE 判定、快照 30/100ms 级联、使能整定、配置事务步进。
// 有界：不等待设备、不等待 Agent、不动态分配。
void motor_control_service(motor_control_context_t *ctx, uint32_t now_ms);

// ---------------------------------------------------------------------------
// 10ms 控制边界（ARD §11.3）：校验并消费一份完整快照；
// 受控集合缺席电机立即软停（P0）。返回 false = 快照被拒（非法/回退序号）。
// ---------------------------------------------------------------------------
bool motor_control_apply_command_snapshot(motor_control_context_t *ctx,
                                          const motor_control_command_snapshot_t *snapshot,
                                          uint32_t now_ms);

// 发现/热插拔探测与 SYNC 心跳（P2）：对 SYNC 能力协议（CREB）的总线发
// 0x080——维持设备 20ms 周期状态上报（总线检测心跳，任务层周期调用；
// 发现窗口复用同一路径）。禁止作为 100Hz 控制触发（ARD §23.4）。
bool motor_control_probe_discovery(motor_control_context_t *ctx, uint8_t bus_index);

// ---------------------------------------------------------------------------
// 输出（固定容量副本；只读消费）
// ---------------------------------------------------------------------------

// 取走待发送帧（最多 capacity 帧，余量保留待下次）。Phase B 的每总线
// TX 调度器按 tx_class 分配 P0/P1/P2 队列。
size_t motor_control_collect_tx(motor_control_context_t *ctx,
                                motor_protocol_tx_frame_t *out, size_t capacity);

size_t motor_control_copy_motion_states(motor_control_context_t *ctx,
                                        motor_control_motion_state_t *out,
                                        size_t capacity, uint32_t now_ms);
size_t motor_control_copy_statuses(motor_control_context_t *ctx,
                                   motor_control_status_t *out, size_t capacity,
                                   uint32_t now_ms);
size_t motor_control_drain_events(motor_control_context_t *ctx,
                                  motor_control_event_t *out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
