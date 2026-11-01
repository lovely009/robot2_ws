#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ID 驱动的电机运行时内核：注册表、型号 Profile 目录、生命周期、安全、freshness、状态/事件快照。
//
// 不依赖 ROS/HAL/FreeRTOS/control_mailbox，也不感知 joint_name（关节语义在 upper 层）；厂家字节只
// 经 motor_protocol 的 adapter 接口——构造时注入 adapter 列表，新增协议不改内核。
// motor_rt_task 是唯一调用者：1ms 服务调 on_raw_frame()/service()，可配控制边界调
// apply_command_snapshot()，TX 调度器消费 collect_tx() 的 P0/P1/P2 帧。
// 一切 freshness/超时判定用 MCU 单调时基毫秒（now_ms 与帧内 rx_tick_ms），绝不用 ROS 时间。

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "firmware_config.h"
#include "motor_protocol_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// 静态容量由固件配置统一约束；满表拒绝并计数/出事件，不静默截断。
// ---------------------------------------------------------------------------
#define MOTOR_CONTROL_MAX_MOTORS FIRMWARE_MAX_MOTORS
#define MOTOR_CONTROL_MAX_BUSES 3U
#define MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS FIRMWARE_MAX_MOTORS
#define MOTOR_CONTROL_MAX_TX_PENDING 160U
// 事件环为并发发现预留余量；身份绑定由 config_step 限速为每毫秒一台。
#define MOTOR_CONTROL_MAX_EVENTS 128U
// 轨迹点数与电机数决定静态 RAM 上界；拼接、mailbox、任务转换、
// 内核锁存与切矢表均占内存，评估时须以链接结果为准。
#define MOTOR_CONTROL_MAX_TRAJECTORY_POINTS 256U
#define MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS FIRMWARE_MAX_MOTORS

// 命令源标识（target_source）：最近一次实际生效的命令来自哪条通道。
#define MOTOR_CONTROL_TARGET_SOURCE_NONE 0U
#define MOTOR_CONTROL_TARGET_SOURCE_SNAPSHOT 1U
#define MOTOR_CONTROL_TARGET_SOURCE_TRAJECTORY 2U

// 命令快照安全阈值（ARD-0007 §13.2）。
#define MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS 30U
#define MOTOR_CONTROL_SNAPSHOT_DISABLE_MS 100U

// 设备参数上电回读的尝试上限与退避（读是只读事务，重试无副作用）。
#define MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS 3U
#define MOTOR_CONTROL_DEV_READ_RETRY_MS 500U

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

// 控制生命周期与旧运行态分离；数值对齐 ROS 2，ACTIVE 表示 MCU 控制授权。
typedef enum {
  MOTOR_CONTROL_CONTROL_LIFECYCLE_UNKNOWN = 0U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_UNCONFIGURED = 1U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_INACTIVE = 2U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVE = 3U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_FINALIZED = 4U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_CONFIGURING = 10U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVATING = 13U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_DEACTIVATING = 14U,
  MOTOR_CONTROL_CONTROL_LIFECYCLE_ERROR_PROCESSING = 15U
} motor_control_control_lifecycle_t;

typedef enum {
  MOTOR_CONTROL_EXECUTION_UNKNOWN = 0U,
  MOTOR_CONTROL_EXECUTION_IDLE = 1U,
  MOTOR_CONTROL_EXECUTION_HOLDING = 2U,
  MOTOR_CONTROL_EXECUTION_COMMAND = 3U,
  MOTOR_CONTROL_EXECUTION_TRAJECTORY = 4U,
  MOTOR_CONTROL_EXECUTION_SOFT_STOP = 5U,
  MOTOR_CONTROL_EXECUTION_ENABLING = 6U,
  MOTOR_CONTROL_EXECUTION_DISABLING = 7U
} motor_control_execution_state_t;

// 统一故障码（厂家原始故障字另行透传，映射在 adapter——ARD §24）。
typedef enum {
  MOTOR_CONTROL_FAULT_NONE = 0U,
  MOTOR_CONTROL_FAULT_PROTOCOL_ERROR = 1U,
  MOTOR_CONTROL_FAULT_VENDOR = 2U,
  MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE = 3U,
  MOTOR_CONTROL_FAULT_CONFIG_CONFLICT = 4U,
  MOTOR_CONTROL_FAULT_OFFLINE = 5U,
  // 设备参数核对诊断码，仅适用于设备侧限位型号。当前核对结论不限制运动。
  MOTOR_CONTROL_FAULT_DEV_PARAM = 6U,
  // 策略代数未前进（SET_POLICY：「未武装 → 武装」需要新意图）。只写入
  // last_failure_reason 供本机诊断，不进 unified_fault_code——它不是设备
  // 故障，而是本机拒绝了一条重复的授权请求。
  MOTOR_CONTROL_FAULT_STALE_GENERATION = 7U,
  // 值 8 保留，不得复用。0x2541 只改力矩方向，不改坐标系，但可能导致位置环正反馈。
  // 保护确知关闭时，武装须显式确认；拒绝仅写 last_failure_reason，不改生命周期。
  MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED = 9U
} motor_control_fault_code_t;

// 上电/重连回读设备并与固件记录原值比对；当前仅报告结论，不限制运动。
typedef enum {
  MOTOR_CONTROL_DEV_INIT_NA = 0U,        // 非设备侧型号（VAXOR）：无从比对
  MOTOR_CONTROL_DEV_INIT_PENDING = 1U,   // 尚未回读设备（上电/重连后）
  MOTOR_CONTROL_DEV_INIT_OK = 2U,        // 有记录且逐槽一致
  MOTOR_CONTROL_DEV_INIT_MISMATCH = 3U,  // 有记录、某些槽不一致 → 未初始化
  MOTOR_CONTROL_DEV_INIT_UNVERIFIED = 4U, // 回读失败（超时/中止）→ 未核对
  // 无设备参数基线；与无设备对象的 NA 区分。部分槽位有记录时逐槽比对。
  MOTOR_CONTROL_DEV_INIT_NO_RECORD = 5U
} motor_control_dev_init_t;

// 限值来源（参数视图用）：设备回读 > Flash 记录 > 型号标定。
// 页面据此标注"设备真值"还是"离线回落值"，不把两种来源混成同一串数字。
typedef enum {
  MOTOR_CONTROL_LIMITS_SOURCE_NONE = 0U,    // 无有效值
  MOTOR_CONTROL_LIMITS_SOURCE_PROFILE = 1U, // 型号标定默认
  MOTOR_CONTROL_LIMITS_SOURCE_RECORD = 2U,  // Flash 记录（电机未上线时回落）
  MOTOR_CONTROL_LIMITS_SOURCE_DEVICE = 3U   // 设备回读（权威）
} motor_control_limits_source_t;

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
  MOTOR_CONTROL_EVENT_REGISTRY_FULL = 14U,
  // 设备参数初始化定论（reason = motor_control_dev_init_t）。不一致/未核对
  // 是运维可见事实：出事件以便上层定位是哪一台、什么结论。
  MOTOR_CONTROL_EVENT_DEV_INIT = 15U,
// 轨迹被整条拒绝（结构与 mailbox 同规则，另加注册表存在性）：reason =
// motor_control_fault_code_t。轨迹合法但被拒属于异常路径（mailbox 已先
// 校验过一遍），出事件便于定位是哪一侧放行的非法载荷。
  MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED = 16U
} motor_control_event_type_t;

typedef struct {
  motor_control_event_type_t type;
  uint32_t tick_ms;
  uint16_t motor_id;
  uint8_t bus_index;
  uint16_t reason;    // 结果码/失败码（motor_control_fault_code_t 或 config
                      // result；PROFILE_REJECTED 事件复用
                      // MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE=3 表示型号
                      // 不可得——未知名/设备未发现/门禁拒绝）
  uint32_t request_id; // 仅 CONFIG_RESULT 携带
  uint32_t seq;        // 内核分配，单调递增（消费者据此判丢帧）
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
  uint32_t received_tick_ms; // mailbox 记录的本地接收 tick（租约基准）
  uint32_t valid_until_tick_ms; // 原始执行截止，不从出队时重新续期
  uint8_t target_count;
  motor_control_snapshot_target_t targets[MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS];
} motor_control_command_snapshot_t;

// 输入：mailbox 已完成拼接的多点轨迹（control_trajectory_t 的同形镜像）。
// 语义：轨迹声明集合内的电机由轨迹独占（快照对其既不下发也不软停）；
// 集合外的电机仍由完整快照负责。起点 = 内核受理时刻的本地单调时基，
// 消息里的 sync_time_ns 不参与任何裁决（ARD §12）。
// ---------------------------------------------------------------------------
typedef struct {
  uint32_t time_from_start_ms; // 相对轨迹起点；同一电机内严格递增
  uint16_t motor_id;
  float position_rad;
  float velocity_rad_s; // 诊断用；执行速度由非均匀 Hermite 解析求导
  float effort_nm;
} motor_control_trajectory_point_t;

typedef struct {
  uint32_t sequence;
  uint32_t received_tick_ms;
  uint32_t trajectory_id;
  uint16_t point_count;
  uint16_t target_count;
  uint16_t target_ids[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
  motor_control_trajectory_point_t points[MOTOR_CONTROL_MAX_TRAJECTORY_POINTS];
} motor_control_trajectory_t;

// ---------------------------------------------------------------------------
// 输入：逐电机策略快照（mailbox control_policy_snapshot_t 的同形镜像）。
// **稀疏语义**：只影响 targets 里列出的电机，未列出的保持原状——这不是
// 命令快照的「缺席即软停」。整条校验，非法即整条拒绝、绝不部分采纳。
// ---------------------------------------------------------------------------
typedef struct {
  uint16_t motor_id;
  bool enabled;
  float kp;
  float kd;
  // 该台设备侧保护**确知为关**时的显式武装确认（每次都要）。它不是代数门：
  // policy_generation 承担「未武装 → 武装需新意图」，两者语义不同、不能互替。
  bool confirm_unprotected;
} motor_control_policy_target_t;

typedef struct {
  uint32_t received_tick_ms;
  uint32_t request_id;
  uint32_t policy_generation;
  uint8_t target_count;
  motor_control_policy_target_t targets[MOTOR_CONTROL_MAX_MOTORS];
} motor_control_policy_snapshot_t;

// ---------------------------------------------------------------------------
// 输出：运动状态副本（50 Hz 发布用，ARD §16；缺失浮点 = NaN，state_age_ms
// 为 MCU 单调时基下最近有效运动测量距今毫秒数，从无测量 = 0xFFFFFFFF）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  motor_control_lifecycle_t lifecycle;
  bool online;
  float position_rad;
  float velocity_rad_s;
  float effort_nm;
  uint32_t state_age_ms;
  // 最近生效的命令源（与 MotorMotionStateTarget 的同名常量对齐）。
  uint8_t target_source;
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
  uint8_t transaction_state; // 0=空闲，1=配置事务进行中
  uint16_t last_failure_reason;
  // 设备侧安全事实（只读遥测；与参数面 limit_protect / direction 两项同源，
  // 都是内核设备影子）。**正向语义**：enabled=true 表示保护生效，厂家
  // 原值（1 = 关）止步于协议层，不许泄漏到这一层。
  // known=false = 本会话尚未核验过该槽（回读/写后核验都没有），此时值无意义。
  bool device_protect_known;
  bool device_protect_enabled;
  bool device_direction_known;
  uint8_t device_direction; // 0 = CW，1 = CCW
  uint8_t control_lifecycle;
  uint8_t execution_state;
  bool policy_enabled;
  bool motion_enabled;
  uint32_t state_age_ms;
  uint32_t policy_generation;
} motor_control_status_t;

// 不透明上下文由库内静态分配；布局仅供 src 与白盒测试，调用方通过 API 操作。
struct motor_control_context;
typedef struct motor_control_context motor_control_context_t;

// 生产单例（库内静态分配）；白盒测试另用 motor_control_init 注入自建上下文。
motor_control_context_t *motor_control_instance(void);

// ---------------------------------------------------------------------------
// 生命周期管理
// ---------------------------------------------------------------------------

// 初始化空注册表。adapter 列表由调用方提供（生命周期覆盖调用）。
// 注意：上电默认 *未* 授权（安全默认）——任务层启动时应把 mailbox 的
// 授权电平转发进来（mailbox 引导期默认 true）。
void motor_control_init(motor_control_context_t *ctx,
    const motor_protocol_adapter_t *const *adapters,
    size_t adapter_count);

// 型号由设备身份串绑定；无身份读对象时仅允许协议内唯一型号。

// 会话边沿。断链（曾连接后断开）立即对全部可控电机发协议级停机/失能（P0）、
// 清运动授权与目标，不等待 100ms（ARD §13.2/§33）。
void motor_control_set_agent_connected(motor_control_context_t *ctx, bool connected,
    uint32_t now_ms);

// 授权门：全局授权与逐电机 MotorPolicy Enable 共同控制运动。撤销授权执行
// 与断链相同的安全路径；
// 恢复授权后必须等新的完整快照才可能回 ACTIVE（require_new_snapshot）。
// 在 P1 离开软件调度器前再次检查安全门，阻止拥塞滞留的过期目标。
bool motor_control_target_valid(const motor_control_context_t *ctx,
    uint16_t slot, uint32_t now_ms);
void motor_control_stop_all(motor_control_context_t *ctx, uint32_t now_ms);
void motor_control_soft_stop_all(motor_control_context_t *ctx, uint32_t now_ms);
void motor_control_disable_mask(motor_control_context_t *ctx, uint64_t motor_mask,
    uint32_t now_ms);
// 邮箱执行前过期/安全取消也必须向配置事务回告终态。
void motor_control_report_config_result(motor_control_context_t *ctx,
    uint32_t request_id, uint16_t motor_id, uint16_t reason, uint32_t now_ms);

bool motor_control_set_authorization(motor_control_context_t *ctx, bool authorized,
    uint32_t now_ms);
bool motor_control_is_authorized(const motor_control_context_t *ctx);

// Profile 绑定（型号目录索引版）。生产路径只有一条：建档/重连读设备自报
// 身份后由内核内部绑定（见 motor_control_start_identity_read）；本入口保留
// 给测试与目录内定向绑定。型号不在目录 → 拒绝并出 PROFILE_REJECTED 事件，
// 该电机永不出运动帧。
bool motor_control_bind_profile(motor_control_context_t *ctx, uint16_t motor_id,
    size_t profile_index, uint32_t now_ms);

// 指针版绑定（深层接口）：供测试注入自定义 Profile，或未来接入目录之外的
// 识别来源。同样受协议一致性与生命周期门禁约束。
bool motor_control_bind_profile_ptr(motor_control_context_t *ctx, uint16_t motor_id,
    const motor_protocol_model_profile_t *profile,
    uint32_t now_ms);

// P0 安全帧下游回告重入队：调度器 P0 FIFO 满或硬件拒收时，任务层
// 把内核生成的帧原样回传（tx_slot = 表项索引）。重入队复用 P0 驱逐语义；
// 待发队列全 P0 的病态下转 p0_pending 挂起（按 last_p0_kind 重新编码，
// service 逐节拍补发）——安全帧不允许在任何一段搬运中静默消失。
bool motor_control_requeue_p0(motor_control_context_t *ctx,
    const motor_protocol_tx_frame_t *frame);

// 低频配置入口（mailbox config FIFO 逐条转发）。SET_GAINS/SET_POSITION_LIMITS
// 在 Vaxor 上是本地 Policy；SET_MODE 与 CERB 限位走 request_id 事务。
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
// 5ms/200Hz 控制边界：校验并消费一份完整快照；
// 受控集合缺席电机立即软停（P0）。返回 false = 快照被拒（非法载荷）。
// ---------------------------------------------------------------------------
bool motor_control_apply_command_snapshot(motor_control_context_t *ctx,
    const motor_control_command_snapshot_t *snapshot,
    uint32_t now_ms);

// 受理一条多点轨迹（整条校验，非法即整条拒绝；空轨迹 = 取消：释放全部
// 所属电机并软停）。被新轨迹替换时，移出集合的电机立即释放。
// 受理只落锁存与归属，实际下发由 motor_control_trajectory_step 在每个
// 5ms/200Hz 控制边界完成。
bool motor_control_apply_trajectory(motor_control_context_t *ctx,
    const motor_control_trajectory_t *trajectory,
    uint32_t now_ms);

// Agent 事务入口：不抢占旧执行；逐目标授权/反馈健康均满足才原子接手。
bool motor_control_trajectory_targets_healthy(const motor_control_context_t *ctx,
    const uint16_t *target_ids, uint16_t target_count, uint32_t now_ms);
bool motor_control_apply_agent_trajectory(motor_control_context_t *ctx,
    const motor_control_trajectory_t *trajectory, uint32_t now_ms);
// 按 ID 取消，绝不影响另一条轨迹；安全帧提交后才能对外结算取消。
bool motor_control_cancel_trajectory_id(motor_control_context_t *ctx,
    uint32_t trajectory_id, uint32_t now_ms);
bool motor_control_trajectory_safety_pending(const motor_control_context_t *ctx,
    const uint16_t *target_ids, uint16_t target_count);

// 轨迹步进：按 elapsed = now_ms - 受理时刻，在完整非均匀时间线上进行单调三次
// Hermite 插值并下发。内部路点切矢由相邻不等时间间隔共同计算，保证段边界
// 速度传递；首尾端点不外推。无活动轨迹时 O(1) 返回。
// 时间超出末点 → 钳末点保持，绝不外推。
void motor_control_trajectory_step(motor_control_context_t *ctx, uint32_t now_ms);

// 每控制边界重发快照目标，维持配置的 CAN 命令率，不依赖上行快照到达率。
// 只在快照新鲜（age ≤ 30ms）时重发；挂起的 P0（软停/失能/使能重试）、整定窗、
// 轨迹所属电机一律跳过。无新证据时它是幂等的重复投递，不改变任何生命周期状态。
// 必须与 motor_control_trajectory_step 在同一拍调用（两者覆盖互斥的电机集合）。
void motor_control_hold_step(motor_control_context_t *ctx, uint32_t now_ms);

// 「与路点无关的拒绝」占位（计数越界 / 归属重复 / 未注册 / 有归属无路点）。
#define MOTOR_CONTROL_TRAJECTORY_BAD_INDEX_NONE 0xFFFFU

// 轨迹执行状态（低频发布用）。时间基准与轨迹路点同源（MCU 本地单调时基）。
typedef struct {
  uint32_t trajectory_id;
  bool active;
  bool completed;   // 时间线走完（归属仍保留：电机在末点保持）
  uint32_t elapsed_ms;
  uint32_t total_ms;
  uint16_t target_count;
  uint16_t target_ids[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
} motor_control_trajectory_state_t;

void motor_control_copy_trajectory_state(
  const motor_control_context_t *ctx,
  motor_control_trajectory_state_t *out, uint32_t now_ms);

// ---------------------------------------------------------------------------
// 内核诊断计数副本；每总线 RX 溢出与堆余量由任务层补齐。
// ---------------------------------------------------------------------------
typedef struct {
  uint32_t rx_frame_count;
  uint32_t unknown_frame_count;
  uint32_t decode_error_count;
  uint32_t snapshot_accepted;
  uint32_t snapshot_rejected;
  uint32_t config_rejected;
  uint32_t event_dropped;
  uint32_t tx_dropped;
  uint32_t p0_retry;
  uint32_t trajectory_accepted;
  uint32_t trajectory_rejected;
  uint32_t policy_accepted;
  uint32_t policy_rejected;
} motor_control_diagnostics_t;

void motor_control_copy_diagnostics(const motor_control_context_t *ctx,
    motor_control_diagnostics_t *out);

// 受理并落地一条策略快照（5ms/200Hz 控制边界调用）。逐台走与 SET_POLICY 相同的
// 落地逻辑（含防复权代数门）；**单台被拒不影响其余**，拒绝经 CONFIG_RESULT
// 事件可见。注意：逐台成功也会各出一条事件，故策略是**低频**接口——高频
// 发布会把事件环灌满（丢弃计数见 diagnostics 话题）。
// 返回 false = 整条非法（计数越界/非有限或负增益/重复 motor_id）。
bool motor_control_apply_policy(
  motor_control_context_t *ctx,
  const motor_control_policy_snapshot_t *snapshot, uint32_t now_ms);

// 发现/热插拔探测与 SYNC 心跳（P2）：对 SYNC 能力协议（CERB）的总线发
// 0x080——维持设备 20ms 周期状态上报（总线检测心跳，任务层周期调用；
// 发现窗口复用同一路径）。禁止作为运动控制边界触发（ARD §23.4）。
bool motor_control_probe_discovery(motor_control_context_t *ctx, uint8_t bus_index);

// ---------------------------------------------------------------------------
// 输出（固定容量副本；只读消费）
// ---------------------------------------------------------------------------

// 取走停机取消集合（bit = 稳定表项槽位，最多 64 台）。唯一 TX 所有者必须
// 在每次 collect_tx/下游 pump 前清除所有调度器中这些槽位的旧 P0/P1。
// 与安全帧入队分离，因此内核队满或安全帧编码失败也不会继续发送旧运动。
uint64_t motor_control_take_tx_cancellations(motor_control_context_t *ctx);

// 取走待发送帧（最多 capacity 帧，余量保留待下次）。每总线
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

// 拓扑条目（motor_id 升序；指针指向内核静态存储，调用方只在本临界区内读，
// model_name 为 NULL 表示未绑定）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  const motor_protocol_adapter_t *protocol;
  const char *model_name;
  uint32_t capability_flags;
  bool online;
  motor_control_lifecycle_t lifecycle;
} motor_control_topology_entry_t;

// 固定容量拓扑副本（最多 capacity 条，按 motor_id 升序；并发由调用方临界区）。
size_t motor_control_copy_topology(motor_control_context_t *ctx,
    motor_control_topology_entry_t *out,
    size_t capacity);


// ---------------------------------------------------------------------------
// 参数视图与存储参数套用（param_bridge / 存储任务共用）
// ---------------------------------------------------------------------------

// 单电机参数视图：值未设置（未绑定/未配置）用 NaN 表示；enable 是策略位。
// device_side_limits：true=CERB 设备对象影子；false=Vaxor 本地软限位字段。
typedef struct {
  uint16_t motor_id;
  bool online;
  bool bound;                  // Profile 已绑定（值可换算/校验）
  bool device_side_limits;
  bool enable;                 // policy_enable
  float kp;
  float kd;
  float pos_min_rad;
  float pos_max_rad;
  float velocity_limit_rad_s;
  float torque_limit_nm;
  bool limits_valid;           // 四个限值全部有效（非 NaN）
  uint8_t limits_source;       // motor_control_limits_source_t
  uint8_t dev_init;            // motor_control_dev_init_t
  // 设备端原值有效性位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*）：有位的槽位
  // 其 dev_raw 就是设备里真正存的值（回读或写后核验得来），持久化据此逐槽
  // 收录——不要求六个槽位齐全，刚写过一项也应当落盘。
  // 关节侧 N·m/A，来源 motor_protocol_profile_limit_nm_per_amp()：有规格书峰值
  // 对就优先用它，否则退 Kt × 减速比 × 效率。参数面的力矩列按 A 呈现时用它换算，
  // 未绑定/系数未知为 NaN。**反馈估计走的是 Kt 链，与它不是同一个数**——
  // R48 上分别是 3.44 与 5.05 N·m/A（11/3.2 对 0.1×101×0.5），差 1.47 倍。
  float torque_per_amp;
  uint8_t dev_raw_present;
  uint32_t dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT]; // cnt/cnt/RPM/mA/保护开关/方向
  // 固件记录原值是上电比对基准；设备回读缺失时，存储投影保留此基准。
  uint8_t record_raw_present;
  uint32_t record_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  // 设备端原值对应的关节侧换算值（与 dev_raw 同槽位索引）：DeviceParamReport
  // 的"驱动板"那一行。缺失槽位为 NaN。
  float dev_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  // 记录的关节侧换算值："固件"那一行。缺失槽位为 NaN。
  float record_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  // 已写、未保存的槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*；0 = 已保存）。
  // 含易失槽（保护开关 / 方向）——它们没有设备保存码，但记录要落 W25 才生效。
  uint8_t record_dirty_slots;
  // 记录与设备**对不上**的槽位位图（口径同 DEV_INIT 事件的 detail）。
  // dev_init == MISMATCH 时它点名是哪几项；其余态为 0。
  uint8_t dev_mismatch_mask;
} motor_control_param_view_t;

// 六个设备端原值是否齐全（页面/工具判"设备真值可完整显示"）。
#define MOTOR_CONTROL_DEV_RAW_ALL \
  ((uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_COUNT) - 1U))

// 只读参数视图副本（固定容量，按注册表顺序）；未发现电机不在结果中。
size_t motor_control_copy_param_views(motor_control_context_t *ctx,
    motor_control_param_view_t *out,
    size_t capacity);

// boot 持久化参数（W25 blob 投影）；未保存字段对应 has_* 为 false。
// 不含 enable（策略位不持久化）。调用方（motor_rt 5ms/200Hz 边界）每边界传入，
// 内核按表项 stored_apply_mask 在 Profile 绑定后分批套用。
typedef struct {
  uint16_t motor_id;
  bool has_kp_kd;
  bool has_position_limits;
  bool has_velocity_limit;
  bool has_torque_limit;
  float kp;
  float kd;
  float pos_min_rad;
  float pos_max_rad;
  float velocity_limit_rad_s;
  float torque_limit_nm;
  // 写入设备时实际落在设备里的原值（与上列 SI 值同一次事务产生）：
  // 上电比对比的是这一组，设备端原值不随换算系数漂移。
  uint32_t device_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  uint8_t device_raw_present; // 位图 = 1u << slot
} motor_control_stored_params_t;

// 套用 boot 存储参数（幂等：套尽的电机 mask 清零；Profile 未绑定的电机
// 保留 mask 等后续边界）。CERB 设备值只填影子，不发任何 SDO 写。
void motor_control_apply_stored_params(
  motor_control_context_t *ctx,
  const motor_control_stored_params_t *records, size_t count,
  uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
