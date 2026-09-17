#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ID 驱动的电机运行时内核（ARD-0007 §8/§9/§13/§20）。
//
// 定位：纯 C 运行时内核——注册表、型号 Profile 目录、生命周期、安全、
// freshness、状态/事件快照。不依赖 ROS、HAL、FreeRTOS、control_mailbox，
// 也不感知 joint_name（关节语义在 upper 层，AR-003）。
// 厂家字节只经 motor_protocol 的 adapter 接口（构造时注入 adapter 列表，
// 新增协议不改内核——ARD §24.3）。
//
// 所有权：motor_rt_task 是唯一调用者。1ms 服务节拍调
// on_raw_frame()/service()；20ms 控制边界调 apply_command_snapshot()；
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
// 事件环深度 = ROS 桥一个轮询间隔内可积压的事件数（64）。
#define MOTOR_CONTROL_MAX_EVENTS 64U

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

// 统一故障码（厂家原始故障字另行透传，映射在 adapter——ARD §24）。
typedef enum {
  MOTOR_CONTROL_FAULT_NONE = 0U,
  MOTOR_CONTROL_FAULT_PROTOCOL_ERROR = 1U,
  MOTOR_CONTROL_FAULT_VENDOR = 2U,
  MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE = 3U,
  MOTOR_CONTROL_FAULT_CONFIG_CONFLICT = 4U,
  MOTOR_CONTROL_FAULT_OFFLINE = 5U,
  // 设备参数未初始化（上电回读失败或与固件记录不一致）。仅设备侧限位
  // 型号（CERB）会产生：参数以设备为准，未核对不得运动。
  MOTOR_CONTROL_FAULT_DEV_PARAM = 6U
} motor_control_fault_code_t;

// 设备参数初始化状态（上电/重连后回读设备 → 与固件记录比对）。
// 参数以设备内为准、固件记录只作比对基准：不一致即"未初始化"，该台不可控。
typedef enum {
  MOTOR_CONTROL_DEV_INIT_NA = 0U,        // 非设备侧型号（VAXOR）：无从比对
  MOTOR_CONTROL_DEV_INIT_PENDING = 1U,   // 尚未回读设备（上电/重连后）
  MOTOR_CONTROL_DEV_INIT_OK = 2U,        // 与记录一致（或本机无记录可比）
  MOTOR_CONTROL_DEV_INIT_MISMATCH = 3U,  // 与记录不一致 → 未初始化
  MOTOR_CONTROL_DEV_INIT_UNVERIFIED = 4U // 回读失败（超时/中止）→ 未核对
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
  MOTOR_CONTROL_EVENT_DEV_INIT = 15U
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
  uint32_t state_age_ms; // 从无测量 = 0xFFFFFFFF
  uint8_t transaction_state; // 0=空闲，1=配置事务进行中
  uint16_t last_failure_reason;
} motor_control_status_t;

// ---------------------------------------------------------------------------
// 运行时上下文（静态分配；字段仅供内核实现读写，调用方走函数接口）。
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 不透明运行时上下文：表项/上下文字段布局对库内私有
// （见 src/motor_control_internal.h，PlatformIO 不向 app 导出库内 src）。
// app 只能持有指针经本头 API 操作，越界访问字段即编译失败。
// ---------------------------------------------------------------------------
struct motor_control_context;
typedef struct motor_control_context motor_control_context_t;

// 生产单例（库内静态分配）；白盒测试另用 motor_control_init 注入自建上下文。
motor_control_context_t *motor_control_instance(void);

// 取走并清除记录脏位（true = 自上次取走后有设备侧写成功，需要落盘）。
bool motor_control_take_record_dirty(motor_control_context_t *ctx);

// ---------------------------------------------------------------------------
// 生命周期管理
// ---------------------------------------------------------------------------

// 初始化空注册表。adapter 列表由调用方提供（生命周期覆盖调用）。
// 注意：上电默认 *未* 授权（安全默认）——任务层启动时应把 mailbox 的
// 授权电平转发进来（mailbox 引导期默认 true）。
void motor_control_init(motor_control_context_t *ctx,
                        const motor_protocol_adapter_t *const *adapters,
                        size_t adapter_count);

// 静态拓扑注入已删除：型号绑定改为建档/重连时读设备自报身份（协议 adapter
// 的 READ_IDENTITY 事务），无身份读对象的协议回退到"协议内唯一型号"。

// 会话边沿。断链（曾连接后断开）立即对全部可控电机发协议级停机/失能（P0）、
// 清运动授权与目标，不等待 100ms（ARD §13.2/§33）。
void motor_control_set_agent_connected(motor_control_context_t *ctx, bool connected,
                                       uint32_t now_ms);

// 授权门：全局授权与逐电机 MotorPolicy Enable 共同控制运动。撤销授权执行
// 与断链相同的安全路径；
// 恢复授权后必须等新的完整快照才可能回 ACTIVE（require_new_snapshot）。
bool motor_control_set_authorization(motor_control_context_t *ctx, bool authorized,
                                     uint32_t now_ms);

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
// 20ms 控制边界（ARD §11.3）：校验并消费一份完整快照；
// 受控集合缺席电机立即软停（P0）。返回 false = 快照被拒（非法载荷）。
// ---------------------------------------------------------------------------
bool motor_control_apply_command_snapshot(motor_control_context_t *ctx,
                                          const motor_control_command_snapshot_t *snapshot,
                                          uint32_t now_ms);

// 发现/热插拔探测与 SYNC 心跳（P2）：对 SYNC 能力协议（CERB）的总线发
// 0x080——维持设备 20ms 周期状态上报（总线检测心跳，任务层周期调用；
// 发现窗口复用同一路径）。禁止作为 50Hz 控制触发（ARD §23.4）。
bool motor_control_probe_discovery(motor_control_context_t *ctx, uint8_t bus_index);

// ---------------------------------------------------------------------------
// 输出（固定容量副本；只读消费）
// ---------------------------------------------------------------------------

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
  // 收录——不要求四个槽位齐全，刚写过一项也应当落盘。
  // 关节侧 N·m/A（Kt × 减速比 × 效率）：参数服务器的力矩列按 A 呈现时要它
  // 换算，未绑定/系数未知为 NaN。设备侧型号与反馈换算用的是同一个系数。
  float torque_per_amp;
  uint8_t dev_raw_present;
  uint32_t dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT]; // cnt/cnt/RPM/mA
  // 固件记录里的设备端原值（上电比对基准）。设备端原值缺失时（本次上电还没
  // 成功回读过设备）存储投影用它保底——否则一次"读不到设备"后再点保存，
  // 会把比对基准整体抹掉，下次上电就再也发现不了不一致。
  uint8_t record_raw_present;
  uint32_t record_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
} motor_control_param_view_t;

// 四个设备端原值是否齐全（页面/工具判"设备真值可完整显示"）。
#define MOTOR_CONTROL_DEV_RAW_ALL \
  ((uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_COUNT) - 1U))

// 只读参数视图副本（固定容量，按注册表顺序）；未发现电机不在结果中。
size_t motor_control_copy_param_views(motor_control_context_t *ctx,
                                      motor_control_param_view_t *out,
                                      size_t capacity);

// boot 持久化参数（W25 blob 投影）；未保存字段对应 has_* 为 false。
// 不含 enable（策略位不持久化）。调用方（motor_rt 20ms 边界）每边界传入，
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
