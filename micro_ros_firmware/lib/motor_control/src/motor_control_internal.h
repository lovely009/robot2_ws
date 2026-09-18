// motor_control 内部共享接口（仅本库 src/ 与白盒测试经 -I 可见，不下沉公共 API）。

#ifndef MOTOR_CONTROL_INTERNAL_H
#define MOTOR_CONTROL_INTERNAL_H

#include "motor_control.h"
#include "motor_control_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONTROL_IDENTITY_CAPACITY 24U

typedef struct {
  bool used;
  uint16_t motor_id; // 全局唯一（当前派生：= 厂家 node 号；冲突即 CONFIG_CONFLICT）
  uint8_t bus_index;
  uint16_t node;
  const motor_protocol_adapter_t *adapter;
  const motor_protocol_model_profile_t *profile; // NULL = 未绑定（不可控）
  // 身份读取（型号绑定的唯一识别来源）：建档/重连各发起一次，读毕置
  // identity_read_done（成败皆置，不逐帧重试——识别失败是设备事实，
  // 重试只会刷事件环）。protocol 无身份读对象时内核直接走"协议内唯一型号"
  // 兜底，同样置 done。
  uint8_t identity[MOTOR_CONTROL_IDENTITY_CAPACITY];
  size_t identity_size;
  bool identity_read_overflow; // 回串超出容量 → 读取失败（不用残缺串绑定）
  bool identity_read_done;
  motor_control_lifecycle_t lifecycle;
  bool online;

  uint32_t last_rx_tick_ms;       // 任意本机帧（OFFLINE 判定）
  uint32_t last_motion_rx_tick_ms; // 最近有效运动反馈（state_age）
  uint32_t last_commanded_tick_ms; // 最近 MIT/使能/配置帧（非自主心跳协议的
                                   // OFFLINE 判定输入）

  bool has_target;
  float target_position_rad;
  float target_velocity_rad_s;
  float target_effort_nm;

  // Policy（低频，不进入 100 Hz 命令——ARD §19.4）。
  bool policy_enable;   // 逐电机 Enable（MotorPolicy；OFFLINE/厂家故障/冲突
                        // 冻结为 false，恢复需上层显式重发 SET_ENABLE(true)
                        // ——ARD §36）
  bool has_kp_kd;
  float kp;
  float kd;
  bool has_soft_limits; // 仅 Vaxor（软件限位，打包前钳制——ARD §26.1）
  float soft_min_rad;
  float soft_max_rad;
  // 软件速度/力矩限值：同样仅对 CAP_LIMIT_SOFTWARE 型号本地生效（VAXOR 无
  // 厂家管理对象）。钳制发生在打包前，只影响目标、不触发驱动器保护，故与
  // 增益一样无生命周期门禁（设备侧写才有门禁，见 motor_control_config.c）。
  bool has_soft_vel_limit;
  float soft_max_velocity_rad_s; // 双向对称限值（±）
  bool has_soft_torque_limit;
  float soft_max_torque_nm;      // 双向对称限值（±）

  // 使能时序（vendor 整定门：adapter->enable_settle_ms）。
  bool motion_enabled;    // 已发使能帧且未被失能
  bool enable_pending;    // 整定窗口内，暂不发运动目标
  uint32_t enable_started_tick_ms;
  // 逐电机运动授权（ARD §36/§10.4）：故障、离线、撤权或 SET_ENABLE(false)
  // 会清除此位，恢复必须显式 SET_ENABLE(true)，不得由新快照自动复立。
  bool motion_authorized;

  // 最近一次受理的 SET_POLICY 代数。「未武装 → 武装」要求代数严格前进，
  // 用于挡住「周期重发同一条策略」在故障后把电机自动重新武装。
  // 不需要在故障/离线/冲突/设备参数路径上单独锁存：那几条路径都会把
  // policy_enable 清成 false，本规则靠「当前未武装」自然生效。
  uint32_t policy_generation;

  // P0 帧重试：推送失败（缓冲满且无可驱逐的 P1/P2）时置位，service 逐节拍重发；
  // 新 MIT 帧成功后作废（新命令取代过期的安全帧）。
  bool p0_pending;
  uint8_t p0_kind; // 0=无，1=软停，2=失能，3=使能（见 motor_control_internal.h）
  uint8_t last_p0_kind; // 最近生成的 P0 帧类别：下游重入队失败转挂起的依据

  motor_protocol_measured_state_t measured;
  bool has_measured;

  // 诊断（ARD §8.1/§32.2）。
  uint16_t unified_fault_code;
  uint32_t vendor_fault_code;
  uint32_t protection_flags;
  uint32_t limit_flags;
  uint32_t protocol_error_count;
  uint16_t last_failure_reason;

  // 设备侧参数影子（CERB）：当前生效的四个限值 + 来源。写事务核验值、
  // REFRESH 回读值、Flash 记录、型号标定都往这里落，limits_source 标明是
  // 哪一种——页面必须能分辨"设备真值"与"离线回落记录"。Vaxor 不用（其限值
  // 是本地 soft_* 字段，来源恒为 NONE/PROFILE）。
  bool dev_limits_valid;
  uint8_t limits_source; // motor_control_limits_source_t
  float dev_min_rad;
  float dev_max_rad;
  float dev_max_velocity_rad_s;
  float dev_max_torque_nm;
  // 设备端原值（cnt/cnt/RPM/mA，按 MOTOR_PROTOCOL_PARAM_SLOT_* 索引）：
  // 与固件记录比对的唯一口径，也让页面显示设备里真正存的东西（换算值会
  // 把系数错误一起吞掉，见 motor_protocol_adapter.h 的 param_* 契约）。
  uint32_t dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  float dev_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT]; // 同槽位的关节侧换算值
  uint8_t dev_raw_present; // 位图 = 1u << slot
  // REFRESH_PARAMS 事务回读暂存：adapter 按阶段追加一个
  // motor_protocol_param_stage_t（原值 + 关节侧换算值）。
  motor_protocol_param_stage_t refresh_stages[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  uint8_t refresh_count;
  // 本次回读已落袋的槽位位图。成功收尾才提交到 dev_raw_present——半途失败
  // （超时/中止）不得让位图声称某些槽位有新值，否则比对会拿半份数据下结论。
  uint8_t refresh_slots;
  // 固件记录（Flash 投影）：上电比对的基准，也是电机未上线时页面的回落值。
  bool record_valid;
  uint32_t record_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  uint8_t record_raw_present;
  bool boot_record_offered; // 本会话已把 Flash 记录（或"无记录"）告知本表项
  // 上电比对结论。PENDING/MISMATCH/UNVERIFIED 三态都不可控（见
  // motor_control_dev_params_ready）。
  motor_control_dev_init_t dev_init;
  bool dev_read_done; // 本会话已对设备参数定论（成功后不再自动重读）
  // 自动回读的尝试计数与退避时刻：SDO 读是纯只读、对设备无副作用，读不到
  // 多半是总线抖动/设备忙，多给几次机会再判"未核对"，避免一次抖动就把该台
  // 卡在不可使能（结论口径不变：没核对通过就不放行）。
  uint8_t dev_read_attempts;
  uint32_t dev_read_retry_tick_ms;
  // boot 存储参数套用状态：bit7=已见过存储记录（只初始化一次），
  // bit0=待套增益、bit1=待套限值；Profile 未就绪时低位保留由 20ms 边界
  // 重试，套尽后只留 bit7（之后的 refresh/设备写影子不被重复 apply 覆盖）。
  uint8_t stored_apply_mask;

  // 配置事务（每电机至多一个未决；新请求 BUSY——ARD §20.3）。
  bool tx_active;
  bool tx_awaiting;
  uint32_t tx_request_id;
  motor_protocol_config_kind_t tx_kind;
  motor_protocol_config_request_t tx_request; // 事务全程持有请求副本
  uint8_t tx_stage;
  uint8_t tx_retries;
  uint32_t tx_deadline_tick_ms;
  // 本事务各阶段实际编码出的设备端原值（写阶段发出时登记）。写事务成功
  // 收尾时用它同步固件记录——记录必须与设备里真正存的原值同口径。
  uint32_t tx_write_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
} motor_control_entry_t;

struct motor_control_context {
  // adapter 注册表（构造注入；测试注入 mock）。
  const motor_protocol_adapter_t *const *adapters;
  size_t adapter_count;

  // 表项索引被 TX 桥引用为 P1 latest-wins 槽位，并被 P0 回告重入队用于定位
  // 表项（tx_slot，见 motor_protocol_adapter.h）：只增不减是其稳定性前提——
  // 禁止实现表项压缩/回收，否则必须同步重定义槽位映射。
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

  // 多点轨迹（ARD §13 的时序补充通道；与快照通道并存）。归属不落字段：
  // 「某电机是否被轨迹拥有」由 (trajectory_active, target_ids, motor_id)
  // 纯派生——故障/离线/冲突/设备参数未核对各自把表项带离可动生命周期，
  // 自动脱离轨迹，无需在那些路径里额外清理归属。
  motor_control_trajectory_t trajectory;
  bool trajectory_active;
  uint32_t trajectory_start_tick_ms; // 起点 = 受理时刻的本地单调时基
  uint32_t trajectory_accepted_count;
  uint32_t trajectory_rejected_count;
  uint32_t policy_accepted_count;
  uint32_t policy_rejected_count;

  // 待发送帧缓冲（collect_tx 消费；满则丢弃并计数）。
  motor_protocol_tx_frame_t tx_pending[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t tx_count;
  uint32_t tx_dropped_count;
  uint32_t p0_retry_count; // P0 帧因缓冲全满而挂起重试的次数（诊断可见）

  // 事件环。
  motor_control_event_t events[MOTOR_CONTROL_MAX_EVENTS];
  size_t event_count;
  uint32_t event_dropped_count;
  // 事件序号分配器：每 push 一条 +1（回绕安全，消费者只比相邻差值）。
  uint32_t event_seq;

  // 诊断计数（ARD §32）。
  uint32_t rx_frame_count;
  uint32_t unknown_frame_count;
  uint32_t decode_error_count;
  uint32_t snapshot_accepted_count;
  uint32_t snapshot_rejected_count;
  uint32_t config_rejected_count;

  // "设备与固件记录一起写"的落盘信号：设备侧写事务成功（设备 + 记录已同步）
  // 后置位。任务层取走并落 Flash——记录不落盘就断电，下次上电比对必然不一致。
  bool record_dirty;
};

// 表项索引（tx_slot 的取值来源）：依赖"表项只增不减"不变量（见
// context entries 注释）。P1 latest-wins 槽位与 P0 回告重入队
// 均按此索引定位表项。
static inline size_t motor_control_entry_index(
    const motor_control_context_t *ctx, const motor_control_entry_t *entry) {
  return (size_t)(entry - ctx->entries);
}

// 设备参数初始化是否放行运动（"初始化完成前不可使能"）。只有 NA/OK 放行：
// PENDING/MISMATCH/UNVERIFIED 一律不放行——没核对完成就没有"用哪份值"的答案。
static inline bool motor_control_dev_params_allow_motion(
    const motor_control_entry_t *entry) {
  return entry != NULL && (entry->dev_init == MOTOR_CONTROL_DEV_INIT_NA ||
                           entry->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}

// 顺序确定化选择器：取下一个「未输出且 motor_id 最小」表项并在 taken
// 标记；无可取返回 NULL（copy_motion_states/statuses/topology 共用）。
const motor_control_entry_t *motor_control_entry_take_sorted(
    const motor_control_context_t *ctx, bool *taken);

// P0 帧重试挂起类别（motor_control_entry_t::p0_kind）。
#define MOTOR_CONTROL_P0_NONE 0U
#define MOTOR_CONTROL_P0_SOFT_STOP 1U
#define MOTOR_CONTROL_P0_DISABLE 2U
#define MOTOR_CONTROL_P0_ENABLE 3U // 使能帧亦为 P0

// 追加事件（环满丢最旧并计数——保最新状态，ARD §18 精神；tick 为 MCU 单调时基）。
void motor_control_push_event(motor_control_context_t *ctx, uint32_t now_ms,
                              motor_control_event_type_t type, uint16_t motor_id,
                              uint8_t bus_index, uint16_t reason, uint32_t request_id);

// 追加待发送帧（满则丢弃并计数，不阻塞实时路径）。
bool motor_control_push_tx(motor_control_context_t *ctx,
                           const motor_protocol_tx_frame_t *frame);

// P0 安全帧推送：满时驱逐最旧的非 P0 帧腾位（对齐 ARD §22.1 抢占语义）；
// 仍失败则按 entry->last_p0_kind 挂 p0_pending，由 service 逐节拍重发
// （不静默丢安全帧）。调用方推送前必须已设置 last_p0_kind。
bool motor_control_push_tx_p0(motor_control_context_t *ctx,
                              motor_control_entry_t *entry,
                              const motor_protocol_tx_frame_t *frame);

// 统一 P0 安全帧生成 + 推送（kind 取 MOTOR_CONTROL_P0_DISABLE /
// MOTOR_CONTROL_P0_SOFT_STOP）。编码失败与队列满都不静默丢：挂起重试
// 并计数。撤权/离线/故障/冲突/软停五条路径共用，勿再各自复制编码逻辑。
bool motor_control_send_p0(motor_control_context_t *ctx,
                           motor_control_entry_t *entry, uint8_t kind);

// 全局键查找（线性扫描：40 台规模 + 64 容量下足够，确定性优于哈希）。
motor_control_entry_t *motor_control_find_entry(motor_control_context_t *ctx,
                                                uint16_t motor_id);

// 撤销运动授权的统一安全路径（断链 / 撤权 / 100ms 超时共用）：
// 对所有已使能电机发协议级停机/失能（P0），清目标与使能状态，
// 受控态降级 READY_DISABLED，置 require_new_snapshot（ARD §13.2/§13.3/§33）。
void motor_control_revoke_motion(motor_control_context_t *ctx, uint32_t now_ms);

// 单电机软停（P0 阻尼帧 + SAFE_STOP）。仅对 ACTIVE 态发帧（去重）。
void motor_control_soft_stop_entry(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry, uint32_t now_ms);

// 绑定后首次设备影子填 Profile 默认（CERB；motor_control_params.c）。
void motor_control_set_default_device_shadow(motor_control_entry_t *entry);

// 策略落地（motor_control_config.c）：SET_POLICY config kind 与批量入口
// motor_control_apply_policy 共用，避免两条路径的门禁与钳制漂移。
// 返回值 = 是否受理；被拒时会自行出 CONFIG_RESULT 事件。
bool motor_control_apply_policy_entry(motor_control_context_t *ctx,
                                      motor_control_entry_t *entry,
                                      const motor_protocol_config_request_t *request,
                                      uint32_t now_ms);

// 配置事务：响应帧裁决（on_raw_frame 调用）与跨节拍步进（service 调用）。
bool motor_control_config_on_frame(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry,
                                   const motor_protocol_raw_frame_t *frame);
void motor_control_config_step(motor_control_context_t *ctx, uint32_t now_ms);

// 身份读取发起（config_step 的不变量复检调用，每拍至多一台）：走事务机器
// 读设备自报型号串，读毕由内核按串精确匹配型号目录并绑定；协议无身份读
// 对象时直接用"协议内唯一型号"兜底。已在读（tx_active）或本轮已定论
// （identity_read_done）的表项直接返回。
void motor_control_start_identity_read(motor_control_context_t *ctx,
                                       motor_control_entry_t *entry,
                                       uint32_t now_ms);

// 配置事务统一收尾（CONFIG_RESULT 终态回告，清理 tx_active/tx_awaiting）。
// 只在事务自己走出裁决（响应、超时、阶段推进完毕）时调用。
void motor_control_finish_tx(motor_control_context_t *ctx,
                             motor_control_entry_t *entry, uint16_t result,
                             uint32_t now_ms);

// 事务中止（离线/冲突/厂家故障/撤权路径共用）：配置事务回 UNCONFIRMED 终态
// 避免上层请求悬挂；身份读取不判成败也不回事件，待表项回到可绑定态重发。
void motor_control_abort_tx(motor_control_context_t *ctx,
                            motor_control_entry_t *entry, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_INTERNAL_H
