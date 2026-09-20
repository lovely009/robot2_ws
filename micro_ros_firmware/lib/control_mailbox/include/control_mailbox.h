// 固定容量的跨任务控制输入交换层。
// 不依赖 HAL、RTOS、Arduino、ROS 或任何电机协议。
//
// 并发契约：本层无内部锁、无原子字段。submit/take/clear/request_*/
// take_events 的每一次调用都必须位于调用方提供的临界区内，且写侧与
// 读侧必须共用同一互斥域（单核下 FreeRTOS taskENTER_CRITICAL 或等效
// 关中断即满足；"两侧各自加锁"不构成互斥，会产生数据竞争）。仅支持
// 任务上下文调用（ISR 内 taskENTER_CRITICAL 非法，本层不做检测）；
// 违反契约（如无锁双写者）不属于受支持用法。

#ifndef CONTROL_MAILBOX_H
#define CONTROL_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "control_command_mailbox.h"
#include "control_policy_mailbox.h"
#include "control_trajectory_mailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY 16U

// 低频设备配置的语义类型。具体型号/协议能力由运行时和协议适配器判断。
typedef enum {
  CONTROL_MAILBOX_CONFIG_SET_MODE = 1U,
  CONTROL_MAILBOX_CONFIG_SET_GAINS = 2U,
  CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS = 3U,
  CONTROL_MAILBOX_CONFIG_SET_ENABLE = 4U, // 逐电机 Enable（ARD §19 MotorPolicy）
  // 设备侧管理对象（仅 CERB，走 CANopen 快速 SDO）：关节侧 SI 量，内核钳制到
  // 型号额定后再由 adapter 换算成电机端原始量（0x6080 RPM / 0x2538 mA）。
  CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT = 5U,
  CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT = 6U,
  // 设备参数回读（无载荷，只用 motor_id；CERB 走 4 个加速 SDO upload，
  // Vaxor 内核直接回 UNSUPPORTED）。
  CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS = 8U,
  // 设零点（无载荷，只用 motor_id；CERB 写 0x2531=1，仅失能态受理）。
  CONTROL_MAILBOX_CONFIG_SET_ZERO = 9U,
  // 故障清除（无载荷，只用 motor_id）：清可清的故障。CERB 发 MIT 控制字
  // bit5；Vaxor 无清错手段 → 内核回 UNSUPPORTED。身份冲突锁存刻意不可
  // 解除（ARD §9.3），内核拒绝。
  CONTROL_MAILBOX_CONFIG_CLEAR_FAULT = 10U,
  // 逐电机策略（无设备事务，本地立即生效）：enable + 增益原子更新，另带
  // 防复权代数。**单台入口**——ROS 面走的是批量通道
  // （control_policy_mailbox.h），批量落地逐台复用同一套逻辑。
  CONTROL_MAILBOX_CONFIG_SET_POLICY = 11U,
  // 设备侧看门狗/限位保护开关（仅 CERB，写 0x2650:00）：载荷是**厂家原值**
  // 0 = 保护开 / 1 = 关。仅未运动时受理，写后读回核验，不跟保存帧。
  CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT = 12U,
  // 设备侧电机方向定义（仅 CERB，写 0x2541:00）：载荷是**原值** 0 = CW /
  // 1 = CCW。仅未运动时受理，写后读回核验，不跟保存帧（易失）。
  CONTROL_MAILBOX_CONFIG_SET_DIRECTION = 13U,
  // 设备侧保存（仅 CERB，写 0x2539 保存码）：把设备 RAM 里的值落到驱动器
  // flash。**与写分开**——写只改设备 RAM + 固件记录，保存才落盘；两次动作
  // 之间该台处于「未保存」（DeviceParamReport.record_dirty）。
  // 载荷 = 待保存槽位位图，非空即可；哪些槽真有厂家保存码由内核按掩码决定。
  CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH = 14U
} control_mailbox_config_kind_t;

typedef struct {
  float kp;
  float kd;
} control_mailbox_gains_t;

typedef struct {
  float min_position_rad;
  float max_position_rad;
} control_mailbox_position_limits_t;

// SET_POLICY 载荷：三项一起送，内核一次落地。
// generation 必须严格前进才受理「未武装 → 武装」的迁移；内核据此挡住
// 「周期重发同一条策略」在故障后把电机自动重新武装起来。
typedef struct {
  uint32_t generation;
  bool enable;
  float kp;
  float kd;
  // 保护确知为关时的显式武装确认（每次都要）。
  bool confirm_unprotected;
} control_mailbox_policy_t;

typedef struct {
  uint32_t request_id;
  uint32_t received_tick_ms;
  uint16_t motor_id;
  control_mailbox_config_kind_t kind;
  union {
    uint8_t mode;
    bool enable;
    // SET_ENABLE：该台保护确知为关时的显式武装确认（每次都要）。与 SET_POLICY
    // 载荷里的同名字段同义——两条使能入口共用内核同一道门禁。
    bool confirm_unprotected;
    control_mailbox_gains_t gains;
    control_mailbox_position_limits_t position_limits;
    float max_velocity_rad_s; // SET_VELOCITY_LIMIT（关节侧 rad/s）
    float max_torque_nm;      // SET_TORQUE_LIMIT（关节侧 N·m）
    // SET_SAFETY_PROTECT：厂家原值 0/1（0 = 保护开，1 = 关）。收原值不收
    // 布尔——方向语义的反转只允许发生在参数面一处。
    uint32_t safety_protect_raw;
    // SET_DIRECTION：厂家/设备原值 0/1（0 = CW，1 = CCW），收原值不收布尔，
    // 与 SET_SAFETY_PROTECT 同口径——语义反转只允许发生在参数面一处。
    uint32_t direction_raw;
    // SAVE_DEVICE_FLASH：待保存槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*）。
    uint8_t save_slots;
    control_mailbox_policy_t policy; // SET_POLICY
  } value;
} control_mailbox_config_request_t;

typedef struct {
  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_edge;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_edge;
} control_mailbox_events_t;

typedef struct {
  control_command_mailbox_t command;
  control_trajectory_mailbox_t trajectory;
  control_policy_mailbox_t policy;
  control_mailbox_config_request_t
      config_queue[CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY];
  uint8_t config_head;
  uint8_t config_count;

  // 拒绝诊断（拒绝必可见，且类别可区分；为生命周期累计，clear 不清零）。
  uint32_t config_invalid_count;   // 请求非法（request_id==0/字段非法/未知 kind）
  uint32_t config_overflow_count;  // 配置 FIFO 满（消费慢于生产）
  uint32_t config_session_count;   // Agent 未连接期的配置提交
  uint32_t command_rejected_count; // 被授权/会话门拒绝的命令提交
                                   //（快照本身非法走同一计数，已导出）
  uint32_t trajectory_rejected_count; // 被授权/会话门拒绝的轨迹提交
                                      //（轨迹本身非法走 trajectory.invalid_trajectory_count）
  uint32_t policy_rejected_count; // 被授权/会话门拒绝的策略提交
                                  //（策略本身非法走 policy.invalid_policy_count）

  bool authorization_requested;     // 授权电平（init 默认 true = 引导豁免）
  bool authorization_pending;       // 电平变化边沿（一次性，take_events 读后清）
  bool authorization_disable_edge;  // 授权撤销边沿（一次性）
  bool agent_state_seen;            // 是否收到过首次 Agent 状态
  bool agent_connected_requested;   // Agent 连接电平（未见状态前视为连接）
  bool agent_state_pending;         // 连接电平变化边沿（一次性）
  bool agent_disconnect_edge;       // 连接后断开边沿（一次性）
} control_mailbox_t;

void control_mailbox_init(control_mailbox_t *mailbox);

// 高频输入：完整快照 latest-wins。未授权或 Agent 已断开时拒绝新命令
// （计入 command_rejected_count，已由诊断话题导出；快照本身非法同样走这里，
// 计数，二者不重复）。
bool control_mailbox_submit_command(
    control_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot);
bool control_mailbox_take_command(control_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot);
// 只撤销"未取走"的输入；诊断计数保留为生命周期统计。
void control_mailbox_clear_command(control_mailbox_t *mailbox);

// 轨迹输入：多点时间线 latest-wins。与 submit_command 同门同序（授权 +
// 会话门先于数据校验）；空轨迹 = 取消，是合法载荷。
bool control_mailbox_submit_trajectory(control_mailbox_t *mailbox,
                                       const control_trajectory_t *trajectory);
bool control_mailbox_take_trajectory(control_mailbox_t *mailbox,
                                     control_trajectory_t *trajectory);
void control_mailbox_clear_trajectory(control_mailbox_t *mailbox);

// 策略输入：批量、latest-wins。与 submit_command 同门同序（授权 + 会话门
// 先于数据校验）。**不走 config FIFO**——那条队列深 16，装不下一条最多 64
// 台的策略消息（详见 control_policy_mailbox.h）。
bool control_mailbox_submit_policy(control_mailbox_t *mailbox,
                                   const control_policy_snapshot_t *snapshot);
bool control_mailbox_take_policy(control_mailbox_t *mailbox,
                                 control_policy_snapshot_t *snapshot);
void control_mailbox_clear_policy(control_mailbox_t *mailbox);

// 低频输入：有界 FIFO，不覆盖、不丢弃已经入队的旧配置请求。
// 配置不属于运动授权：可在禁能/未授权状态下排队（会话门仍生效，
// 且会话门先于数据校验——与 submit_command 同序，断链期拒绝统一
// 计入 config_session_count）。
bool control_mailbox_submit_config(
    control_mailbox_t *mailbox,
    const control_mailbox_config_request_t *request);
bool control_mailbox_take_config(
    control_mailbox_t *mailbox,
    control_mailbox_config_request_t *request);
void control_mailbox_clear_config(control_mailbox_t *mailbox);

// 连接/授权是状态请求：同状态重复调用幂等（不产生新边沿）；
// 断链（曾连接后断开）清命令+配置并撤销授权；会话恢复不恢复授权
// ——恢复必须显式 request_authorization(true)（ARD §33）。
void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized);
void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                             bool connected);
// 返回值 = 自上次调用以来是否出现未读边沿；返回 false 时仍带货当前电平
// ——任务层启动时应先调用一次以同步初始电平。
// 会话电平（授权派生依据）：从未上报过连接状态（引导期，状态未知）或
// 明确在线 = true；曾连接后断开 = false。
bool control_mailbox_session_authorized(const control_mailbox_t *mailbox);

bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MAILBOX_H
