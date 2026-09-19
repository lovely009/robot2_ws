#ifndef MOTOR_PROTOCOL_ADAPTER_H
#define MOTOR_PROTOCOL_ADAPTER_H

// 协议适配层（ARD-0007 §24）：厂家字节布局、CAN-ID 推导、帧长与 FD/BRS 属性
// 全部封闭在 lib/motor_protocol 内。motor_control 内核只通过本头文件的
// adapter 接口访问厂家协议，新增协议家族只新增一个 adapter 实例，
// 不改内核收发循环（§24.3）。
//
// 依赖边界：本层只依赖 motor_protocol.h 的纯编解码函数，
// 不依赖 HAL / FreeRTOS / ROS / control_mailbox / motor_control。

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// 公共故障/限位位标志（ARD-0007 §17.4：唯一公共定义点，业务模块不得重新编号；
// 具体位分配在 TBD-STATUS-01 关闭前为临时占位，跨协议语义保持稳定）。
// ---------------------------------------------------------------------------
#define MOTOR_PROTOCOL_PROT_VENDOR_FAULT (1UL << 0)    // 厂家故障字非零/故障位
#define MOTOR_PROTOCOL_PROT_OVERTEMPERATURE (1UL << 1) // 暂无温度来源，预留
#define MOTOR_PROTOCOL_PROT_OVERCURRENT (1UL << 2)     // 预留
#define MOTOR_PROTOCOL_PROT_UNDERVOLTAGE (1UL << 3)    // 预留
#define MOTOR_PROTOCOL_PROT_DECODE_ERROR (1UL << 4)    // 帧解码失败

#define MOTOR_PROTOCOL_LIMIT_POS_MIN (1UL << 0) // 软件限位下限钳制
#define MOTOR_PROTOCOL_LIMIT_POS_MAX (1UL << 1) // 软件限位上限钳制
#define MOTOR_PROTOCOL_LIMIT_VEL (1UL << 2)     // 速度钳制（预留）
#define MOTOR_PROTOCOL_LIMIT_EFFORT (1UL << 3)  // 扭矩超峰值钳制

// 能力标志（Profile 与 adapter 声明共同消费）。
#define MOTOR_PROTOCOL_CAP_MODE_MIT (1UL << 0)
#define MOTOR_PROTOCOL_CAP_MODE_VELOCITY (1UL << 1)
#define MOTOR_PROTOCOL_CAP_MODE_POSITION (1UL << 2)
#define MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE (1UL << 3)  // CERB 驱动器侧限位
#define MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE (1UL << 4)     // Vaxor 仅软件限位
#define MOTOR_PROTOCOL_CAP_AUTONOMOUS_FEEDBACK (1UL << 5) // vaxor ~20ms 自主上报

// ---------------------------------------------------------------------------
// 固定容量帧类型（ARD-0007 §21.2 原始帧逻辑字段：can_id/is_fd/is_brs/dlc/
// data[64]/rx_tick/bus_index）。ISR 盖 MCU 单调时基戳；内核不发原始帧。
// ---------------------------------------------------------------------------
#define MOTOR_PROTOCOL_MAX_FRAME_BYTES 64U

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  bool is_extended;
  bool is_fd;
  bool is_brs;
  uint8_t dlc;
  uint32_t rx_tick_ms; // MCU 单调时基（接收时刻）
  uint8_t data[MOTOR_PROTOCOL_MAX_FRAME_BYTES];
} motor_protocol_raw_frame_t;

// 待发送帧输出（固定容量）。tx_class 供每总线 TX 调度器
// 分配 P0/P1/P2 队列（ARD-0007 §22）。
typedef enum {
  MOTOR_PROTOCOL_TX_CLASS_P0 = 0U, // 安全帧：软停/停机/失能（可抢占）
  MOTOR_PROTOCOL_TX_CLASS_P1 = 1U, // 50 Hz MIT 控制（latest-wins 槽）
  MOTOR_PROTOCOL_TX_CLASS_P2 = 2U  // 发现/配置管理（有界配额）
} motor_protocol_tx_class_t;

typedef struct {
  uint8_t bus_index;
  motor_protocol_tx_class_t tx_class;
  // 内核注册表表项索引（只增不减故稳定；同总线两电机必有不同索引）：
  // P1 = latest-wins 槽位；P0 = 下游（调度器 FIFO 满）回告重入队按此定位
  // 表项；组帧点必须填写。P2 使用 FIFO，忽略此字段。
  uint16_t tx_slot;
  uint32_t can_id;
  bool is_extended;
  bool is_fd;
  bool is_brs;
  uint8_t size;
  uint8_t data[MOTOR_PROTOCOL_MAX_FRAME_BYTES];
} motor_protocol_tx_frame_t;

// ---------------------------------------------------------------------------
// 型号 Profile（ARD-0007 §8.2）：编译期"型号目录"的行类型。
// 数值来源：ros2_ws/src/motor_hardware_interface/config/motor.yaml 型号库
// （仓库唯一权威），Vaxor-80 速比无完整规格，刻意不入目录（禁止项 15）。
// ---------------------------------------------------------------------------
typedef struct {
  const char *model_name; // 与 ROS 型号库同名（TBD-PROFILE-01 一致性核对的锚点）
  // 设备自报身份串（协议的身份读取对象回串，实测 CERB SDO 0x1008：
  // "R48LITE_V2" / "R58LITE_V2"）——型号绑定的唯一自动来源。
  // NULL = 该型号无可读身份（协议无身份读对象），绑定只能靠"协议内唯一型号"
  // 兜底（见 motor_control_profile_sole）。
  const char *identity_string;
  const struct motor_protocol_adapter *protocol; // 所属协议 adapter
  float gear_ratio;                // 电机轴↔关节轴减速比
  float gear_efficiency;           // 关节侧扭矩效率
  float torque_constant_nm_per_amp; // 电机轴 Kt：反馈 Iq(A) → 电机轴 Nm
  float rated_torque_nm;           // 关节侧额定扭矩（0 = 未确认）
  float peak_torque_nm;            // 关节侧峰值扭矩（0 = 未确认，不参与钳制）
  // 电机端电流限值（规格书原始值，0 = 未确认）。设备侧管理对象 0x2538
  // （「电机最大电流参数」，mA）是**电气上限**：外部写入不得把设备里的电流
  // 推到峰值电流之上，故钳制必须同时看这一列，而不只是扭矩列。
  float rated_current_amp;
  float peak_current_amp;
  float rated_speed_rad_s;         // 关节侧额定转速（0 = 未确认）
  float supply_voltage_v;          // 标称供电电压
  float default_kp;                // 默认 MIT 增益（使能/软停阻尼用）
  float default_kd;
  float mit_position_limit_rad;    // MIT 量化范围（CERB 逐帧量化用）
  float mit_velocity_limit_rad_s;
  float mit_torque_limit_nm;
  float kp_max;                    // 增益线协议上限（Policy 钳制用）
  float kd_max;
  uint32_t capability_flags;       // MOTOR_PROTOCOL_CAP_*
} motor_protocol_model_profile_t;

// ---------------------------------------------------------------------------
// 控制意图与测量状态（SI、关节侧；内核与 adapter 的唯一交换形状）。
// ---------------------------------------------------------------------------
typedef struct {
  float position_rad;   // 关节侧目标位置
  float velocity_rad_s; // 关节侧目标速度
  float effort_nm;      // 关节侧目标扭矩（前馈，REV-02/TBD-ROS-03：唯一动态目标）
  float kp;
  float kd;
  bool enable; // 设备侧使能位（使能时序由内核状态机编排）
} motor_protocol_control_intent_t;

// 测量状态：缺失量一律 NaN（ARD-0007 §38.3）。profile 未绑定时运动量为 NaN，
// 但身份/状态字段照常填充（发现与识别依赖它）。
typedef struct {
  float position_rad;    // 关节侧（已按 Profile 换算；未绑定为 NaN）
  float velocity_rad_s;  // 关节侧
  float effort_nm;       // 关节侧（Iq×Kt×减速比×效率）
  float current_a;       // 电机电流（vaxor/CERB 反馈均有）
  float voltage_v;       // 仅 vaxor 20B 状态帧
  float temperature_c;   // 当前两协议均无来源，恒 NaN
  uint8_t drive_mode;    // 厂家模式字节（原样透传）
  uint32_t vendor_fault_code; // 厂家故障字（原样透传）
  uint8_t vendor_state;  // vaxor motor_state 字节
  uint8_t calibration_state;
  uint8_t motor_type;    // vaxor 型号线索（绑定 Profile 的诊断参考）
  bool has_motion_state; // 本帧含位置/速度/电流运动量（12B 反馈）
  bool is_status_frame;  // vaxor 20B 状态帧
  bool fault_active;     // 厂家故障位有效（如 CERB data[11]&0x20）
} motor_protocol_measured_state_t;

// ---------------------------------------------------------------------------
// 低频配置请求（形状与 control_mailbox 契约一致但类型独立，内核不依赖 mailbox）。
// 事务结果码为 ARD-0007 §20.5 统一七码。
// ---------------------------------------------------------------------------
typedef enum {
  MOTOR_PROTOCOL_CONFIG_SET_MODE = 1U,
  MOTOR_PROTOCOL_CONFIG_SET_GAINS = 2U,
  MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS = 3U,
  MOTOR_PROTOCOL_CONFIG_SET_ENABLE = 4U, // 逐电机 Enable（ARD §19 MotorPolicy）
  // 设备侧管理对象（仅 CERB；走 CANopen 快速 SDO，写后读回核验）：
  //   0x6080 电机最大转速（电机端 RPM）
  MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT = 5U,
  //   0x2538 电机峰值电流（电机端 mA）
  MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT = 6U,
  // 内核内部事务（不属 mailbox 契约，1..6 的数值对应关系不受影响）：
  // 读设备自报身份串，结果不回 CONFIG_RESULT，用于型号自动绑定。
  MOTOR_PROTOCOL_CONFIG_READ_IDENTITY = 7U,
  // 设零点（属 mailbox 契约，与 control_mailbox 侧同值 9 锁死）：CERB 写
  // 0x2531（字典「设置电机输出端零位」，写 1；手册注明「设置时电机需要在失能
  // 状态」）。单阶段：只等 SDO 写应答（0x60），不做读回——0x2531 是命令式对象，
  // 读回值语义未定义。不跟 0x2539 保存码：字典未给出"零位"对应的保存码。
  MOTOR_PROTOCOL_CONFIG_SET_ZERO = 9U,
  // 设备参数回读（属 mailbox 契约，与 control_mailbox 侧同值 8 锁死）：
  // 顺序 upload 0x607D:01/02、0x6080、0x2538，每阶段追加一个
  // motor_protocol_param_stage_t（设备端原值 + 关节侧换算值）进读缓冲，
  // 内核据此刷新设备参数影子并与固件记录比对。仅 CERB。
  MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS = 8U,
  // 故障清除（属 mailbox 契约，与 control_mailbox 侧同值 10 锁死）：主动
  // 下发厂家清错手段。CERB 是 MIT 控制字 bit5（清错位），单阶段、不等应答
  // ——是否真清掉由设备后续状态帧的 fault_active 判定（既有
  // FAULT → READY_DISABLED 路径，无新确认机制）。协议无清错手段时
  // config_stage_count 返回 0，内核统一回 UNSUPPORTED（不猜厂家字节）。
  MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT = 10U,
  // 逐电机策略（属 mailbox 契约，与 control_mailbox 侧同值 11 锁死）：enable +
  // MIT 增益，**一次原子更新**。走本地的 Policy 路径，不产生任何设备事务。
  // 拆成 SET_GAINS + SET_ENABLE 两次调用会留下「用旧 Kd 使能」的窗口。
  MOTOR_PROTOCOL_CONFIG_SET_POLICY = 11U,
  // 设备侧看门狗/限位保护开关（属 mailbox 契约，与 control_mailbox 侧同值 12
  // 锁死）：CERB 写 0x2650:00 = 0（保护生效）/ 1（关闭看门狗 + 限位保护），
  // 写后读回核验，**不跟 0x2539 保存帧**（手册没给这个对象对应的保存码，
  // **掉电不保存**（2026-09-19 实测）；0x2539 保存码对它无效）。仅 CERB；
  // Vaxor 阶段数 0 → 内核回 UNSUPPORTED。
  MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT = 12U
} motor_protocol_config_kind_t;

typedef enum {
  MOTOR_PROTOCOL_CONFIG_OK = 0U,
  MOTOR_PROTOCOL_CONFIG_BUSY = 1U,
  MOTOR_PROTOCOL_CONFIG_INVALID = 2U,
  MOTOR_PROTOCOL_CONFIG_UNSUPPORTED = 3U,
  MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR = 4U,
  MOTOR_PROTOCOL_CONFIG_TIMEOUT = 5U,
  MOTOR_PROTOCOL_CONFIG_UNCONFIRMED = 6U
} motor_protocol_config_result_t;

typedef struct {
  float min_position_rad;
  float max_position_rad;
} motor_protocol_position_limits_t;

typedef struct {
  uint32_t request_id; // 调用方提供，非 0（TBD-CONFIG-02：回绕/去重规则未冻结）
  uint16_t motor_id;   // 内核侧键；adapter 阶段拿到的已是解析后的 node
  motor_protocol_config_kind_t kind;
  bool enable; // SET_ENABLE（逐电机 Enable，ARD §19）
  uint8_t mode; // SET_MODE
  float kp;     // SET_GAINS
  float kd;
  motor_protocol_position_limits_t position_limits; // SET_POSITION_LIMITS（rad）
  float max_velocity_rad_s; // SET_VELOCITY_LIMIT（关节侧 rad/s）
  float max_torque_nm;      // SET_TORQUE_LIMIT（关节侧 N·m）
  // SET_SAFETY_PROTECT：设备端**原值**（0 = 保护开，1 = 关）。这里刻意收原值
  // 而不是"保护是否生效"的布尔——协议层只认厂家字节，方向语义的反转只允许
  // 发生在参数面（param_bridge 的 limit_protect → 本字段）一处。
  uint32_t safety_protect_raw;
  // SET_POLICY：代数必须严格前进才受理「从未武装到武装」的迁移（防周期重发
  // 自动复权）。其余 kind 忽略本字段。
  uint32_t policy_generation;
} motor_protocol_config_request_t;

// 设备端参数槽位（顺序是内核影子/固件记录/比对的契约，勿重排；前四槽与
// PARAM_FIELD_POS_MIN/POS_MAX/VEL_LIMIT/TORQUE_LIMIT 四字段一一对应，
// 后两槽是 v3 的设备侧安全事实，无对应 SI 字段）。
#define MOTOR_PROTOCOL_PARAM_SLOT_NONE 0xFFU
#define MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN 0U
#define MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX 1U
#define MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX 2U
#define MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX 3U
// v3：设备侧安全事实（无 SI 换算，原值透传）。
#define MOTOR_PROTOCOL_PARAM_SLOT_SAFETY 4U    // 0x2650:00
#define MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION 5U // 0x2541
#define MOTOR_PROTOCOL_PARAM_SLOT_COUNT 6U

// REFRESH_PARAMS 每阶段回填载荷（读缓冲按阶段顺序追加，每阶段一个）。
// raw = 设备端原值（cnt / 电机端 RPM / 电机端 mA），value_si = 关节侧换算值
// （rad / rad/s / N·m）。原值是与固件记录比对的依据，SI 值供影子与页面。
typedef struct {
  uint32_t raw;
  float value_si;
} motor_protocol_param_stage_t;

// 配置事务阶段（ARD-0007 §20.4：分阶段、跨 1ms 节拍步进、每阶段可重试 ≤2 次）。
//
// param_* 字段：本阶段与之对应的**设备端参数槽位**与原值（CERB 的
// 0x607D:01/:02、0x6080、0x2538）。内核据此在事务成功后写固件记录、并把
// 回读原值与记录比对——比对必须以设备端原值为准：关节侧 SI 值两端都过同一
// 套 Kt×减速比换算，系数配错时两边一起错、比对恒通过（见内核 dev_init）。
// 不涉及设备参数的阶段（身份读、Vaxor 本地策略）必须保持 param_raw_valid=false。
typedef struct {
  bool waits_response; // 本阶段是否等待设备响应帧
  uint32_t timeout_ms; // 等待超时（无响应阶段忽略）
  bool param_raw_valid; // 本阶段是否携带设备端参数（原值/槽位有意义）
  uint8_t param_slot;   // MOTOR_PROTOCOL_PARAM_SLOT_*
  uint32_t param_raw;   // 写=写入值；读=请求对象（回值由读缓冲带回）
} motor_protocol_config_stage_info_t;

// 响应裁决：ADVANCE=进入下一阶段；RETRY=重发本阶段；FAIL=读回不一致等失败
// （内核映射为 UNCONFIRMED——读回类失败不得返回 OK，ARD §20.4/禁止项 9）；
// COMPLETE=本阶段即整个事务的终态（设备提前给出末段，后续阶段不再发送）；
// IGNORE=本帧不是本阶段的应答（如同一节点的周期上报与事务应答共用归属），
// 保持等待与重试计数不变，帧交由其他路径继续处理。
typedef enum {
  MOTOR_PROTOCOL_STAGE_ADVANCE = 0U,
  MOTOR_PROTOCOL_STAGE_RETRY = 1U,
  MOTOR_PROTOCOL_STAGE_FAIL = 2U,
  MOTOR_PROTOCOL_STAGE_COMPLETE = 3U,
  MOTOR_PROTOCOL_STAGE_IGNORE = 4U
} motor_protocol_stage_outcome_t;

// 读取类事务的载荷回填缓冲（调用方提供固定容量存储；adapter 只追加）。
// 溢出置 overflow 而不截断静默——内核据其判读取失败（不得用残缺身份绑定）。
typedef struct {
  uint8_t *data;
  size_t capacity;
  size_t size;
  bool overflow;
} motor_protocol_read_buffer_t;

// ---------------------------------------------------------------------------
// adapter 接口（C 函数表）。所有 encode/decode 均为纯函数：不持静态状态，
// 使能时序/事务状态由内核保存；字段顺序/帧长/FD 属性只出现在本库实现内
// （TBD-CAN-03：各帧型 FD/BRS/DLC 属性确认后改实现，不改接口）。
// ---------------------------------------------------------------------------
typedef struct motor_protocol_adapter {
  const char *protocol_name;
  uint16_t node_id_min; // 厂家节点号合法范围（vaxor 1..60，CERB 1..127）
  uint16_t node_id_max;
  uint32_t offline_timeout_ms; // OFFLINE 判定阈值占位（TBD-SAFETY-03）
  uint32_t enable_settle_ms;   // 使能后到可发运动目标的整定时间（CERB 100ms）
  bool autonomous_feedback;    // true：存在与命令无关的 20ms 状态心跳（vaxor
                               // 20B 状态帧 / CERB SYNC 上报帧，均承担总线检测）
                               // ——OFFLINE 按心跳龄判定；false：反馈仅在命令
                               // 期出现——只在"正在发命令却持续无反馈"时判离线
  // true：配置事务的写阶段无厂家确认手段（无读回对象），事务成功只能收敛
  // UNCONFIRMED 不得报 OK（ARD §20.5 / 禁止项 9）。确认对象在 TBD-CONFIG-01
  // 关闭后由该协议 adapter 更新。
  bool config_verify_required;

  // 帧归属：该帧是否属于本协议；是则给出厂家节点号。两协议 ID 段不重叠。
  // 含协议内一切可归属帧——含事务响应帧（如 CERB 的 SDO 应答 0x580+node），
  // 它们要能命中已知表项完成裁决。
  bool (*match_rx_frame)(const motor_protocol_raw_frame_t *frame, uint16_t *out_node);

  // "设备存在"证据帧判定（发现建档/重连的唯一依据）：match_rx_frame 命中的
  // 帧里，只有周期上报/反馈类帧能证明设备在线上。事务响应帧可归属到已知
  // 设备，但不得凭它凭空建档。NULL = 本协议 match 到的帧皆是存在证据。
  bool (*is_discovery_frame)(const motor_protocol_raw_frame_t *frame);

  // 解码 + SI 换算。profile 为 NULL（未绑定）时运动量为 NaN，
  // 身份/状态字段照常填充。失败（长度/魔数不符）返回 false。
  bool (*decode_frame)(const motor_protocol_model_profile_t *profile, uint16_t node,
                       const motor_protocol_raw_frame_t *frame,
                       motor_protocol_measured_state_t *out);

  // 50 Hz MIT 控制帧（SI 关节侧目标；软件限位钳制发生在内核，量化在 codec）。
  bool (*encode_mit_command)(const motor_protocol_model_profile_t *profile, uint16_t node,
                             uint8_t bus_index,
                             const motor_protocol_control_intent_t *intent,
                             motor_protocol_tx_frame_t *out);

  // 使能帧（中性目标 + 使能位）。vaxor=control(run=1,mode)，CERB=中性 MIT。
  bool (*encode_enable)(const motor_protocol_model_profile_t *profile, uint16_t node,
                        uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 软停帧（30ms 无快照/快照缺席路径）。TBD-SAFETY-01：默认 MIT 阻尼帧
  // {0,0,0,default_kd,0} 且保持使能——厂家级语义关闭前不得更改安全边界。
  bool (*encode_soft_stop)(const motor_protocol_model_profile_t *profile, uint16_t node,
                           uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 协议级停机/失能帧（100ms 超时/断链路径，P0）。
  bool (*encode_disable)(const motor_protocol_model_profile_t *profile, uint16_t node,
                         uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 发现/重连/热插拔探测帧（CERB 0x080 SYNC；vaxor 无 → 返回 false）。
  // 禁止作为常规 50Hz 控制触发（ARD §23.4、禁止项 14）。
  bool (*encode_discovery_sync)(uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 配置事务：阶段数（0 = 该请求不被本协议支持 → 内核回 UNSUPPORTED）。
  // READ_IDENTITY 上返回 0 = 本协议无身份读对象（内核回退到"协议内唯一型号"
  // 兜底绑定，不再发帧）。
  uint8_t (*config_stage_count)(const motor_protocol_model_profile_t *profile,
                                const motor_protocol_config_request_t *request);
  bool (*config_stage_info)(uint8_t stage,
                            const motor_protocol_model_profile_t *profile,
                            const motor_protocol_config_request_t *request,
                            motor_protocol_config_stage_info_t *out);
  bool (*config_encode_stage)(uint8_t stage,
                              const motor_protocol_model_profile_t *profile, uint16_t node,
                              uint8_t bus_index,
                              const motor_protocol_config_request_t *request,
                              motor_protocol_tx_frame_t *out);
  // profile 传入的原因：设备侧管理对象的读回核验要做单位换算
  // （0x6080 电机端 RPM、0x2538 电机端 mA），换算系数在 profile 里。
  // read_out 仅读取类事务（READ_IDENTITY）使用：adapter 追加设备回传载荷，
  // 其余事务传 NULL 并忽略。调用方保证缓冲在事务全程有效。
  // 契约：只在返回 ADVANCE/COMPLETE 的路径上追加；判 RETRY/FAIL/IGNORE
  // 之前不得写入缓冲——重试会重发同一阶段，内核不替 adapter 回退载荷。
  motor_protocol_stage_outcome_t (*config_on_response)(
      uint8_t stage, const motor_protocol_model_profile_t *profile,
      const motor_protocol_config_request_t *request,
      const motor_protocol_raw_frame_t *response,
      motor_protocol_read_buffer_t *read_out);
} motor_protocol_adapter_t;

// 内置 adapter（新增协议家族 = 新增一个实例，内核零改动）。
extern const motor_protocol_adapter_t motor_protocol_vaxor_adapter;
extern const motor_protocol_adapter_t motor_protocol_cerb_adapter;

// 限值对象（0x2538 电流上限）的关节侧换算系数 N·m/A。
// 优先用规格书的**峰值对**：峰值扭矩 ↔ 峰值电流两端都写在规格书上，用它换算
// 可保证"写到峰值扭矩"正好等于"写到峰值电流"，即钳制口径与规格书同源；
// 缺任一字段时退回 Kt×减速比×效率（反馈链口径）。
// 注意：反馈（Iq → 实测扭矩估计）仍走 Kt×减速比×效率，两者是不同量——前者是
// 限值映射、后者是物理估计；规格书自身的额定/峰值两点隐含的效率并不相同
// （R58 额定 0.66、峰值 0.48），单一常数无法同时命中，故各取所需。
static inline float motor_protocol_profile_limit_nm_per_amp(
    const motor_protocol_model_profile_t *profile) {
  if (profile == NULL) {
    return 0.0f;
  }
  if (profile->peak_torque_nm > 0.0f && profile->peak_current_amp > 0.0f) {
    return profile->peak_torque_nm / profile->peak_current_amp;
  }
  return profile->torque_constant_nm_per_amp * profile->gear_ratio *
         profile->gear_efficiency;
}

// 节点号合法性（vaxor ID 算术函数不校验范围，入口统一在此把关）。
static inline bool motor_protocol_adapter_valid_node(
    const motor_protocol_adapter_t *adapter, uint16_t node) {
  return adapter != NULL && node >= adapter->node_id_min &&
         node <= adapter->node_id_max;
}

#ifdef __cplusplus
}
#endif

#endif // MOTOR_PROTOCOL_ADAPTER_H
