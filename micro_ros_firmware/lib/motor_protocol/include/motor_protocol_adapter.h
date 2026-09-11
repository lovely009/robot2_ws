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
#define MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE (1UL << 3)  // CREB 驱动器侧限位
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
  MOTOR_PROTOCOL_TX_CLASS_P1 = 1U, // 100 Hz MIT 控制（latest-wins 槽）
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
  const struct motor_protocol_adapter *protocol; // 所属协议 adapter
  float gear_ratio;                // 电机轴↔关节轴减速比
  float gear_efficiency;           // 关节侧扭矩效率
  float torque_constant_nm_per_amp; // 电机轴 Kt：反馈 Iq(A) → 电机轴 Nm
  float rated_torque_nm;           // 关节侧额定扭矩（0 = 未确认）
  float peak_torque_nm;            // 关节侧峰值扭矩（0 = 未确认，不参与钳制）
  float rated_speed_rad_s;         // 关节侧额定转速（0 = 未确认）
  float supply_voltage_v;          // 标称供电电压
  float default_kp;                // 默认 MIT 增益（使能/软停阻尼用）
  float default_kd;
  float mit_position_limit_rad;    // MIT 量化范围（CREB 逐帧量化用）
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
  float current_a;       // 电机电流（vaxor/CREB 反馈均有）
  float voltage_v;       // 仅 vaxor 20B 状态帧
  float temperature_c;   // 当前两协议均无来源，恒 NaN
  uint8_t drive_mode;    // 厂家模式字节（原样透传）
  uint32_t vendor_fault_code; // 厂家故障字（原样透传）
  uint8_t vendor_state;  // vaxor motor_state 字节
  uint8_t calibration_state;
  uint8_t motor_type;    // vaxor 型号线索（绑定 Profile 的诊断参考）
  bool has_motion_state; // 本帧含位置/速度/电流运动量（12B 反馈）
  bool is_status_frame;  // vaxor 20B 状态帧
  bool fault_active;     // 厂家故障位有效（如 CREB data[11]&0x20）
} motor_protocol_measured_state_t;

// ---------------------------------------------------------------------------
// 低频配置请求（形状与 control_mailbox 契约一致但类型独立，内核不依赖 mailbox）。
// 事务结果码为 ARD-0007 §20.5 统一七码。
// ---------------------------------------------------------------------------
typedef enum {
  MOTOR_PROTOCOL_CONFIG_SET_MODE = 1U,
  MOTOR_PROTOCOL_CONFIG_SET_GAINS = 2U,
  MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS = 3U,
  MOTOR_PROTOCOL_CONFIG_SET_ENABLE = 4U // 逐电机 Enable（ARD §19 MotorPolicy）
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
} motor_protocol_config_request_t;

// 配置事务阶段（ARD-0007 §20.4：分阶段、跨 1ms 节拍步进、每阶段可重试 ≤2 次）。
typedef struct {
  bool waits_response; // 本阶段是否等待设备响应帧
  uint32_t timeout_ms; // 等待超时（无响应阶段忽略）
} motor_protocol_config_stage_info_t;

// 响应裁决：ADVANCE=进入下一阶段；RETRY=重发本阶段；FAIL=读回不一致等失败
// （内核映射为 UNCONFIRMED——读回类失败不得返回 OK，ARD §20.4/禁止项 9）。
typedef enum {
  MOTOR_PROTOCOL_STAGE_ADVANCE = 0U,
  MOTOR_PROTOCOL_STAGE_RETRY = 1U,
  MOTOR_PROTOCOL_STAGE_FAIL = 2U
} motor_protocol_stage_outcome_t;

// ---------------------------------------------------------------------------
// adapter 接口（C 函数表）。所有 encode/decode 均为纯函数：不持静态状态，
// 使能时序/事务状态由内核保存；字段顺序/帧长/FD 属性只出现在本库实现内
// （TBD-CAN-03：各帧型 FD/BRS/DLC 属性确认后改实现，不改接口）。
// ---------------------------------------------------------------------------
typedef struct motor_protocol_adapter {
  const char *protocol_name;
  uint16_t node_id_min; // 厂家节点号合法范围（vaxor 1..60，CREB 1..127）
  uint16_t node_id_max;
  uint32_t offline_timeout_ms; // OFFLINE 判定阈值占位（TBD-SAFETY-03）
  uint32_t enable_settle_ms;   // 使能后到可发运动目标的整定时间（CREB 100ms）
  bool autonomous_feedback;    // true：存在与命令无关的 20ms 状态心跳（vaxor
                               // 20B 状态帧 / CREB SYNC 上报帧，均承担总线检测）
                               // ——OFFLINE 按心跳龄判定；false：反馈仅在命令
                               // 期出现——只在"正在发命令却持续无反馈"时判离线
  // true：配置事务的写阶段无厂家确认手段（无读回对象），事务成功只能收敛
  // UNCONFIRMED 不得报 OK（ARD §20.5 / 禁止项 9）。确认对象在 TBD-CONFIG-01
  // 关闭后由该协议 adapter 更新。
  bool config_verify_required;

  // 帧归属：该帧是否属于本协议；是则给出厂家节点号。两协议 ID 段不重叠。
  bool (*match_rx_frame)(const motor_protocol_raw_frame_t *frame, uint16_t *out_node);

  // 解码 + SI 换算。profile 为 NULL（未绑定）时运动量为 NaN，
  // 身份/状态字段照常填充。失败（长度/魔数不符）返回 false。
  bool (*decode_frame)(const motor_protocol_model_profile_t *profile, uint16_t node,
                       const motor_protocol_raw_frame_t *frame,
                       motor_protocol_measured_state_t *out);

  // 100 Hz MIT 控制帧（SI 关节侧目标；软件限位钳制发生在内核，量化在 codec）。
  bool (*encode_mit_command)(const motor_protocol_model_profile_t *profile, uint16_t node,
                             uint8_t bus_index,
                             const motor_protocol_control_intent_t *intent,
                             motor_protocol_tx_frame_t *out);

  // 使能帧（中性目标 + 使能位）。vaxor=control(run=1,mode)，CREB=中性 MIT。
  bool (*encode_enable)(const motor_protocol_model_profile_t *profile, uint16_t node,
                        uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 软停帧（30ms 无快照/快照缺席路径）。TBD-SAFETY-01：默认 MIT 阻尼帧
  // {0,0,0,default_kd,0} 且保持使能——厂家级语义关闭前不得更改安全边界。
  bool (*encode_soft_stop)(const motor_protocol_model_profile_t *profile, uint16_t node,
                           uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 协议级停机/失能帧（100ms 超时/断链路径，P0）。
  bool (*encode_disable)(const motor_protocol_model_profile_t *profile, uint16_t node,
                         uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 发现/重连/热插拔探测帧（CREB 0x080 SYNC；vaxor 无 → 返回 false）。
  // 禁止作为常规 100Hz 控制触发（ARD §23.4、禁止项 14）。
  bool (*encode_discovery_sync)(uint8_t bus_index, motor_protocol_tx_frame_t *out);

  // 配置事务：阶段数（0 = 该请求不被本协议支持 → 内核回 UNSUPPORTED）。
  uint8_t (*config_stage_count)(const motor_protocol_model_profile_t *profile,
                                const motor_protocol_config_request_t *request);
  bool (*config_stage_info)(uint8_t stage,
                            const motor_protocol_config_request_t *request,
                            motor_protocol_config_stage_info_t *out);
  bool (*config_encode_stage)(uint8_t stage,
                              const motor_protocol_model_profile_t *profile, uint16_t node,
                              uint8_t bus_index,
                              const motor_protocol_config_request_t *request,
                              motor_protocol_tx_frame_t *out);
  motor_protocol_stage_outcome_t (*config_on_response)(
      uint8_t stage, const motor_protocol_config_request_t *request,
      const motor_protocol_raw_frame_t *response);
} motor_protocol_adapter_t;

// 内置 adapter（新增协议家族 = 新增一个实例，内核零改动）。
extern const motor_protocol_adapter_t motor_protocol_vaxor_adapter;
extern const motor_protocol_adapter_t motor_protocol_creb_adapter;

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
