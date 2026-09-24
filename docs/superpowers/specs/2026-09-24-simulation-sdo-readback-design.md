# 仿真设备参数 SDO 回读设计

- 日期：2026-09-24
- 状态：待用户审阅
- 范围：host + vcan 仿真设备参数只读回读；真机数据通路不变

## 背景

WebHost 设备参数页当前只订阅固件的 `/micro_ros_up/device_param_report` 和
`/micro_ros_up/system_status`。纯软件仿真没有 micro-ROS Agent 与固件，因此参数页
无法列出或回读虚拟电机。仿真中的 Gazebo `/joint_states` 是关节运动状态，不能代替
驱动器对象字典的 SDO 回读。

目标是在仿真和真机之间建立显式、互不混用的数据源。仿真回读必须在线路上实际发送
CANopen SDO 请求，并接收虚拟电机通过 vcan 发回的应答。

## 决策

### 数据源边界

- 设备参数面提供“真机 / 仿真”来源选择。默认保持现有真机来源。
- 真机继续使用 `/micro_ros_up/*` 与 `/micro_ros_do/*`；仿真只使用
  `/motor_sim/*`。两边的状态、请求和浏览器缓存分别维护，切换来源不会把另一来源的
  数据当作当前读数，也不会自动回退到另一来源。
- 本次来源切换只作用于设备参数面。主监控、运动状态与 3D 视图不在本次范围。
- 仿真参数面只读：保留全量读取和逐电机回读；隐藏修改、W25 保存、设零和清错操作。

### 仿真 SDO 请求链路

在 `motor_sim` 包内增加独立的 `sim_device_param_gateway` ROS 2 节点。它使用独立
SocketCAN socket 绑定仿真 vcan 接口；`motor_sim` 原有节点负责作为虚拟设备接收并答复
SDO。两个 socket 都通过同一个 vcan CAN 网络收发，ros2_control 的 CAN socket 继续
独立工作并忽略 SDO 帧。

`robot_arm_gazebo/simulation.launch.py` 只在
`controller_mode:=host transport:=vcan` 时启动网关，并把同一 vcan 接口传给网关和
模拟器。其他控制模式、物理 SocketCAN 和 SLCAN 首期不启动该网关，也不宣称支持
仿真参数 SDO 回读。

现有消息类型复用在独立命名空间：

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| WebHost → 网关 | `/motor_sim/system_command` | `can_bridge_msgs/msg/SystemCommand` | `REFRESH` 单台读取；全量读取由页面逐台串行发送 |
| 网关 → WebHost | `/motor_sim/device_param_report` | `can_bridge_msgs/msg/DeviceParamReport` | 仿真配置基准与 SDO 回读快照 |
| 网关 → WebHost | `/motor_sim/system_status` | `can_bridge_msgs/msg/SystemStatus` | 独立错误状态和 `refresh_seq` 完成序号 |
| 网关 → WebHost | `/motor_sim/motor_events` | `can_bridge_msgs/msg/MotorEvent` | `DEV_INIT` 核对结论及不一致/未读槽位位图 |

WebHost 仍通过话题发布请求，不新增 ROS 服务，也不向真机命令话题发送仿真请求。

### 虚拟对象字典与单位

模型的 `simulation_topology.yaml` 为每个 CERB 映射电机声明仿真配置基准和可选的
SDO 原始值覆盖；省略覆盖时，对象字典从仿真限位、速度、电流配置和显式的保护/方向
设置派生，因此初始回读与仿真配置一致。覆盖可用于模拟设备值不一致或缺失对象。
`simulation.launch.py` 继续从权威真机拓扑绑定电机 ID 和协议，并把仿真对象字典写入
运行时配置。ID 不在仿真配置中重复声明。对象值按设备线协议原始整数存储：

| SDO 对象 | 设备值 | WebHost 显示值 |
|---|---|---|
| `0x607D:01/02` | 负载位置限位，cnt | rad，`65536 cnt / 2π rad` |
| `0x6080:00` | 电机最大转速，RPM | 关节 rad/s，除以减速比 |
| `0x2538:00` | 峰值电流，mA | A |
| `0x2650:00` | 保护开关原值 | 正向保护语义，按真机规则反转 |
| `0x2541:00` | 方向原值，0=CW / 1=CCW | 原值直通 |

参数报告的 `rec_*` 表示模型仿真配置基准，`dev_*` 表示本次 SDO 应答换算结果；仿真
界面分别标为“仿真配置 / SDO 回读”。同一消息类型在真机来源下仍标为“固件记录 /
驱动板回读”。网关启动后周期发布配置基准报告，为页面提供可回读的电机 ID；未读取
前的 `dev_*` 用 NaN、状态为 PENDING。虚拟字典与基准默认一致；配置缺项或设备未
返回的数值用 NaN，不填 0。

首期只实现已有固件 SDO 契约中的 CERB 对象。VAXOR 没有本次涉及的 SDO 对象字典，
显示“无设备对象”，不伪造设备值，也不向其协议地址发 CERB SDO。

### 事务、并发与错误

- 网关一次只保留一个未完成 SDO 请求；单台读取按对象顺序串行执行。页面全量读取按
  电机逐台等待完成后再发下一条 `REFRESH`，避免同一 CANopen 节点上并发主站事务。
- 使用标准 CAN ID：请求 `0x600 + motor_id`，响应 `0x580 + motor_id`；加速上传请求
  为 `0x40 index_lo index_hi sub 00 00 00 00`。CERB SDO 使用 8 字节 CAN-FD/BRS 帧，
  应答匹配 motor ID、索引和子索引。
- 只对超时重试；收到 SDO ABORT 后按明确失败处理。未知对象按虚拟 CERB 设备的 ABORT
  行为应答。每次请求无论成功或失败都会推进仿真 `refresh_seq`。成功但与仿真配置
  基准不一致时用错误码 40 和 `DEV_INIT` 的不一致槽位位图；超时或 ABORT 用错误码
  41 和未读槽位位图。报告中的缺失字段保持 NaN。
- 仿真状态序号与错误值只存在于 `/motor_sim/system_status`。来源切换时以选中来源的
  序号作为回读基线，避免另一来源的状态帧让事务误判完成。

## 文件与模块边界

- `motor_sim`：解析虚拟 SDO 对象字典；接收 CERB SDO 上传请求并返回应答；新增独立
  SDO 网关可执行入口与单事务读取逻辑。
- `robot_arm_gazebo`：生成包含仿真对象字典的运行时配置；仅为 host-vcan 组合启动网关。
- `web_host/web/app.js` 与 `index.html`：设备参数来源选择、来源独立缓存与回读请求；
  仿真源用只读控件和来源专属错误文案。
- `motor_drive` 与 `motor_hardware_interface`：不修改。
- 固件、Agent 与 `/micro_ros_*` 真机话题契约：不修改。

## 验证标准

1. `motor_sim` 的 SDO 单元验证覆盖已配置对象的加速上传应答、未知对象 ABORT 和非
   CERB 地址忽略；网关事务覆盖匹配响应、超时、ABORT 与串行全量读取。
2. host-vcan launch 启动网关并使用同一 vcan 接口；其他传输/控制模式不启动它。
3. WebHost 切到仿真源后只发 `/motor_sim/system_command`，收取 `/motor_sim/*` 报告，
   显示值与 SDO 原始单位换算一致；切回真机后只使用原有固件报告和状态。
4. 对相关 ROS 2 包执行构建；自动验证不启动 Gazebo 或真实硬件。

## 非目标

- 仿真设备参数写入、保存、设零、清错或模拟固件参数记录持久化。
- 改变仿真状态、3D 姿态、曲线、使能/增益等其他 WebHost 数据来源。
- 为物理 SocketCAN、SLCAN、外部控制器或固件 HIL 暴露仿真回读。
- 增加 VAXOR SDO 协议实现，或通过 `/joint_states` 推算设备对象参数。
