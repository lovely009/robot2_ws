# micro_ros_firmware

STM32H723VGT6 上的 **micro-ROS 固件**（PlatformIO + STM32duino + FreeRTOS），
通过 **serial over USB CDC**（PA11=USB_DM / PA12=USB_DP）与主机侧
`../micro_ros_agent/` 通信，桥接进 ROS 2 Humble 域。

当前业务为 **BMI088 IMU** 与 **三路 FDCAN**，底层功能移植自官方
`/home/ly/文档/dm-mc02-master/例程/CtrBoard-H7_IMU_TempCtrl`：
SPI2 + DMA1 配置、POWER_5V/CS/EXTI、TIM3 加热与温度 PID 均保留；
Mahony 姿态解算与 micro-ROS 发布为本工程增强，不属于官方例程内容。
三路 FDCAN（FDCAN1/2/3）移植自
`/home/ly/文档/dm-mc02-master/例程/CtrBoard-H7_FDCAN`：FD + BRS、标称
1Mbps / 数据 5Mbps、接收中断回调与周期测试帧发送均保留；FDCAN1 已通过
`can_bridge_msgs/CANFrame` 接入 ROS 调试话题。
电机协议编解码（RP + 25/30mm）已从 `ros2_ws/src/motor_drive` 移植为
`lib/motor_protocol`（同一文件内按 RP / vaxor 两部分分区），字节布局与
原实现一致。固件暂不维护完整拓扑：`control_task` 内置两个默认测试电机
（RP + vaxor），订阅 `/joint_commands`（`sensor_msgs/msg/JointState`）
直接下发；完整拓扑后续通过参数服务下发，不在此扩展。

## 关键决策

| 决策 | 取值 | 理由 |
|---|---|---|
| 框架 | `framework = arduino`（STM32duino） | `micro_ros_platformio` 官方只支持 Arduino 框架 |
| 实时内核 | `STM32FreeRTOS`（stm32duino） | 调度器接管 SysTick，micro-ROS 与 IMU 独立任务 |
| ROS 2 发行版 | `board_microros_distro = humble` | 与 `micro_ros_agent` 源码对齐 |
| 传输 | `board_microros_transport = serial`（物理层 USB CDC） | micro-ROS 侧保持 serial，agent 以 `/dev/ttyACM0` 对接 |
| USB 引脚 | **PA11=USB_DM / PA12=USB_DP** | H723 的 USB OTG HS 内嵌 PHY（FS 模式）固定引脚 |
| USB 构建宏 | `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC` + `PIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED_FULLMODE` | H723 无独立 `USB_OTG_FS`，须以 FS 模式 + 内嵌 PHY 使用 |
| IMU 通信 | **SPI2**（PB13=SCK / PC1=MOSI / PC2=MISO，7.5MHz） | 官方 TempCtrl 同源 |
| IMU DMA | **DMA1_Stream0(RX) / Stream1(TX)** 保留配置并链接 | 官方生成代码同构；收发仍走有界阻塞 HAL，每字节最多 2ms |
| IMU 采样触发 | **ACC_INT=PE10 EXTI + FreeRTOS 二值信号量，ODR=200Hz** | 官方 TempCtrl 由加速度计 DRDY 唤醒；本工程从 800Hz 降至 200Hz |
| 加热 PWM | **TIM3_CH4=PB1**（500Hz），使用 Arduino `HardwareTimer` | 官方 PSC=24-1 / ARR=10000-1 语义 |
| 状态 LED | **PA7（WS2812，SPI6 MOSI 驱动）** | 板载 1 颗 RGB 灯珠 |
| CANFD 通信 | **FDCAN1=PD0/PD1、FDCAN2=PB5/PB6、FDCAN3=PD12/PD13**，FD + BRS，RX/TX 元素 64B | ISR 写静态 RX 队列；容量 CAN1=32、CAN2/3=8，队满丢最旧帧 |
| CANFD 时钟 | **PLL2Q = 80MHz**（标称 1Mbps / 数据 5Mbps） | 官方例程用 PLL1Q=80MHz；本工程 PLL1Q 已给 SPI2，故改用 PLL2Q |
| CANFD 构建 | `-DHAL_FDCAN_MODULE_ENABLED` | STM32duino 默认关闭 FDCAN HAL 模块 |
| CAN1 调试桥 | `-DENABLE_CAN1_DEBUG_BRIDGE=1`（默认开启） | 通过 `micro_ros/can1/send`、`micro_ros/can1/receive` 调试 FDCAN1；关掉则不在 ROS 侧建实体 |
| JointState 桥 | `-DENABLE_MOTOR_COMMAND_BRIDGE=1`（默认开启） | 订阅 `/joint_commands`（`sensor_msgs/msg/JointState`）驱动默认测试电机（RP + vaxor） |
| 板卡变体 | `STM32H7xx/H723V(E-G)(H-T)_H730VB(H-T)_H733VG(H-T)` + `variant_generic.h` | STM32duino 官方 GENERIC_H723VGTX 变体 |

## 时钟

`src/system_clock.cpp` 强定义覆盖 WEAK `SystemClock_Config`：

- HSE 24MHz → PLL1 → **480MHz SYSCLK**（HCLK 240MHz / APB 120MHz）
- **HSI48** 作为 USB 时钟源
- **SPI6 = HSE 24MHz**（WS2812 编码）
- **SPI123 = PLL1Q = 240MHz**（BMI088 SPI2，预分频 32 -> 7.5MHz）
- **FDCAN = PLL2Q = 80MHz**（HSE 24MHz / 2 × 40，Q=6；标称 1Mbps / 数据 5Mbps）

## 任务编排

`setup()` 只做板级初始化和创建 `start_task`，随后 `vTaskStartScheduler()`。
IMU 初始化不得放回 `setup()`：`BMI088_delay_ms` 内部使用 `vTaskDelay`，
调度器启动前调用会非法。

| 任务 | 优先级 | 栈 | 职责 |
|---|---|---|---|
| `start_task` | 1 | 256 | 创建 `micro_ros`/`sensor`/`control`/`supervisor` 后自删除 |
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发，20ms bridge timer 发布各域最新快照；断连自动重建；状态灯红/绿闪/蓝 |
| `sensor` | 4 | 1024 | 500ms 延时 → 加热 PWM → BMI088 单次初始化；失败时每 1s 降级重试，成功后由约 200Hz ACC DRDY 触发采样/Mahony/PID |
| `control` | 3 | 1024 | 500ms 延时 → FDCAN1/2/3 初始化；100Hz 统一消费命令/debug mailbox 并发送；官方 0x520 测试帧默认关闭（宏开启） |
| `supervisor` | 1 | 512 | 每 100ms 观察 control/sensor/micro-ROS 状态机 heartbeat，聚合 GDB 可读诊断；开发期不复位、不喂 IWDG |

supervisor 在各任务第一次 heartbeat 前保留启动宽限；进入运行期后，control
连续 100ms 无进展即告警，sensor 健康/降级阈值分别为 300ms/2500ms，micro-ROS
阈值为 1500ms（容纳 `rmw_uros_ping_agent(200, 3)` 的合法阻塞）。这些阈值只更新
`supervisor_diagnostics_t` 的 heartbeat、miss count 和 fault bitmask，当前开发阶段
是 **observation-only**：没有硬件 IWDG 初始化、喂狗或复位动作。

## 代码组织

业务按“大任务”分域，ROS 实体不堆在 `ros_bridge`：

| 模块 | 职责 |
|---|---|
| `ros_bridge` | agent 通信生命周期：node/executor/timer，断连整体重建；定时器依次轮询各域桥 |
| `imu_bridge` | `micro_ros/imu/data` 与 `micro_ros/imu/temp` 的缓冲/填充/发布 |
| `can_debug_bridge` | `micro_ros/can1/send` 校验/入 mailbox 与 `micro_ros/can1/receive` 发布（宏控制） |
| `motor_bridge` | `/joint_commands` 订阅展开 + `micro_ros/JointState` 反馈发布（宏控制） |
| `control_task` | 电机表、RP/vaxor 协议收发与反馈解码，不接触 ROS |
| `lib/fdcan` | FDCAN 初始化/收发/中断回调；ISR 写三路静态 RX 队列，主队列仅由 control task 消费 |
| `lib/imu` | BMI088 驱动/通信层/Mahony/SI 聚合（依赖 `lib/dma`） |
| `lib/motor_protocol` | RP/vaxor 协议编解码（纯逻辑） |
| `lib/motor_control` | 100/300ms timeout、协议状态机、RP slot 聚合及总线级 enable/disable（纯逻辑） |
| `lib/control_mailbox` | 固定容量命令/debug overwrite mailbox 与连接/授权边沿锁存（纯逻辑） |
| `lib/ws2812` | 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7） |
| `lib/dma` | 板级 DMA 初始化（当前 SPI2 专用）与中断转发 |

数据流：FDCAN1/2/3 ISR → 对应静态 RX 队列 → control task 有界消费并解码反馈；
control task 同时复制每路最新帧到只读 debug snapshot，`can_debug_bridge` 只读取
CAN1 的副本，不消费主队列。ROS 电机命令和 CAN debug TX 回调只覆盖静态
mailbox；control task 以 100Hz 绝对周期统一取走并作为正常 FDCAN 发送的唯一
任务上下文。IMU 则由 sensor task 写双缓冲 snapshot，20ms bridge timer 复制后发布，
桥接层不持有 sensor task 的内部源指针。

## 目录结构

```
micro_ros_firmware/
├── platformio.ini             # arduino/humble/serial+USB CDC；FDCAN HAL 构建宏
├── boards/
│   └── stm32h723vgt6.json     # 自定义板卡定义（变体/宏/USB VID-PID/烧录协议）
├── extra_packages/
│   └── can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs
│                               # micro-ROS 构建复用同仓库权威消息定义
├── include/                   # 应用层头文件（任务/ROS 桥）
│   ├── ros_bridge.h           # micro-ROS 生命周期与域桥编排接口
│   ├── imu_bridge.h           # IMU/温度域 ROS 桥接接口
│   ├── can_debug_bridge.h     # CAN1 调试域 ROS 桥接接口
│   ├── motor_bridge.h         # 电机域 ROS 桥接接口
│   ├── sensor_task.h          # 传感器任务接口（sensor_task_start）
│   └── control_task.h         # 控制任务接口（control_task_start）
├── lib/                       # 板级/协议库（PlatformIO 自动发现）
│   ├── imu/                   # BMI088 驱动 + 通信层 + Mahony + SI 聚合（依赖 dma）
│   ├── dma/                   # 板级 DMA：SPI2 配置 + HAL_SPI_MspInit/IRQ 转发
│   ├── fdcan/                 # 三路 FDCAN 初始化/收发/中断回调/静态 RX 队列
│   ├── motor_control/         # 电机 timeout/故障/恢复状态机（host 可测）
│   ├── control_mailbox/       # 固定容量 mailbox 与边沿锁存（host 可测）
│   ├── motor_protocol/        # RP/vaxor 协议编解码（纯逻辑）
│   └── ws2812/                # WS2812 彩灯驱动（SPI6 MOSI=PA7）
└── src/
    ├── main.cpp               # 入口：板级初始化 + start_task 编排
    ├── ros_bridge.cpp         # node/executor/timer 生命周期 + 域桥编排
    ├── imu_bridge.cpp         # IMU/温度发布（micro_ros/imu/data、temp）
    ├── can_debug_bridge.cpp   # CAN1 调试桥（send/receive）
    ├── motor_bridge.cpp       # 电机命令/反馈桥（/joint_commands、JointState）
    ├── sensor_task.cpp        # 传感器任务：EXTI/信号量/温度 PID/加热 PWM
    ├── control_task.cpp       # 控制任务：三路 FDCAN + 100Hz mailbox/状态机编排
    ├── supervisor_task.cpp    # 100ms 开发期 heartbeat 观察任务
    └── system_clock.cpp       # 覆盖 SystemClock_Config
```

## 硬件接线

| 信号 | MCU 引脚 |
|---|---|
| USB_DM / USB_DP | PA11 / PA12（直接 USB 线） |
| 状态灯 | PA7（WS2812，SPI6 MOSI） |
| SPI2_SCK / MOSI / MISO | PB13 / PC1 / PC2 |
| ACC_CS / GYRO_CS | PC0 / PC3 |
| POWER_5V | PC15（置高使能 IMU 供电） |
| ACC_INT / GYRO_INT | PE10 / PE12（上升沿 EXTI） |
| 加热 PWM | PB1（TIM3_CH4） |
| FDCAN1_RX / TX | PD0 / PD1 |
| FDCAN2_RX / TX | PB5 / PB6 |
| FDCAN3_RX / TX | PD12 / PD13 |
| 保留输出 | PA0（官方例程固定置低） |

## 构建与烧录

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 编译
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录
~/.platformio/penv/bin/pio device monitor # 串口监视（115200，USB CDC 下波特率无效）
```

自定义消息通过仓库内符号链接
`extra_packages/can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs`
复用同一份 `can_bridge_msgs` 定义；不要复制或另建第二份消息包。已缓存
`micro_ros_platformio` 和 STM32FreeRTOS 依赖时，`pio run -t clean && pio run`
可离线重编固件对象；全新机器或删除 micro-ROS 构建环境后的 clean bootstrap
仍需联网下载依赖/ROS 2 源，网络或 TLS 不可用时会失败。常规验证不运行
`clean_microros`，除非确实需要重建该下载环境。

## 与 micro_ros_agent 联调

```bash
source /opt/ros/humble/setup.bash
source ../micro_ros_agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

建链后发布：

- `micro_ros/imu/data`：`sensor_msgs/msg/Imu`（四元数、SI 角速度/线加速度）
- `micro_ros/imu/temp`：`std_msgs/msg/Float32`（BMI088 芯片温度）
- `/joint_commands`：订阅 `sensor_msgs/msg/JointState`；`name`/`position`/
  `velocity`/`effort` 对应默认测试电机；vaxor 模式严格校验：恰好只发
  `position`=位置模式、恰好只发 `velocity`=速度模式，其余情况（含仅
  `effort`、两者同发）保持 MIT；切换模式先“停机+切模式”再运行；
  kp/kd 用电机默认值
- `micro_ros/motor_enable`：订阅 `std_msgs/msg/Bool`，`false` 立即撤销授权并
  清空待处理命令；`true` 仅在 agent 已连接且无 bus/motor fault 时重新授权。
  授权恢复不会重放旧命令，必须在 `true` 之后发送一条新的 `/joint_commands`
  命令，RP 总线才会原子执行 enable + 新广播帧，vaxor 才会重新 run
- `micro_ros/JointState`：发布 `sensor_msgs/msg/JointState`（默认测试电机反馈）

宏 `ENABLE_CAN1_DEBUG_BRIDGE=1` 开启时，额外提供 FDCAN1 调试桥
（`can_bridge_msgs/msg/CANFrame`，与 `micro_ros_agent` 同一定义）：

- `micro_ros/can1/send`：订阅；合法 DLC 帧覆盖单槽 mailbox，control task 每
  10ms 最多发送一帧（latest-wins）。连续覆盖、拒绝和成功发送可通过
  `g_control_debug_{overwrite,rejected,sent}_count` 观察；断连丢弃未发送帧。
- `micro_ros/can1/receive`：发布；FDCAN1 中断收到的帧定时上报（CAN → ROS）

## 控制安全状态与恢复语义

- control task 使用 `vTaskDelayUntil` 保持 10ms（100Hz）绝对周期；每个关节命令
  各有一个 latest-wins slot，CAN debug TX 另有一个 latest-wins slot。ROS
  callback 不直接调用 FDCAN HAL，debug TX 也服从 control task 唯一发送者约束。
- 命令 age `<100ms` 时 active；`>=100ms` 进入 soft stop，以 MIT 零位置、零速度、
  零前馈和默认 `kd` 发送阻尼；`>=300ms` 使该命令失效。vaxor 随后发送 stop，
  RP 则只把超时电机自身的 8B broadcast slot 置 neutral，其他新鲜 slot 可继续运行。
- RP enable/disable 是总线级动作：bus fault、`motor_enable=false` 或 agent 断连会
  使该 RP 总线反复发送 disable；单个 RP motor fault/timeout 只隔离自己的 slot，
  不应误伤同总线其他 slot。清除故障后仍须重新授权并提供新命令，才会在同一
  control 周期按 enable + broadcast 顺序恢复。
- agent 已连接后若断连，所有旧命令立即失效、授权撤销、未发送 debug 帧丢弃；
  重连本身不恢复运动。先发布 `micro_ros/motor_enable=true`，再发布新命令。
- NaN/Inf、未知 joint、非法 vaxor mode 或非法 CAN DLC 被拒绝，不刷新命令 freshness。

上述是软件失效响应，不是硬件安全认证。电机/机构当前**无抱闸**，通信失效或
零力矩后仍可能因重力、惯性或外力运动；硬件急停、机械限位、供电隔离和真机
标定不可由本逻辑替代。

## CAN RX 与 IMU 诊断

- 三路 FDCAN ISR 使用 `xQueueSendFromISR` 写预分配静态队列；CAN1 容量 32，
  CAN2/3 各 8。队满时先 `xQueueReceiveFromISR` 丢最旧帧，再放入新帧。
  `bsp_can_init()` 的 single-init guard 保证重复入口不会重建或清空队列。
- control task 是三路主队列的 sole consumer，每个 10ms 周期每路最多 drain 8 帧，
  避免 RX burst 无界侵占控制周期；有剩余积压时递增
  `g_control_can_rx_backlog_count[3]`。GDB 还可通过
  `can_rx_get_diagnostics()` 查看每路 `rx_count`、`overflow_count`、
  `hal_receive_error_count`，并查看 `g_control_tx_{ok,error}_count` 与 debug
  mailbox 计数。control task 复制的 latest debug snapshot 仅供 ROS 发布。
- FDCAN IT0/IT1 抢占优先级固定为 5，且必须数值不小于
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，否则不可调用 FreeRTOS
  `FromISR` API；修改 NVIC/FreeRTOS priority 配置时必须同时验证该约束。
- BMI088 初始化使用 `imu_try_init()`：每次只尝试一次，失败后 sensor task
  进入降级态并每 1s 重试，不阻塞 control/micro-ROS。每个 SPI byte transfer
  最多阻塞 2ms，首次 HAL 失败后锁存 transport error，当前初始化/采样余下访问
  快速失败；失败时 heater PWM 立即置 0。初始化成功后，空的二值信号量保证
  第一份有效样本必须来自真实 ACC DRDY；sensor task 写双缓冲 snapshot，ROS
  只复制已发布缓冲。运行中 SPI 失败会标记 IMU unhealthy、关闭 heater，并在
  后续 DRDY 再尝试采样；`g_imu_init_error_count` 和 supervisor fault bit 可供 GDB
  观察。

## `dev-real` 无负载真机验证清单（仅人工执行）

以下项目**不得由自动化测试执行**。只能在 `dev-real`、机械无负载、硬件急停
已验证可用且人员可立即断电时逐项执行；不安装负载、不验证抱闸（当前无抱闸），
也不得据此宣称已达到硬件安全：

- [ ] MIT 小幅零附近命令连续运行，示波器/logic analyzer 验证 control TX 为
  100Hz，并记录 control cycle WCET、周期 jitter 与 supervisor heartbeat。
- [ ] 停止刷新单个命令，验证 100ms 边界进入默认 `kd` 阻尼、300ms 边界 hard
  stop；RP 仅 neutral 对应 slot，vaxor 发 stop，其他有效 RP slot 不受影响。
- [ ] Agent 断连/重连，验证断连即撤销授权、清旧命令/旧 debug TX；重连后必须
  依次发送 `motor_enable=true` 与新命令，期间不得自行恢复或重放。
- [ ] 发布 `motor_enable=false/true`，验证 RP 总线级 disable，以及只有新鲜命令
  到达后才执行 enable + broadcast；vaxor 同样不得复用旧命令。
- [ ] 注入 NaN/Inf、未知 joint、非法 mode/DLC，验证拒绝计数增加且 freshness
  不更新。
- [ ] CAN 无 ACK/拔除收发器，验证 TX error/backlog 可见，任务与 supervisor
  仍可观察；硬件急停作为最终保护，不依赖软件停机。
- [ ] 对 CAN1 注入超过 32 帧、CAN2/3 注入超过 8 帧的 RX burst，记录每路
  `rx/overflow/backlog`、control cycle WCET/jitter，确认 drop-oldest 与每周期
  每总线最多 8 帧的有界 drain；数据率必须覆盖预计极端负载。
- [ ] 同一 RP 总线配置多个 slot，分别制造单 slot timeout/fault 与 bus fault，
  验证前者仅 neutral 对应 slot、后者整总线 disable，恢复顺序无旧命令重放。
- [ ] 断开/恢复 BMI088、阻塞 SPI 或停止 DRDY，验证 2ms transport bound、每秒
  初始化重试、heater off、双缓冲不撕裂及 healthy/degraded heartbeat 阈值。

## 已知问题与后续

- DMA 仅保留官方配置与中断转发，BMI088 收发仍走阻塞
  `HAL_SPI_TransmitReceive`；若后续改为 DMA 传输，需在半完成回调中
  给 IMU 任务补一次醒信号。
- 无 agent 时节点不注册、固件在 `kWaitAgent` 轮询，属正常。
- `loop()` 被 STM32FreeRTOS 的 idle hook 调用，只能放非阻塞代码。
- 官方 `0x520` 测试帧循环保留在 `ENABLE_CANFD_DEMO_LOOP=1` 下，默认关闭；
  开启后总线无对端 ACK 时 TX FIFO 会积压（`fdcanx_send_data` 返回失败）。
- `control_task` 已接 JointState；默认仅 `left_shoulder_yaw_joint`（RP）与
  `left_wrist_yaw_joint`（vaxor，节点 ID=2，当前 CAN1 实际接入）两台测试
  电机（`sensor_msgs/JointState`），完整拓扑待参数服务。
- `lib/motor_protocol` 是协议编解码库（RP/25-30mm），由 `control_task`
  调用 pack/unpack 生成/解析 CAN 帧；当前 CAN1 已接入一台 vaxor。
- CAN1 调试桥 RX/TX 元素已扩到 64 字节，可承载 RP 64B 广播与 vaxor 12/20B
  帧；`CANFrame.id` 为 `uint16`，29 位扩展 ID 仅能表示低 16 位。
- RX burst 下的实际 control WCET/jitter、队列容量余量及 100/300ms 动作边界尚需
  按上述清单在 `dev-real` 无负载测量；host/固件编译不能替代真机时序证据。
