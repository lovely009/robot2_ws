# micro_ros_firmware

STM32H723VGT6 上的 **micro-ROS 固件**（PlatformIO + STM32duino + FreeRTOS），
通过 **serial over USB CDC**（PA11=USB_DM / PA12=USB_DP）与主机侧
`../micro_ros_agent/` 通信，桥接进 ROS 2 Humble 域。

当前业务为 **BMI088 IMU** 与 **三路 FDCAN**，底层功能移植自官方
`/home/ly/文档/dm-mc02-master/例程/CtrBoard-H7_IMU_TempCtrl`：
SPI2 阻塞传输、POWER_5V/CS/EXTI、TIM3 加热与温度 PID 均保留；
Mahony 姿态解算与 micro-ROS 发布为本工程增强，不属于官方例程内容。
三路 FDCAN（FDCAN1/2/3）移植自
`/home/ly/文档/dm-mc02-master/例程/CtrBoard-H7_FDCAN`：FD + BRS、标称
1Mbps / 数据 5Mbps、接收中断回调与周期测试帧发送均保留。
电机协议（vaxor 25/30mm + CERB CAN FD/MIT）已收敛为 `lib/motor_protocol`
adapter 接缝：厂家字节布局、CAN-ID 推导与 FD 属性只存在于该库，内核只见 SI
关节侧物理量。电机运行时为 **ID 驱动、发现式**（ARD-0007）：固件
不内置任何电机表或 `joint_name`，设备经反馈帧自主发现（vaxor 20ms 状态心跳 /
CERB SYNC 上报心跳），型号由上层经 Profile 绑定确认；命令是最多 64 目标的
ID 化完整快照。反馈模型（2026-09-05 与实际设备核实）：两协议反馈均分
"命令实时回显（100Hz）"与"20ms 周期状态心跳（承担总线检测）"两部分；
OFFLINE = 心跳丢失 100ms。上层迁移与台架验证进度见
`docs/handoffs/HANDOFF-motor-runtime-refactor.md`。

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
| IMU 传输 | **SPI2 阻塞收发，不使用 DMA** | 单次寄存器访问仅 8 Byte；每字节 HAL timeout 2ms，失败锁存至当前操作结束 |
| IMU 采样触发 | **ACC_INT=PE10 EXTI + FreeRTOS 二值信号量，ODR=100Hz** | 官方 TempCtrl 由加速度计 DRDY 唤醒；本工程从 800Hz 降至 100Hz（与 Mahony sampleFreq 对齐） |
| 加热 PWM | **TIM3_CH4=PB1**（500Hz），使用 Arduino `HardwareTimer` | 官方 PSC=24-1 / ARR=10000-1 语义 |
| 状态 LED | **PA7（WS2812，SPI6 MOSI 驱动）** | 板载 1 颗 RGB 灯珠 |
| CANFD 通信 | **FDCAN1=PD0/PD1、FDCAN2=PB5/PB6、FDCAN3=PD12/PD13**，FD + BRS，RX/TX 元素 64B | ISR 只盖时戳写每总线 SPSC RX ring（容量 64，满丢新帧并计数），不解码 |
| CANFD 时钟 | **PLL2Q = 80MHz**（标称 1Mbps / 数据 5Mbps） | 官方例程用 PLL1Q=80MHz；本工程 PLL1Q 已给 SPI2，故改用 PLL2Q |
| CANFD 构建 | `-DHAL_FDCAN_MODULE_ENABLED` | STM32duino 默认关闭 FDCAN HAL 模块 |
| W25Q64 Flash | OCTOSPI2 Port 1：PA1 / PA3 / PB0 / PB2 / PE11 / PD11，Quad page-program / Quad I/O read | `lib/w25q64` 保留，当前不执行启动探测 |
| 电机接口 | ID 化完整快照（5 个订阅 + 5 个发布，共 10 个话题） | 发现式运行时，无 `joint_name`/固定电机表（ARD-0007） |
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
| `start_task` | 1 | 256 | 创建 `micro_ros`/`sensor`/`motor_rt` 后自删除 |
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发，10ms bridge timer（100Hz）按域节拍发布：运动/IMU 数据 100Hz、状态 20Hz、拓扑 1Hz、温度 2Hz；断连自动重建 |
| `led` | 1 | 256 | 每 50ms 读取 micro-ROS 链路状态并作为 WS2812 唯一运行期写入者；红/绿闪/蓝 |
| `sensor` | 2 | 512 | 500ms 延时 → 加热 PWM → BMI088 单次初始化；失败时每 1s 降级重试，成功后由约 100Hz ACC DRDY 触发采样/Mahony/PID（IMU 非关键路径：低于 motor_rt，栈台架 TBD-RT-01 复测） |
| `motor_rt` | 3 | 1024 | FDCAN1/2/3 初始化；每 1ms 有界服务（RX drain/内核服务/SYNC 心跳/TX 路由），每 10ms 控制边界消费完整快照与配置 FIFO |

## 代码组织

业务按“大任务”分域，ROS 实体不堆在 `ros_bridge`：

| 模块 | 职责 |
|---|---|
| `ros_bridge` | agent 通信生命周期：node/executor/timer，断连整体重建；定时器依次轮询各域桥 |
| `imu_bridge` | `micro_ros/imu/data` 与 `micro_ros/imu/temp` 的缓冲/填充/发布 |
| `motor_bridge` | ID 化命令/Policy/Config 订阅 + motion/status/topology/events/config_result 发布 |
| `motor_rt_task` | 唯一电机实时任务：1ms 服务 + 10ms 控制边界，内核唯一调用者 |
| `lib/fdcan` | FDCAN 初始化/中断回调/每总线 SPSC RX ring/P0-P1-P2 TX 调度/硬件统计 |
| `lib/imu` | BMI088 驱动/通信层、SPI2 阻塞传输、Mahony/SI 聚合（单一 IMU 域） |
| `lib/motor_protocol` | adapter 接缝 + vaxor/CERB 编解码（厂家字节唯一归属地） |
| `lib/motor_control` | ID 驱动运行时内核：注册表/型号目录/九态生命周期/安全级联/配置事务 |
| `lib/control_mailbox` | ID 完整命令快照、低频配置 FIFO 与连接/授权边沿（纯逻辑） |
| `lib/motor_event_relay` | 内核事件中继环：任务侧搬入、桥轮询取走（满丢最旧并计数，host 可测） |
| `lib/dma` | DMA1/BDMA 控制器级资源（时钟、NVIC、占用登记）；设备配置由各设备库自持 |
| `lib/w25q64` | OCTOSPI2 W25Q64 驱动（保留，未接入启动路径） |
| `lib/ws2812` | 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7） |

数据流：FDCAN1/2/3 ISR → 每总线 SPSC ring → `motor_rt_task` 有界 drain 并经
adapter 解码进内核 → 内核按完整快照/会话/心跳裁决生命周期与运动 → 待发帧
按 P0/P1/P2 路由进每总线 TX 调度器 → 由任务层统一提交 HAL。
ROS 桥只通过 mailbox（写侧）与内核只读副本/事件队列（读侧）交互；
原始 CAN 帧不进入 ROS 域。IMU 则由 sensor task 写双缓冲 snapshot，
10ms bridge timer（100Hz）复制后发布；桥接层不持有 sensor task 的内部源指针。

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
│   ├── motor_bridge.h         # 电机域 ROS 桥接接口
│   ├── micro_ros_link_state.h # micro-ROS 链路状态类型
│   ├── micro_ros_task.h       # micro-ROS 任务接口
│   ├── led_task.h             # LED 任务接口 + host 可测的状态到颜色策略
│   ├── sensor_task.h          # 传感器任务接口（sensor_task_start）
│   └── motor_rt_task.h        # 电机实时任务接口（motor_rt_task_start）
├── lib/                       # 板级/协议库（PlatformIO 自动发现）
│   ├── imu/                   # BMI088 驱动 + SPI2 阻塞传输 + Mahony + SI 聚合
│   ├── fdcan/                 # 三路 FDCAN/中断回调/SPSC RX ring/P0-P1-P2 TX 调度
│   ├── motor_control/         # ID 驱动运行时内核（host 可测）
│   ├── motor_event_relay/     # 内核事件中继环（满丢最旧并计数，host 可测）
│   ├── control_mailbox/       # 固定容量 mailbox 与边沿锁存（host 可测）
│   ├── motor_protocol/        # adapter 接缝 + vaxor/CERB 编解码（纯逻辑）
│   ├── dma/                   # DMA1/BDMA 控制器级资源（设备配置由各库自持）
│   ├── w25q64/                # OCTOSPI2 W25Q64 驱动（保留，未接入启动路径）
│   └── ws2812/                # WS2812 彩灯驱动（SPI6 MOSI=PA7）
└── src/
    ├── main.cpp               # Arduino 入口与 start_task 编排
    ├── micro_ros_task.cpp     # micro-ROS agent 状态机
    ├── led_task.cpp           # WS2812 状态显示任务 + host 可测的颜色策略
    ├── ros_bridge.cpp         # node/executor/timer 生命周期 + 域桥编排
    ├── imu_bridge.cpp         # IMU/温度发布（micro_ros/imu/data、temp）
    ├── motor_bridge.cpp       # 电机 ID 化话题桥（订阅/发布，见"联调"节）
    ├── sensor_task.cpp        # 传感器任务：EXTI/信号量/温度 PID/加热 PWM
    ├── motor_rt_task.cpp      # 电机实时任务：1ms 服务 + 10ms 控制边界
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
| W25Q64 OCTOSPI2 Port 1 | PA1 / PA3 / PB0 / PB2 / PE11 / PD11 |
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

### W25Q64

`lib/w25q64` 驱动及其 geometry host test 保留，但当前固件不执行启动探测，
也不定义 W25Q64 启动开关或暴露启动诊断变量。后续运行期读写必须由明确的
专用任务/模块单独管理 OSPI 句柄。

## 与 micro_ros_agent 联调

```bash
source /opt/ros/humble/setup.bash
source ../micro_ros_agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

建链后可见电机话题（消息均属 `can_bridge_msgs`，源在
`../micro_ros_agent/src/can_bridge_msgs`）：

订阅（下行，1 个话题；命令为最多 64 目标的完整 ID 快照）：

- `/motor_command_array`（`MotorCommandArray`，best-effort）：`motor_id +
  position/velocity/effort` 完整快照；NaN/Inf、未知或重复 `motor_id` 整体拒绝。
  缺席的受控电机立即软停，禁止沿用旧目标。

下行配置通道（`motor_enable`/`motor_policy`/`motor_config`/`motor_bind` 与
`motor_config_result`）已删除：型号绑定由内核读设备自报身份自动完成
（2026-09-16 起，静态拓扑表已删）——发现建档/重连后经事务读 CERB SDO 0x1008
（厂家名称）并与型号目录整串精确匹配；vaxor 无身份读对象，回退到"协议内唯一
型号"。未命中目录的设备保持未绑定不可控。授权/增益/限位取内核安全默认（建档
`policy_enable` 默认 true、全局授权 mailbox 引导豁免、增益回退 profile
`default_kp/default_kd`、CERB 限位在驱动器侧配置）。

发布（上行，4 个话题；均为 ID 化，缺失浮点 = NaN）：

- `micro_ros/motion_states`（`MotorMotionStateArray`）：运动状态（100Hz，
  由 10ms bridge timer 驱动）。
- `micro_ros/motor_status_array`（`MotorStatusArray`）：诊断（生命周期/故障码/
  保护位/事务状态）。
- `micro_ros/motor_topology`（`MotorTopology`，1s 节流）：发现式拓扑
  （`motor_id`/总线/协议/型号/能力/冲突标志）。
- `micro_ros/motor_events`（`MotorEvent`）：生命周期/安全事件流（含身份识别
  的 PROFILE_BOUND / PROFILE_REJECTED）。

`motor_id ↔ joint_name` 的映射、方向与零位是**上层职责**（上层迁移进行中，
见 HANDOFF「剩余工作」）；固件不做 name-to-ID 适配。

## 控制安全状态与恢复语义

- `motor_rt_task` 每 1ms 服务、每 10ms（100Hz）控制边界消费一次完整快照；
  内核是唯一生命周期裁决者，任务层是唯一正常 FDCAN 发送者（P0/P1/P2 分级）。
- 快照 freshness（MCU 单调时基）：`>30ms` 无新快照 → 全体受控电机 MIT 默认
  `kd` 阻尼软停；`>100ms` → 协议级失能并清授权；Agent 断链立即失能清授权，
  不等 100ms。恢复必须 新授权 + 新完整快照，不复用旧目标。
- 快照内缺席的受控电机立即软停；`motor_id` 重复/未知/NaN 的快照整体拒绝。
- 授权由会话状态派生（motor_enable 话题已删）：Agent 在线即全局授权；断连立即
  撤权、30/100ms 快照超时软停/失能后，**重新发送新鲜完整快照即自动恢复**
  （任务层每控制边界按会话电平重立全局授权）。FAULT/OFFLINE/CONFIG_CONFLICT
  冻结态的逐电机授权不被全局重立复活——这三类恢复只能复位控制板。
- CERB 使能后等待至少 100ms（adapter 整定门）才发送运动目标；SET_MODE 事务
  结束后强制重走使能序列。厂家故障置位 → FAULT 禁运动 + P0 失能；清除 →
  READY_DISABLED 等待显式重授权。
- CONFIG_CONFLICT 锁存（同 `motor_id` 跨总线/协议身份不符）：禁运动、穿越
  OFFLINE/重连不自动解除，只能整机重启复位。

上述是软件失效响应，不是硬件安全认证。电机/机构当前**无抱闸**，通信失效或
零力矩后仍可能因重力、惯性或外力运动；硬件急停、机械限位、供电隔离和真机
标定不可由本逻辑替代。

## CAN RX 与 IMU 诊断

- 三路 FDCAN ISR 只盖 MCU 单调时戳并把原始帧写入每总线 SPSC ring（容量 64，
  head/tail 单调序号）；环满丢**新**帧并递增 `overflow_count`，ISR 不解码、
  不回收槽位。`bsp_can_init()` 的 single-init 语义保证重复入口不会重建。
- `motor_rt_task` 是唯一消费者：每 1ms 每总线最多 drain 16 帧（批量 8），经
  adapter 匹配/解码进内核；未知帧与解码失败分别计数可见。原始 CAN 帧不通过
  ROS 发布；电机反馈/状态/拓扑只由内核解码后以只读副本输出。TX 侧每总线
  P0/P1/P2 调度器自带 dropped/replaced/hw_reject 计数。
- FDCAN IT0/IT1 抢占优先级固定为 5，且必须数值不小于
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`；修改 NVIC/FreeRTOS priority
  配置时必须同时验证该约束。错误状态（EW/EP/BUS_OFF）目前仅计数，
  恢复策略待台架 T12 数据。
- BMI088 初始化使用 `imu_try_init()`：每次只尝试一次，失败后 sensor task
  进入降级态并每 1s 重试，不阻塞 control/micro-ROS。每个 SPI byte transfer
  最多阻塞 2ms，首次 HAL 失败后锁存 transport error，当前初始化/采样余下访问
  快速失败；失败时 heater PWM 立即置 0。初始化成功后，空的二值信号量保证
  第一份有效样本必须来自真实 ACC DRDY；sensor task 写双缓冲 snapshot，ROS
  只复制已发布缓冲。运行中 SPI 失败会标记 IMU unhealthy、关闭 heater，并在
  后续 DRDY 再尝试采样；`g_imu_init_error_count` 可供 GDB 观察。

## `dev-real` 无负载真机验证清单（仅人工执行）

以下项目**不得由自动化测试执行**。只能在 `dev-real`、机械无负载、硬件急停
已验证可用且人员可立即断电时逐项执行；不安装负载、不验证抱闸（当前无抱闸），
也不得据此宣称已达到硬件安全：

- [ ] MIT 小幅零附近命令连续运行，示波器/logic analyzer 验证 motor_rt TX 为
  100Hz，并记录 motor_rt cycle WCET 与周期 jitter。
- [ ] 停止刷新快照，验证 30ms 边界进入默认 `kd` 阻尼软停、100ms 边界协议级
  失能并清授权；单电机从快照缺席，验证仅该电机软停、其他电机不受影响。
- [ ] Agent 断连/重连，验证断连立即失能清授权、清旧命令；重连后内核重新读
  设备自报身份（CERB SDO 0x1008）并重绑定收敛 READY_DISABLED，恢复运动必须
  复位控制板（编译期授权语义）+ 新完整快照，期间不得自行恢复或重放。
- [ ] 身份识别台架核对：`ros2 topic echo micro_ros/motor_topology` 中各 CERB
  节点的 `model_name` 应与铭牌一致（R48/R58 不得互相误判）；拔掉某台设备后
  该行消失，重新上电后被重新识别；目录外型号应停留在未绑定并出
  PROFILE_REJECTED 事件。
- [ ] 验证 OFFLINE/厂家故障冻结的逐电机授权不会因重连自动复立（须复位）；
  CERB 重新使能后必须等待至少 100ms 才发送运动目标。
- [ ] 注入 NaN/Inf、未知/重复 motor_id、非法 mode，验证快照整体拒绝、拒绝
  计数增加且 freshness 不更新。
- [ ] CAN 无 ACK/拔除收发器，验证 hw_reject/backlog 计数可见；硬件急停作为
  最终保护，不依赖软件停机。
- [ ] 对三总线注入超 64 帧 RX burst，记录每路 `rx/overflow`、motor_rt cycle
  WCET/jitter，确认丢新帧语义与每周期每总线 16 帧的有界 drain；数据率必须
  覆盖预计极端负载（约 40 台）。
- [ ] 同一总线混挂 vaxor/CERB，分别制造单电机心跳丢失/厂家故障与 bus fault，
  验证故障隔离、P0 失能帧和恢复顺序均无旧命令重放；验证 CONFIG_CONFLICT
  锁存不被离线/重连清除。
- [ ] 示波器或 candump 验证 CERB 所在总线每 10ms 仅一帧 `0x080` SYNC，设备
  20ms 返回上报帧；确认上报密度与 100ms OFFLINE 边界匹配实际设备行为。
- [ ] 断开/恢复 BMI088、阻塞 SPI 或停止 DRDY，验证 2ms transport bound、每秒
  初始化重试、heater off 与双缓冲不撕裂。

## 已知问题与后续

- BMI088 收发固定走阻塞 `HAL_SPI_TransmitReceive`，当前不占用 DMA stream；
  若后续切换 DMA，设备句柄/回调仍归 `lib/imu`，控制器资源须向 `lib/dma` 登记。
- 无 agent 时节点不注册、固件在 `kWaitAgent` 轮询，属正常。
- `loop()` 被 STM32FreeRTOS 的 idle hook 调用，只能放非阻塞代码。
- 官方 `0x520` 测试帧循环保留在 `ENABLE_CANFD_DEMO_LOOP=1` 下，默认关闭；
  开启后总线无对端 ACK 时 TX FIFO 会积压（`fdcanx_send_data` 返回失败）。
- 电机运行时已完成 ID 驱动化（ARD-0007，剩余项见 HANDOFF「剩余工作」）；13 个
  `can_bridge_msgs` 消息由 `../micro_ros_agent/src/can_bridge_msgs` 唯一维护，
  固件侧只通过 `extra_packages/can_bridge_msgs` 符号链接复用。
- CERB R48/R58 的 MIT 范围配置为位置 `±π rad`、速度 `±π rad/s`、
  力矩 `±11/±45 Nm`，必须与真机参数设置页一致；`motor_rt_task` 每 10ms 发送
  `0x080` SYNC 维持设备 20ms 状态心跳（总线检测）。
- RX burst 下的实际 motor_rt WCET/jitter、ring 容量余量及 30/100ms 动作边界
  尚需按上述清单在 `dev-real` 无负载测量；host/固件编译不能替代真机时序证据。
