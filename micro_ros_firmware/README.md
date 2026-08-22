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
| IMU DMA | **DMA1_Stream0(RX) / Stream1(TX)** 保留配置并链接 | 官方生成代码同构；收发仍走阻塞 HAL，与官方行为一致 |
| IMU 采样触发 | **ACC_INT=PE10 EXTI + FreeRTOS 二值信号量，ODR=200Hz** | 官方 TempCtrl 由加速度计 DRDY 唤醒；本工程从 800Hz 降至 200Hz |
| 加热 PWM | **TIM3_CH4=PB1**（500Hz），使用 Arduino `HardwareTimer` | 官方 PSC=24-1 / ARR=10000-1 语义 |
| 状态 LED | **PA7（WS2812，SPI6 MOSI 驱动）** | 板载 1 颗 RGB 灯珠 |
| CANFD 通信 | **FDCAN1=PD0/PD1、FDCAN2=PB5/PB6、FDCAN3=PD12/PD13**，FD + BRS，RX/TX 元素 64B | 官方同源；64B 为承载 RP 广播与 vaxor 命令/反馈 |
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
| `start_task` | 1 | 256 | 创建 `micro_ros`/`sensor`/`control` 后自删除 |
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发，断连自动重建；状态灯红/绿闪/蓝 |
| `sensor` | 4 | 1024 | 500ms 延时 → 加热 PWM → BMI088 初始化 → EXTI 信号量 → 读取/Mahony/PID；后续传感器扩展点 |
| `control` | 3 | 1024 | 500ms 延时 → FDCAN1/2/3 初始化；JointState 编码下发；官方 0x520 测试帧默认关闭（宏开启） |

## 代码组织

业务按“大任务”分域，ROS 实体不堆在 `ros_bridge`：

| 模块 | 职责 |
|---|---|
| `ros_bridge` | agent 通信生命周期：node/executor/timer，断连整体重建；定时器依次轮询各域桥 |
| `imu_bridge` | `micro_ros/imu/data` 与 `micro_ros/imu/temp` 的缓冲/填充/发布 |
| `can_debug_bridge` | `micro_ros/can1/send` 订阅与 `micro_ros/can1/receive` 发布（宏控制） |
| `motor_bridge` | `/joint_commands` 订阅展开 + `micro_ros/JointState` 反馈发布（宏控制） |
| `control_task` | 电机表、RP/vaxor 协议收发与反馈解码，不接触 ROS |
| `lib/fdcan` | FDCAN 初始化/收发/中断回调，通过 `can1_rx_snapshot()` 对外提供最新帧 |
| `lib/imu` | BMI088 驱动/通信层/Mahony/SI 聚合（依赖 `lib/dma`） |
| `lib/motor_protocol` | RP/vaxor 协议编解码（纯逻辑） |
| `lib/ws2812` | 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7） |
| `lib/dma` | 板级 DMA 初始化（当前 SPI2 专用）与中断转发 |

数据流：FDCAN1 中断写快照 → `ros_bridge` 定时器轮询各域桥 → 域桥经
`can1_rx_snapshot()` 取帧并发布/解码；ROS 命令经域桥回调下发给 `control_task`。

## 目录结构

```
micro_ros_firmware/
├── platformio.ini             # arduino/humble/serial+USB CDC；FDCAN HAL 构建宏
├── boards/
│   └── stm32h723vgt6.json     # 自定义板卡定义（变体/宏/USB VID-PID/烧录协议）
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
│   ├── fdcan/                 # 三路 FDCAN 初始化/收发/中断回调/接收快照
│   ├── motor_protocol/        # RP/vaxor 协议编解码（纯逻辑）
│   └── ws2812/                # WS2812 彩灯驱动（SPI6 MOSI=PA7）
└── src/
    ├── main.cpp               # 入口：板级初始化 + start_task 编排
    ├── ros_bridge.cpp         # node/executor/timer 生命周期 + 域桥编排
    ├── imu_bridge.cpp         # IMU/温度发布（micro_ros/imu/data、temp）
    ├── can_debug_bridge.cpp   # CAN1 调试桥（send/receive）
    ├── motor_bridge.cpp       # 电机命令/反馈桥（/joint_commands、JointState）
    ├── sensor_task.cpp        # 传感器任务：EXTI/信号量/温度 PID/加热 PWM
    ├── control_task.cpp       # 控制任务：初始化三路 FDCAN + 周期测试帧
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
  `velocity`/`effort` 对应默认测试电机；vaxor 模式由字段决定：只发 `position`
  =位置模式、只发 `velocity`=速度模式、只发 `effort`=MIT 前馈，切换模式自动
  先停机再启动；kp/kd 用电机默认值，命令到达自动使能
- `micro_ros/JointState`：发布 `sensor_msgs/msg/JointState`（默认测试电机反馈）

宏 `ENABLE_CAN1_DEBUG_BRIDGE=1` 开启时，额外提供 FDCAN1 调试桥
（`can_bridge_msgs/msg/CANFrame`，与 `micro_ros_agent` 同一定义）：

- `micro_ros/can1/send`：订阅；收到一帧即经 FDCAN1 发出（ROS → CAN）
- `micro_ros/can1/receive`：发布；FDCAN1 中断收到的帧定时上报（CAN → ROS）

## 已知问题与后续

- IMU 任务在调度器启动后无条件运行；BMI088 未上电/未接线时会阻塞重试
  初始化，`imu_init()` 返回前不发布有效数据。
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
