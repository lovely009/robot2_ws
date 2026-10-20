# micro_ros_firmware

STM32H723VGT6 上的 **micro-ROS 固件**（PlatformIO + STM32duino + FreeRTOS），
通过 **serial over USB CDC**（PA11=USB_DM / PA12=USB_DP）与主机侧
`../micro_ros_agent/` 通信，桥接进 ROS 2 Humble 域。

当前业务为 **BMI088 IMU** 与 **三路 FDCAN**，底层功能移植自达妙官方例程
（CtrBoard-H7 的 `IMU_TempCtrl` 与 `FDCAN` 两例；例程整树不入库）：SPI2 阻塞传输、
POWER_5V/CS/EXTI、TIM3 加热与温度 PID 均保留；Mahony 姿态解算与 micro-ROS 发布为
本工程增强，不属于官方例程内容。三路 FDCAN（FDCAN1/2/3）的 FD + BRS、接收中断回调
同样来自例程；官方例程的周期测试帧未移植。
电机协议（vaxor 25/30mm + CERB CAN FD/MIT）已收敛为 `lib/motor_protocol`
adapter 接缝：厂家字节布局、CAN-ID 推导与 FD 属性只存在于该库，内核只见 SI
关节侧物理量。电机运行时为 **ID 驱动、发现式**（ARD-0007）：固件
不内置任何电机表或 `joint_name`，设备经反馈帧自主发现（vaxor 20ms 状态心跳 /
CERB SYNC 上报心跳），型号由内核读设备 SDO 0x1008 自报身份、查内置型号目录
自动绑定；命令是最多 64 目标的
ID 化完整快照。反馈模型（2026-09-05 与实际设备核实）：两协议反馈均分
"命令实时回显（500Hz，跟随控制边界）"与"20ms 周期状态心跳（承担总线检测）"两部分；
OFFLINE = 心跳丢失 100ms。**开放项、阶段门槛与「什么明确不做」见本文件末尾的
「开放项与开发计划」一节。**

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
| W25Q64 Flash | OCTOSPI2 Port 1：PA1 / PA3 / PB0 / PB2 / PE11 / PD11，Quad page-program / Quad I/O read | `storage_task` 独占：双槽（每槽 2 扇区）A/B 参数持久化（见「W25Q64」节） |
| 电机接口 | ID 化有界消息（5 订 10 发） | 发现式运行时，无 `joint_name`/固定电机表（ARD-0007） |
| 板卡变体 | `STM32H7xx/H723V(E-G)(H-T)_H730VB(H-T)_H733VG(H-T)` + `variant_generic.h` | STM32duino 官方 GENERIC_H723VGTX 变体 |

## 领域术语与所有权

固件拥有设备发现、电机安全与执行、CAN 传输、IMU 采集与设备参数持久化；ROS 桥只做
应用边界的消息转换，**不拥有硬件状态**。

| 术语 | 含义与所有者 |
|---|---|
| CAN TX 路由 | `lib/motor_tx_router`：挑有界 P0/P1/P2 工作、核算物理帧预算、调用注入的 submit 端口。起手有 P0 的泵只处理 P0，首次被拒即停止提交并把该帧与未尝试的 P0 按序放回；P1/P2 只在不带 P0 的泵上跑。`motor_rt_task` 持有路由器实例，是唯一正常 HAL 提交者 |
| P0 / P1 / P2 | 调度类别：安全动作 / latest-wins 控制 / 发现与配置。协议无关队列归 `lib/fdcan`，出队与物理提交策略归路由器 |
| P1 聚合 | `lib/motor_protocol` 适配器行为：CERB 合并相邻兼容的 MIT 命令，Vaxor 与不兼容或单条输入直通。调度器不知道协议布局 |
| 设备 SAVE | 请求驱动器把脏的、可保存的参数字段落到它自己的 flash；`lib/save_transaction` 等到关联结果才继续 |
| W25 SAVE | 固件参数投影落到外部 W25Q64。**只有 `storage_task` 访问 flash**，且只在设备保存阶段成功后进行 |
| 脏槽位 | 设备侧活值与本机持久化记录不一致的槽位；脏状态来自运行时快照，由事务协调器过滤 |

**变更约束**

- 协议字节与 CAN ID 留 `lib/motor_protocol`；队列存储与 P0/P1/P2 类别机制留 `lib/fdcan`；
  两者的有界连接留 `lib/motor_tx_router`。SAVE 时序留在纯 C 事务模块，ROS 与 storage
  回调留 `src/param_bridge.cpp`。
- 新增协议适配器**只能追加**到注册表：注册顺序同时是拓扑快照里的数字协议 ID，改序会移动既有条目。
- mailbox 的配置类别枚举与协议配置类别枚举保持**不同类型**（共享数值由
  `include/motor_config_contract.hpp` 穷举校验，纯协议值不进该映射）。
- 保持单写者所有权：电机运行时、正常 CAN TX、W25Q64 各只有一个所有者。

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
| `start_task` | 1 | 256 | 创建 `led`/`micro_ros`/`sensor`/`storage`/`motor_rt` 五任务后自删除 |
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发，10ms bridge timer（100Hz）按域节拍发布：运动状态 50Hz、IMU 数据 100Hz、状态 20Hz、拓扑 1Hz、温度 2Hz；断连自动重建 |
| `led` | 1 | 256 | 每 50ms 读取 micro-ROS 链路状态并作为 WS2812 唯一运行期写入者；红/绿闪/蓝 |
| `sensor` | 2 | 512 | 500ms 延时 → 加热 PWM → BMI088 单次初始化；失败时每 1s 降级重试，成功后由约 100Hz ACC DRDY 触发采样/Mahony/PID（IMU 非关键路径：低于 motor_rt，栈台架 TBD-RT-01 复测） |
| `storage` | 1 | 1024 | W25Q64 唯一所有者：W25 参数持久化（boot 顺序整读双槽 blob，每槽 2 扇区；只在收到 `SystemCommand` 的 `COMMAND_SAVE` 时落盘——写事务只置脏，没有自动落盘，见「W25Q64」节） |
| `motor_rt` | 3 | 1024 | FDCAN1/2/3 初始化；每 1ms 有界服务（RX drain/内核服务/SYNC 心跳/TX 路由），每 2ms/500Hz 控制边界消费完整快照、已拼接轨迹与配置 FIFO |

## 代码组织

业务按“大任务”分域，ROS 实体不堆在 `ros_bridge`：
模块依赖、执行上下文、数据所有权和测试边界的唯一索引见
`docs/architecture/module-boundaries.md`。

| 模块 | 职责 |
|---|---|
| `ros_bridge` | agent 通信生命周期：node/executor/timer，断连整体重建；定时器依次轮询各域桥 |
| `imu_bridge` | `micro_ros_up/imu/data` 与 `micro_ros_up/imu/temp` 的缓冲/填充/发布 |
| `motor_bridge` | ID 化下行订阅（low_cmd/trajectory/policy）+ 上行发布（motion/status/topology/events/trajectory_state/diagnostics/device_param_report） |
| `param_bridge` | 参数面：下行 `config` 一条订阅（`kind` 分参数写/系统命令）+ 上行 `system_status`；参数写是异步事务，成败经 `last_error` 与完成序号呈现 |
| `motor_rt_task` | 唯一电机实时任务：1ms 服务 + 2ms/500Hz 控制边界，内核唯一调用者（消费快照/轨迹/策略/配置） |
| `lib/session_sync` | 会话边沿/电平→内核授权的纯派生（500Hz 边界调用，host 可测） |
| `lib/storage_projection` | 内核参数视图↔W25 blob 纯投影（持久化字段语义，host 可测） |
| `heater` | TIM3 加热 PWM 所有权 + 纯 PID（host 可测） |
| `lib/fdcan` | FDCAN 初始化/中断回调/每总线 SPSC RX ring/P0-P1-P2 TX 调度 |
| `lib/motor_tx_router` | 有界 P0/P1/P2 出队、物理帧预算、P0 重试和 HAL 提交回调；不含厂商帧布局 |
| `lib/imu` | BMI088 驱动/通信层、SPI2 阻塞传输、Mahony/SI 聚合（单一 IMU 域） |
| `lib/motor_protocol` | 唯一 adapter 注册表、vaxor/CERB 编解码和协议自有 P1 聚合（厂家字节唯一归属地） |
| `lib/motor_control` | ID 驱动运行时内核：注册表/型号目录/九态生命周期/安全级联/配置事务 |
| `lib/control_mailbox` | ID 完整命令快照、多点轨迹、逐电机策略、低频配置 FIFO 与连接/授权边沿（纯逻辑） |
| `lib/save_transaction` | 设备 Flash 保存与 W25 持久化的异步顺序、request ID 关联及超时（纯 C） |
| `lib/board` | 板级共享常量（ISR syscall 阈值），板级库经它免依赖 FreeRTOS |
| `lib/dma` | DMA1/BDMA 控制器级资源（时钟、NVIC、占用登记）；设备配置由各设备库自持 |
| `lib/w25q64` | OCTOSPI2 W25Q64 驱动；唯一所有者为 `storage_task`（见「W25Q64」节） |
| `lib/ws2812` | 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7） |

数据流：FDCAN1/2/3 ISR → 每总线 SPSC ring → `motor_rt_task` 有界 drain 并经
adapter 解码进内核 → 内核按完整快照/会话/心跳裁决生命周期与运动 → 待发帧
按 P0/P1/P2 放入 `lib/fdcan` 队列 → `motor_tx_router` 核算逻辑/物理预算并选择
优先级 → `motor_protocol` adapter 可将兼容 CERB MIT P1 帧聚合 → 由
`motor_rt_task` 唯一提交 HAL。P0 队列非空时路由器只出队 P0；首个硬件拒收会
停止本轮提交，并把该帧及后续未尝试的 P0 按 FIFO 顺序放回队首。P0 清空后才
在后续 pump 开始时确认 P0 队列为空后，才调度 P1/P2；P1 保持 latest-wins，
P2 配额由路由器按每次 pump 管理。
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
├── include/                   # 应用接口与跨库契约头
│   ├── motor_config_contract.hpp # mailbox/protocol 配置 kind 数值契约
│   ├── ros_bridge.h           # micro-ROS 生命周期与域桥编排接口
│   ├── imu_bridge.h           # IMU/温度域 ROS 桥接接口
│   ├── motor_bridge.h         # 电机域 ROS 桥接接口
│   ├── micro_ros_link_state.h # micro-ROS 链路状态类型
│   ├── micro_ros_task.h       # micro-ROS 任务接口
│   ├── led_task.h             # LED 任务接口 + host 可测的状态到颜色策略
│   ├── sensor_task.h          # 传感器任务接口（sensor_task_start）
│   ├── storage_task.h         # W25Q64 参数持久化任务接口（storage_task_start）
│   ├── param_bridge.h         # 参数与系统命令话题桥接口
│   └── motor_rt_task.h        # 电机实时任务接口（motor_rt_task_start）
├── lib/                       # 板级/协议库（PlatformIO 自动发现）
│   ├── imu/                   # BMI088 驱动 + SPI2 阻塞传输 + Mahony + SI 聚合
│   ├── fdcan/                 # 三路 FDCAN/中断回调/SPSC RX ring/P0-P1-P2 TX 调度
│   ├── motor_tx_router/       # host 可测的 P0/P1/P2 预算、聚合分发与 HAL 提交 seam
│   ├── motor_control/         # ID 驱动运行时内核（host 可测）
│   ├── control_mailbox/       # 固定容量 mailbox 与边沿锁存（host 可测）
│   ├── motor_protocol/        # 唯一 adapter 注册表 + vaxor/CERB 编解码与 P1 聚合
│   ├── save_transaction/      # host 可测的设备保存 → W25 异步事务协调器
│   ├── dma/                   # DMA1/BDMA 控制器级资源（设备配置由各库自持）
│   ├── w25q64/                # OCTOSPI2 W25Q64 驱动（storage_task 独占访问）
│   └── ws2812/                # WS2812 彩灯驱动（SPI6 MOSI=PA7）
└── src/
    ├── main.cpp               # Arduino 入口与 start_task 编排
    ├── micro_ros_task.cpp     # micro-ROS agent 状态机
    ├── led_task.cpp           # WS2812 状态显示任务 + host 可测的颜色策略
    ├── ros_bridge.cpp         # node/executor/timer 生命周期 + 域桥编排
    ├── imu_bridge.cpp         # IMU/温度发布（micro_ros_up/imu/data、temp）
    ├── motor_bridge.cpp       # 电机 ID 化话题桥（订阅/发布，见"联调"节）
    ├── param_bridge.cpp       # 参数与系统命令话题桥（稀疏写 + 完成序号）
    ├── sensor_task.cpp        # 传感器任务：EXTI/信号量/温度 PID/加热 PWM
    ├── storage_task.cpp       # W25Q64 唯一所有者：boot 加载双槽 blob（每槽 2 扇区）+ save 落盘
    ├── motor_rt_task.cpp      # 电机实时任务：1ms 服务 + 2ms/500Hz 控制边界
    └── system_clock.cpp       # 覆盖 SystemClock_Config
```

## C/C++ 格式化

C/C++ 使用 Uncrustify 0.72.0，规则集中在 `.uncrustify.cfg`；UTF-8、LF、缩进和换行
由 `.editorconfig` 统一；Python 使用 4 空格缩进。检查 C/C++ 格式执行
`tools/format_firmware.sh --check`，
有意重排源码时执行 `tools/format_firmware.sh --write`。脚本只处理固件的
`src/`、`include/`、`lib/` 与 `test/host/` C/C++ 文件。

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

### 台架编译开关

`platformio.ini` 的 `build_flags` 里有一个默认关闭、只给模拟台架用的开关。**接真实电机
或量产前必须确认它关着**；改宏后须 `pio run -t clean` 全量重建。

| 宏 | 作用 | 代价 / 何时用 |
|---|---|---|
| `FDCAN_SELF_ACK_BENCH` | 三路 FDCAN 走 EXTERNAL_LOOPBACK，空总线也能自发自收供嗅探 | MON 位使本节点**不应答**总线上其他节点，接真实电机/完整台架前必须关 |

**FDCAN 位时序是冻结的标定值，不提供台架变体**：仲裁 1 Mbps（N=1/SJW=16/Seg1=63/Seg2=16
→ 80 tq，采样点 80%）、数据 5 Mbps（N=1/SJW=2/Seg1=12/Seg2=3 → 16 tq，采样点 81.25%），
定义在 `lib/fdcan/src/fdcan.c`，每次编译会打印一行
`FDCAN bit timing frozen: arb 1.0 Mbps (SP 80.0%), data 5.0 Mbps (SP 81.25%)`。板子的
81.25% 与真机电机的 80% 是匹配的，**改板子会把真机链路做坏**。2026-10-14 曾加过
`-DFDCAN_DATA_2M_BENCH` 把台架数据段降到 2 Mbps 来绕开总线间歇静默，当日已删除——台架
余量问题只能在适配器侧修（见 `docs/guides/fdcan1-bench-guide.md` 的"数据相位"一节）。

**启动顺序是硬要求**：板子必须在 ACK 节点（主机模拟器）已在场时才复位。板子若在无 ACK
节点的总线上启动，SYNC 拿不到 ACK → `AutoRetransmission` 重传把三路 TX FIFO 全部填满
（诊断里 `tx_frame_count == 16`，恰好是 FIFO 深度）→ 此后发不出任何帧，**再复位也好不了**。

自定义消息通过仓库内符号链接
`extra_packages/can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs`
复用同一份 `can_bridge_msgs` 定义；不要复制或另建第二份消息包。已缓存
`micro_ros_platformio` 和 STM32FreeRTOS 依赖时，`pio run -t clean && pio run`
可离线重编固件对象；全新机器或删除 micro-ROS 构建环境后的 clean bootstrap
仍需联网下载依赖/ROS 2 源，网络或 TLS 不可用时会失败。常规验证不运行
`clean_microros`，除非确实需要重建该下载环境。

### W25Q64

`storage_task`（P1/栈 1024 words）是 W25Q64 的唯一所有者。boot 同步
`w25q64_init()` 并**顺序整读两个槽**（槽 A `0x0000–0x1FFF`、槽 B `0x2000–0x3FFF`，
每槽 2 个 4 KB 扇区）：每块整读后过 `param_store_blob_validate()`（CRC 覆盖
头+记录），选 CRC 有效且 `seq` 最新者；胜者是先读的槽时重读一次再校验。
**刻意不做「只读 32B 头选胜者」的捷径**——那会拿未经校验的 `seq` 做裁决，
头损坏时可能选错槽。无芯片/ID 不符静默降级为无持久化，不阻断启动。
blob v3 = 32B 头 + 64×120B 记录（7712B）；v2（48B 记录）与 v1（32B 记录）
加载时各自就地迁移到 v3（两条 fan-in 路径，**不是** v1→v2→v3 链）。
保存**只有一个触发源**：`micro_ros_do/config` 的 `kind=SYSTEM_COMMAND`、`COMMAND_SAVE`。
设备写事务成功后只置脏（`record_dirty`），没有任何自动落盘——2026-09-21 起
连那 500ms 去抖也删掉了（它会在控制边界里偷偷擦 flash）。`COMMAND_SAVE` 先分批
下发设备保存请求并等待每台 `CONFIG_RESULT=OK`，然后才擦写非活动槽的**两个**扇区、
写 7712B，按 256B 页读回 memcmp 校验；任一失败不报告保存成功，W25 失败也不翻转活动槽
（旧槽仍可启动）。设备侧保存和 W25 之间没有跨介质原子提交，W25 失败后的恢复是重试
保存，不能宣称已回滚设备侧。

**升级迁移（一次性，含已知缺口）**：旧布局槽 B 基址是 `0x1000`，新布局下它
落进槽 A 的擦除范围；boot **只在两个新槽都无效**时兜底读一次 `0x1000`，把旧
板子的参数迁过来。**缺口（刻意不修）**：若旧槽 A 有效、而旧槽 B 持有更新的
一代，新版会按旧槽 A 启动，即**静默回退一代**。反向亦然——新固件 save 过一次
后两槽都是 v3，旧固件会判两槽皆无效并从零开始，**回滚到旧固件会丢参数**。

**擦写阻塞**：v2 单扇区实测 **~1s**（**旧值**）；v3 save 改擦 **2** 个扇区，
阻塞时间约翻倍，**双扇区实测值待台架**（见 validation plan）。擦写只在
storage 任务内执行，IWDG 由 motor_rt 独立喂。

## 与 micro_ros_agent 联调

```bash
source /opt/ros/humble/setup.bash
source ../micro_ros_agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

> 下行命令/轨迹需 `micro_ros_bridge` 在跑（`../micro_ros_agent/tools/bench_up.sh` 会起；
> 用 `robot.launch.py` 时 L1 已起，勿重复）。

建链后可见电机话题（消息均属 `can_bridge_msgs`，源在
`../micro_ros_agent/src/can_bridge_msgs`）：

订阅（下行，4 个话题；线上契约见 `WireContract.msg`）：

- `/micro_ros_do/low_cmd`（`LowCommand`，best-effort）：100 Hz 完整命令快照，
  位置/速度/力矩是 int16 定点（量程见 `WireContract`）；未列出的电机 = 软停。
- `/micro_ros_do/config`（`ConfigRequest`，reliable）：参数写与系统命令合一，
  `kind` 区分 `PARAM_SET` / `SYSTEM_COMMAND`，两组字段互斥。
- `/micro_ros_do/policy`（`MotorPolicyArray`，reliable）：逐电机 enable/kp/kd。
- `/micro_ros_do/trajectory`（`MotorTrajectory`，reliable）：多点轨迹。

上行仍是旧话题（名称与节拍见本节后面的「发布（上行）」清单）；S4 切流后本节与上行表一起重写。

`low_cmd` 的 NaN/Inf 在主机编码端（`micro_ros_bridge`）整帧拒绝，线上只带 −32768 哨兵；
未知或重复 `motor_id` 由固件整体拒绝。缺席的受控电机立即软停，
禁止沿用旧目标。**快照到达速率不决定命令速率**：受理后的目标由内核在每个
2ms/500Hz 控制边界保持重发（零阶保持，仅在快照新鲜 ≤30ms 期间），所以上行即使
按 50Hz 发，驱动器侧收到的目标为 500Hz 命令流。上行送达率实测封顶 ~72/s
（best-effort 静默丢弃），因此**不要**靠提高快照频率去提高命令率——那是无效的。

### 多点轨迹（`/micro_ros_do/trajectory`）

轨迹是完整快照的**时序补充**通道，不是替代品：快照给"现在在哪"，轨迹给
"未来一段时间怎么走"。同一 `trajectory_id` 可拆成多个可靠分段，分段带有
`segment_index`/`segment_count`；固件按序在固定缓冲区内拼接，只有收到完整组后才
原子替换当前轨迹，分段未收齐时旧轨迹继续运行。执行时每个 2ms/500Hz 边界使用
完整非均匀时间线做**单调三次 Hermite**插值（Fritsch–Carlson 切矢），段边界
速度由左右不等时间间隔共同传递，位置和解析速度连续；重叠边界的完全相同路点
只保留一次，冲突路点整组拒收；没有重叠时刻也按整条绝对时间线直接拼接，不把
每个分段端点错误地当成零速端点。

CAN 命令**每台每个边界恰好一条（500Hz）**；CERB 同总线最多 6 条命令合成一帧
`0x210`（FD+BRS、64B），VAXOR 保持单轴帧。路点里的 `velocity` 不参与执行——上层
常把它填 0，直通会让 MIT 律的 Kd 朝 0 速刹车，该字段只作诊断；前馈 `effort`
段内保持左端点值。时间超出末点即**钳末点保持，绝不外推**。轨迹起点是 MCU
本地受理完整轨迹的 tick，消息里的 `sync_time_ns` 只用于跨系统日志对齐（ARD §12）。

- **归属仲裁**：`target_ids` 声明的电机在轨迹在播期间由轨迹**独占**——同一
  时刻到达的 100Hz 快照对它们既不下发也不软停（否则快照会立刻打死轨迹）。
  集合外的电机仍由快照负责，"缺席即软停"语义不变。
- **解除归属只有三条显式路径**：轨迹被新轨迹替换（移出集合的电机立即软停）、
  发送**空轨迹**取消、会话断链（整机撤权同时作废轨迹）。快照**不是**解除路径。
- **与 30/100ms 快照新鲜度**：级联只作用于快照通道的电机——轨迹自带时长，
  "多久没收到快照"不是它的失效判据。轨迹电机仍受会话断链（立即全局撤权）与
  逐台故障/离线/冲突约束。**没有任何活动轨迹时，行为与改动前逐字节一致。**
- **播完**：保持末点位置（机构无抱闸，软停会掉臂）。上层按自己发出的末点
  时刻判断结束，再发新轨迹或取消。
- **容量**：拼接后的整条轨迹最多 256 个路点（所有电机合计，
  `CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS`）/ 64 台电机
  （`MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS`）；单个分段的 `count` 是**该分段**的有效
  路点数。分段乱序、缺段、时间回退、总点数超限或边界数据冲突时整组拒收，当前
  轨迹不受影响。拼接增加一个固定 256 点 assembly 缓冲，禁止动态内存；实际静态
  占用以当前目标链接图为准——2026-09-22 目标构建测得 `.data + .bss = 310,220 B`，
  其中落在 320 KiB RAM_D1，约 **94.7%**；PlatformIO 全 RAM 口径为 53.7%，
  Flash 为 20.4%。改小上面两个常量是唯一的省 RAM 手段。**RAM 口径**：`.data`+`.bss`
  全部落在 320 KiB 的 RAM_D1 上；判断余量必须使用 RAM_D1 口径。历史 68.7% 数值
  已过期，参数面不占任何常驻堆（见「参数面」一节）。

### 故障清除（`SystemCommand` 的 `COMMAND_CLEAR_FAULT`）

参数写"电机 id"触发，只清**可清的**故障，按**成因**区分：

- **身份冲突锁存**（同 `motor_id` 跨总线/协议身份不符，`CONFIG_CONFLICT`）：
  ARD §9.3 刻意不可解除，**拒绝**（复位只能整机重启）。
- **设备参数未初始化**：⚠️ 2026-10-12 起它**不再是 `CONFIG_CONFLICT` 生命周期**、
  也不进 `last_error`（只报 `dev_init`）。要重跑回读与比对仍走一次
  `REFRESH_PARAMS` 事务；真不一致仍判 `MISMATCH`——清除只负责"允许重走授权链"，
  **不伪造一致**。
- **厂家故障**：发厂家清错帧（CERB 的 MIT 控制字 bit5）。是否真清掉由设备后续
  状态帧的 `fault_active` 判定，走既有 `FAULT → READY_DISABLED` 路径，无新确认
  机制。Vaxor 控制帧没有清错位 → 内核回 `UNSUPPORTED`（不猜厂家字节）。
- 清错帧是控制类帧，**仅非运动态受理**（与设备侧限位写同口径）；清完仍须显式
  **重新武装**（`/micro_ros_do/policy` 的 `enabled=true`，且代数须前进）+ 新的完整
  快照/轨迹才可能回运动（ARD §36）。

只读完成序号 `system_status.clear_fault_seq`，用法同 `set_zero_seq`（发命令 → 轮询
seq 收敛，不要只看 `last_error`）。

### 参数面：两条话题（2026-09-21 起，rclc 参数服务器已退役）

参数面**全部是话题**，没有 ROS 参数服务器：

| 方向 | 话题 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| 下行 | `micro_ros_do/config` | `ConfigRequest` | reliable | `kind=PARAM_SET`：稀疏参数写，**整条受理或整条拒绝**；`kind=SYSTEM_COMMAND`：`SAVE` / `REFRESH` / `SET_ZERO` / `CLEAR_FAULT` / `REPORT` |
| 上行 | `micro_ros_up/system_status` | `SystemStatus` | best-effort, 5 Hz | `last_error` 电平 + 四个完成序号 |

- **`ParamSetTarget` 的 `field` 是设备槽序**（0=pos_min 1=pos_max 2=vel_limit
  3=torque_limit 4=limit_protect 5=direction），与 `device_param_report` 的行序、
  `mismatch_mask`/`record_dirty_slots` 的位序同一套——读写同序可比。
  ⚠️ **它不等于 `param_field_t` 的次序**（那里 3 = pos_min、5 = vel_limit）；两套里
  `field=3` 都合法，互用**不会被拒绝，只会静默写错槽位**。缓解三条并用：.msg 里
  定义常量（调用方不手打数字）、两端注释写明、台架专测一条（写 `field=3` 后确认
  只有 `rec_torque_limit` 变）。固件侧映射表 `src/param_bridge.cpp` 的 `kFields[]`。
- **位置限位必须成对出现在同一条消息里**（只给一边 → 整条拒绝），因为成对提交才
  表达得了"下限 < 上限"这个约束；只想改一边时另一边从 `device_param_report` 的
  `rec_pos_*` 读回一起发。
- **上界 16 是内核约束不是带宽约束**：每个目标一次配置 FIFO 提交，而该 FIFO 深 16。
  参数面是唯一逐目标进 FIFO 的下行路径（low_cmd/trajectory/policy 都是整条快照一次
  提交），有 `static_assert` 守着。
- **写是异步事务**：`config`（`kind=PARAM_SET`）同步返回只表示"已受理"，成败经内核
  `CONFIG_RESULT` 事件异步给出，最终呈现为 `system_status.last_error`。**失败不回滚、
  没有影子**——话题是命令不是状态副本；失败即 `last_error` + `record_dirty` 不翻转，
  重发即可。
- **完成判据是序号不是错误码**：`system_status` 的 `refresh_seq`/`set_zero_seq`/
  `clear_fault_seq`/`save_seq` 只增不减，每次命令收尾（**无论成败**）+1。只看
  `last_error` 分不清"上一轮成功留下的 0"和"这一轮还没跑"。
- **`save_in_progress` 覆盖全程**：设备侧分批下发、等待每台 `CONFIG_RESULT`、W25
  落盘三段都算"保存中"，必须等设备侧全部成功且 W25 出终态才转 false。旧实现把
  设备请求推出完或落盘请求发出误当成"保存完成"，报告完成时设备/W25 可能仍未完成。
- **`enable`/`kp`/`kd` 不在这里**：它们在 `/micro_ros_do/policy`。这三项变化快，
  且 ARD §19.4 本就禁止它们进命令快照。**稀疏语义**：只影响列出的电机，其余保持原状。
  **代数门**：「未武装 → 武装」要求 `policy_generation` 严格前进，挡住「周期
  重发同一条策略」在故障后自动复权（挡不住每次都递增代数的发布者——那与人工
  逐次授权不可区分，ARD §36 的「显式」在机制层面到此为止）。
- **为什么退役**：rclc 参数服务器每槽**实测边际 328 B**（结构 269 B = `sizeof(Parameter)=112`
  + `descriptor=68` + 50 B 名，余为分配器块开销），且 init 时**按 max_params 一次性
  全部分配、与板上实际几台电机无关**——72 槽常驻 23,616 B 的 DTCM 堆，而那个池子
  **物理不可扩**（= DTCM 128 KiB − 主栈 8 KiB；FreeRTOS 与 newlib 同池）。
  它从未被正面论证过，且剩下的 `system.*` 本就是命令不是参数（"写 = 电机 id、
  轮询序号判完成"）。历史推导留在
  `docs/architecture/motor-runtime-decision-log.md` §6.9/§6.12 与
  `docs/plans/active/48-motor-scale-design.md` §5.2/§5.3。
- **收益口径（不要夸大）**：真正拿回来的是 **~23.6 KB DTCM 堆**（运行期，不体现在
  `.data`/`.bss`）。`MAX_SERVICES` 的 5 个槽确实成了余量，但 publisher/subscription
  槽被这三条新话题吃回去了（净收益 ≈ 0），executor handle 9 → 6。
- `param_field_t` 仍是 8 项且**序号一个都不能动**——它是 Flash blob 的 `present`
  位图与 `device_raw` 槽索引契约；记录布局不变、无需迁移。名字生成/解析
  （`m<id>.<field>`）已随参数服务器删除。

下行配置通道（`motor_enable`/`motor_policy`/`motor_config`/`motor_bind` 与
`motor_config_result`）已删除：型号绑定由内核读设备自报身份自动完成
（2026-09-16 起，静态拓扑表已删）——发现建档/重连后经事务读 CERB SDO 0x1008
（厂家名称）并与型号目录整串精确匹配；vaxor 无身份读对象，回退到"协议内唯一
型号"。未命中目录的设备保持未绑定不可控。授权/增益/限位取内核安全默认（建档
`policy_enable` 默认 true、全局授权 mailbox 引导豁免、增益回退 profile
`default_kp/default_kd`、CERB 限位在驱动器侧配置）。

设备侧管理对象（`0x607D:01/:02` 位置限位、`0x6080` 电机端最大转速、
`0x2538` 电机端峰值电流）的写 + 读回核验留在内核/adapter 层（写仅限未运动
状态，读回不一致收敛 `UNCONFIRMED`），ROS 侧入口是 `config`（`kind=PARAM_SET`）的
`FIELD_POS_MIN`/`FIELD_POS_MAX`/`FIELD_VEL_LIMIT`/`FIELD_TORQUE_LIMIT`（见下）。

**参数单位（2026-09-20 改）**：位置 **rad**、速度 **rad/s**、力矩 **安培**。前两项与内核、
`motion_states`、URDF 同一套单位——参数与状态放在同一个页面上时要人脑换算两套角度
是没有道理的（此前 2026-09-17 起对外用度/度每秒，已改）。力矩列仍用 A：设备侧权威
对象是 `0x2538`（电机最大电流，「建议用默认」），读出来的正是设备里存的值；折成
N·m 要过一次 Kt×减速比×效率，而该系数存在未决争议（见
`docs/architecture/motor-runtime-decision-log.md` §6.4/§6.6）。力矩的 A ↔ N·m 换算在
`param_bridge` 边界做一次（`lib/param_store/include/param_units.h`），位置/速度是直通。
**系数未知（未绑定型号）时拒写而不是猜**：换算失败在整条预校验阶段就把该条消息拒掉，
不会留下"半个限位已下发"。

**外部写入不得超过固定硬件参数表（2026-09-17）**：型号目录
（`lib/motor_control/src/motor_control_profile.c`，数值取自规格书
`docs/CE-RB-R{58,48}-101-*-参数规格.pdf`）是唯一基准。写入按四维取更严者钳制——
扭矩/电流（峰值扭矩 45/11 N·m、峰值电流 10.8/3.2 A）、速度（≤ 额定 π rad/s）、
位置（≤ ±π）。**限值对象 0x2538 的换算系数用规格书峰值对**（45/10.8、11/3.2），
所以"写峰值扭矩"正好写出"峰值电流"；反馈扭矩估计仍走 Kt×减速比×效率。

**设零点（2026-09-17）**：`SystemCommand` 的 `COMMAND_SET_ZERO` + `motor_id` 触发，内核写
**0x2531 = 1**（字典表「设置电机输出端零位」）。单阶段、只等写应答；**仅失能态
受理**（运动中被拒，不隐式失能）。零点不是本机记录的参数，不改影子/记录。
⚠️ 零点改变后设备内 0x607D 限位随之移动，须重认限位与机械行程。**持久化已实测**
（设零后驱动器断电重上电零位保持）：0x2531 写入即落盘，不需要额外保存码。

**设备侧写事务以写 FLASH 收尾（2026-09-17）**：0x607D/0x6080/0x2538 写 + 读回核验
之后追加一条 **0x2539「Flash保存参数代码」**（字典表：1=电机参数、2=正限位、
4=负限位；速度/电流发 1，位置限位发 4 与 2 两条）。厂家手册明确「0x607D 设置
完成后需要保存 Flash」——驱动器里的限值不落盘就掉电回出厂值，下次上电的设备参数
比对会判"不一致 → 未初始化"。该阶段无应答、不重试（驱动器写 flash 会阻塞自身控制
循环，等 ack 只会把总线一起拖住）；是否真落盘由下次上电比对检出。

**配置事务一问一答（2026-09-17）**：厂家 SDO 是严格请求-应答，固件按总线串行
——同一总线上任一时刻至多一个未决阶段（并发发问会让设备应答排队、迟到应答被判
重试，表现为"回读一直超时"）；CERB 阶段超时 30ms。上位机判断"读完了"用只读的
`system_status.refresh_seq`（每次回读收尾 +1），不要只看 `last_error`（分不清轮次）。

**固件记录为权威（2026-09-21 反转 09-17 口径）**：上电/重连绑定完成后，内核自动对
CERB 发一次 `REFRESH_PARAMS` 回读六个管理对象，与 W25 里的固件记录**逐槽位比设备端
原值**（cnt/RPM/mA；比关节侧 SI 会在换算系数配错时两边一起错）。**要动的是驱动板，
不是记录**——初始化语义的来源是固件那份表。

- 一致 → 初始化通过，正常可控；
- ⚠️ **2026-10-12：初始化结论只报不拦（维护者 为台架取消设备参数门禁）**。
  `PENDING` 回读未收尾 / `MISMATCH` 不一致 / `UNVERIFIED` 回读失败 / `NO_RECORD`
  本机无该台记录，**都不再改 lifecycle、不再拒绝 `SET_ENABLE`、不再拦运动帧**：该台
  停在 `READY_DISABLED`，可直接使能与运动。结论仍如实上报——`dev_init`、
  `EVENT_DEV_INIT`（reason = 结论、detail = 不一致槽位位图）、
  `device_param_report`。代价：`dev_init ≠ OK` 时该台的限位/速度/力矩可能不是核对过的
  值（影子取设备真值，读不到时回落型号标定），兜底只剩硬件急停与机械限位。
  恢复门禁需改两处，缺一不可：`lib/motor_control/src/motor_control_internal.h` 的
  `motor_control_dev_params_allow_motion` 改回 `(dev_init == NA || dev_init == OK)`；
  `lib/motor_control/src/motor_control_config.c` 放开注释掉的
  `freeze_entry_as_conflict(...)`。设备参数结论不再驱动 `CONFIG_CONFLICT` 生命周期与
  `system_status.last_error`（那两者现在只对**身份冲突**保留）；
- **写 ≠ 保存**：从 `config`（`kind=PARAM_SET`）写回该台参数只改设备 RAM 与固件记录
  （写后立即重判，写齐即转好），要**持久化**必须再点「保存」（写 `0x2539` + 落 W25）。
  未保存的台在 `micro_ros_up/device_param_report` 上 `record_dirty=true`，页面标黄。
- **运行中回读发现不一致 → 只报结论**（`dev_init=MISMATCH`），**不动运行态**：
  不软停、不失能、不掉授权（2026-10-12 起；此前是"软停、保留使能"）；
- 自动回读失败会退避重试（≤3 次、间隔 500ms；读是只读事务，重试无副作用），
  用尽才判 `41`，避免一次总线抖动就让该台使能不了；
- 设备侧写事务读回核验通过后，内核用该事务实际编码出的原值同步固件记录并置
  `record_dirty`；**落盘只在收到 `COMMAND_SAVE` 时发生**（写不再跟 `0x2539`、
  也没有去抖自动落盘）。
- W25 blob 升 v3（记录 120 B：多 4 个设备端原值 + 保护开关/方向 + 型号身份），
  v1/v2 各自就地迁移，不丢已存参数。

⚠️ **本节新口径尚未上板验证**：`NO_RECORD`（`dev_init=NO_RECORD`；2026-10-12 起不再有
`last_error=42`）、写/保存分离、`0x2539` 独立保存事务与 `device_param_report` 话题为
2026-09-21 的设计/host 口径；
**「写后不保存即复位」仅台架可判**（host 没有 W25 与掉电语义）。真机台架项见
`docs/plans/active/motor-runtime-validation.md`（TBD-CONFIG-01 追加项，**未运行**）。

**上电归一：保护默认开、方向按记录写回（2026-09-20）**：CERB 的两个安全对象
（`0x2650:00` 看门狗+限位保护、`0x2541:00` 方向）**都是易失的**——上电分别回到
`1`（保护关）与 `0`（CW）。固件因此在设备参数**回读比对之前**插入 **0a 上电归一**阶段
（`motor_control_config_step`）：保护恒写 `0`（生效），方向写 W25 记录值（记录里没有
这一槽就**跳过、不猜**）。归一没成功（写/读回核验重试用尽）→ 该台判 `UNVERIFIED(41)`
（2026-10-12 起只报结论，不再进 `CONFIG_CONFLICT`、不再拦使能），且不会发起回读。
⚠️ **顺序是契约**：写在比对之后会让每次上电白报一次不一致（比对先看到上电初值）。
⚠️ 归一未收尾前，人工 `COMMAND_REFRESH`（页面"回读"按钮）会被拒绝（BUSY）——否则
读到的同样是归一前的值，还会把该台锁在冲突态、连归一都做不下去。

**方向参数（`ParamSetTarget` 的 `FIELD_DIRECTION`，0 = CW / 1 = CCW，原值直通不反转）**：可读可写，
写 = SDO 写 `0x2541:00` + 读回核验，仅未运动时受理。**真机实测（2026-09-20）纠正**：翻转**不**改变位置坐标系——同一物理方向转动
关节时，设备侧 `0x6064` 的计数方向不随 `0x2541` 变化（同向转两次 Δ 均为正：
+2394 / +2573 cnt），`0x607D` 限位对象也全程不变。所以**零点与限位都不镜像、
不需要重新设零**（早先"镜像 + ZERO_STALE 冻结"的设计基于一条未经实测的推断，
已按实测删除）。翻转真正的风险是：驱动器自持的位置环变成**正反馈** → 使能后
关节朝**远离目标**的方向冲，直到撞上设备限位（限位仍拦得住，它按位置判）。
因此方向的门禁只有一条：**仅未运动时受理**；页面点一次方向按钮即切换。

**保护被手动关掉后仍可武装，但每次都要显式确认**：`MotorPolicyTarget.confirm_unprotected`
（缺它就是按不下去，拒因 `UNPROTECTED_UNCONFIRMED(9)`，只进 `last_failure_reason`）。
关保护 = 该关节没有任何限位在跑，页面据此把该行标红。**下次上电归一会自动恢复为开。**

**设备侧保护开关与方向纳入上电比对（2026-09-19）**：CERB 的 `0x2650:00`
（看门狗+限位保护开关：`0` = 保护生效、`1` = 关闭）与 `0x2541`（方向，0 = CW）也并入
`REFRESH_PARAMS` 回读，与记录逐槽位比对。CERB 的关节限位**完全靠设备侧
`0x607D` 强制执行**（`CAP_LIMIT_DEVICE_SIDE`，固件没有软限位兜底），所以
`0x2650:00 = 1` 等于该关节**没有任何限位在执行**。任一不一致 → 该台判
`MISMATCH`，并在 `EVENT_DEV_INIT` 的 reason 位图点名 bit4=保护开关、bit5=方向
（2026-10-12 起只报结论，不再进 `CONFIG_CONFLICT`、不再拒绝 `SET_ENABLE`）。
**恢复路径（2026-09-19 起）**：`0x2650:00` 已进参数面，参数项
`FIELD_LIMIT_PROTECT`（**1 = 保护生效**，0 = 关闭）可读可写，写事务 = SDO 写
`23 50 26 00 00 …` + 读回核验，仅未运动时受理，写完记录自动同步。**保护开关是
参数面上唯一的反向字段**，反转只发生在 `src/param_bridge.cpp` 的
`submit_safety_protect` 一处（mailbox 与协议层只认厂家原值）。~~剩下的
`0x2541` 方向仍然不在参数面~~ **已于 2026-09-20 更正**：方向也进了参数面，
参数项 `FIELD_DIRECTION`（原值直通：0 = CW，1 = CCW），写事务 = SDO 写
`23 41 25 00 **0x** …` + 读回核验（`40 41 25 00 …`），仅未运动时受理。
掉电不保存，由**上电归一**在回读比对之前按 W25 记录写回（见上面的新小节）。
**对象语义**（经 维护者 手工核实，SLCAN 复现）：读 `40 50 26 00 00 …` → 反馈
`43 50 26 00 **0x** …`（`0x` 即当前状态）；`23 50 26 00 **00** …` = 开启保护、
`23 50 26 00 **01** …` = 关闭保护。

⚠️ **掉电不保存（2026-09-19 实测）**：`0x2650` 是**易失**设置，每次上电回到上电
初值（台架四台均为 `1` = 关闭）。`0x2539` 保存码**对它无效**——官方 §4.11 明确
码 `1` 只保存 "PI 参数和最大电流"；实测也印证（发过码 1 后掉电，照样回退）。
若上层要求"保护必须开着"，**必须每次上电显式发一次** `23 50 26 00 00 …`，
不能指望设备记住。
⚠️ **开保护的自锁风险：假设未成立（2026-09-19 实测）**：手册称「看门狗数据帧
间隔要小于 500ms，否则电机自锁」，但 1 号机开到保护后停发全部帧、静置 5 秒，
反馈仍在 12B 上报、故障位为 0、SDO 仍应答——**没有自锁**。单机空载静态观察，
不足以证明带载运动中也安全。

⚠️ **语义是"变更检测"，不是"保护必须在位"**（2026-09-19 明确）：比对基准是
**上次成功落盘时设备里的值**，不是"保护=0"这个常量。两处必须记住：
- **先关保护、再触发任意一次保存，记录就被重新基线化成"关"**——下次上电设备
  与记录一致，该台正常使能（而它此刻确实没有限位在执行）。保存触发不要求写参数：
  `dev_init ≠ OK` 的台仍受理限位写事务，写成功即置脏等保存。所以别把"上电没报
  不一致"当成"保护一定开着"。
- **刚由 v2 迁移上来的记录，两槽不参与比对，直到第一次保存**（迁移只搬低 4 位），
  即每块刚升级的板子都处在这个状态里。

发布（上行，8 个话题；均为 ID 化，缺失浮点 = NaN）：

- `micro_ros_up/motion_states`（`MotorMotionStateArray`）：运动状态（50Hz，
  由 10ms bridge timer 每 2 拍发一帧；控制边界独立为 2ms/500Hz）。
- `micro_ros_up/motor_status_array`（`MotorStatusArray`，20Hz）：诊断与电气状态
  （生命周期/故障码/保护位/事务状态）；每个 `MotorStatusTarget` 也带有实际发现的
  `bus_index`（0..2 = FDCAN1..3）。
- `micro_ros_up/motor_topology`（`MotorTopology`，1s 节流）：发现式拓扑
  （`motor_id`/总线/协议/型号/能力/冲突标志）。
- `micro_ros_up/motor_events`（`MotorEvent`）：生命周期/安全事件流（含身份识别
  的 PROFILE_BOUND / PROFILE_REJECTED）。带 `event_seq`（单调递增，相邻跳号即
  说明事件环满过；累计丢弃数见 diagnostics）。
- `micro_ros_up/trajectory_state`（`MotorTrajectoryState`，10Hz）：轨迹执行状态——
  `trajectory_id`、active/completed、已执行/总时长、受控集合。轨迹是 best-effort
  下发的，没有它就只能靠猜「我发的计划受理了吗、跑到哪了」。
- `micro_ros_up/diagnostics`（`MotorDiagnostics`，1Hz）：固件级计数器一次性曝光
  （帧计数/解码错误/快照与配置拒绝/事件与 TX 丢弃/P0 重试/轨迹与策略收支/
  每总线 ring 溢出/堆余量）。这些此前只存在于芯片里，实时性与总线容量的问题
  只能靠 JTAG 定位。
- `micro_ros_up/device_param_report`（`DeviceParamReport`，**按需**：REPORT
  命令 / CONFIG_RESULT 写事务收尾 / DEV_INIT 设备初始化结论）：
  参数核对双行快照（固件记录列 / 驱动板回读列，同槽位同单位、力矩列按 A）+ 未
  保存标记 + 不一致槽位位图。**它是 `config`（`kind=PARAM_SET`）的读侧对照**：写之前
  先读回 `rec_*` 列，位置限位要成对发就把两边一起读出来（行序与 `ParamSetTarget` 的
  `field` 是同一套）。没有在线电机时帧里 `dev_*` 全为 NaN，上层看到的就是板内参数表。
- `micro_ros_up/system_status`（`SystemStatus`，5 Hz + 变化即发）：`last_error`
  电平 + `last_error_motor` + `refresh/set_zero/clear_fault/save_seq` +
  `save_in_progress`。参数面的"这次命令跑完没有"只由它回答（见「参数面」节）。

**三份上行状态分工**：topology 是身份与总线映射的权威（谁在、哪条总线、什么协议、
型号、能力、冲突、`schema_version`）；status 提供 20Hz 状态/电气遥测，并重复携带
`bus_index`，让 Web 能从同一条实时状态流关联总线；motion_states 只提供运动学
（位置/速度/力矩、`state_age_ms`、`target_source`）。`target_source` 让上层看见
「我的快照为什么被忽略」。当前消息面版本为 3（status 追加了 `bus_index`）。

`motor_id ↔ joint_name` 的映射、方向与零位是**上层职责**（上层分段转发的源代码
已同步，rosbridge/Agent/MCU 端到端与真机连续性仍待验证）；固件不做 name-to-ID
适配。

## 控制安全状态与恢复语义

- `motor_rt_task` 每 1ms 服务、每 2ms（500Hz）控制边界消费一次完整快照；
  内核是唯一生命周期裁决者，任务层是唯一正常 FDCAN 发送者（P0/P1/P2 分级）。
  **控制边界同时决定 CAN 命令速率**：轨迹机组由 `trajectory_step` 逐边界插值、
  快照机组由 `hold_step` 逐边界保持重发，两条通道合计每台每边界恰好一条命令。
  上位机的快照到达速率（实测 ~72/s 封顶）不参与决定命令率。
- 快照 freshness（MCU 单调时基）：`>30ms` 无新快照 → 全体受控电机 MIT 默认
  `kd` 阻尼软停；`>100ms` → 协议级失能并清授权；Agent 断链立即失能清授权，
  不等 100ms。恢复必须 新授权 + 新完整快照，不复用旧目标。
- 快照内缺席的受控电机立即软停；`motor_id` 重复/未知/NaN 的快照整体拒绝。
- 授权由会话状态派生（motor_enable 话题已删）：Agent 状态未知（引导期）或在线
  即全局授权；断连立即撤权，30/100ms 快照超时软停/失能后会话仍在线时，**重新
  发送新鲜完整快照即自动恢复**（下一控制边界按会话电平重立非冻结态授权）。
  FAULT/OFFLINE/CONFIG_CONFLICT 冻结态的逐电机授权不被全局重立复活——基础条件
  消除后只能显式 SET_ENABLE(true) + 新完整快照恢复（身份冲突锁存见下条）。
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
  配置时必须同时验证该约束。错误状态（EW/EP/BUS_OFF）计数；BUS_OFF 由
  `HAL_FDCAN_ErrorStatusCallback` 清 CCCR.INIT 自恢复（M_CAN 进 bus-off 硬件
  自动置 INIT，软件清零后等总线同步，回调幂等），持续物理故障的恢复策略仍待
  台架 T12 数据。
- BMI088 初始化使用 `imu_try_init()`：每次只尝试一次，失败后 sensor task
  进入降级态并每 1s 重试，不阻塞 control/micro-ROS。每个 SPI byte transfer
  最多阻塞 2ms，首次 HAL 失败后锁存 transport error，当前初始化/采样余下访问
  快速失败；失败时 heater PWM 立即置 0。初始化成功后，空的二值信号量保证
  第一份有效样本必须来自真实 ACC DRDY；sensor task 写双缓冲 snapshot，ROS
  只复制已发布缓冲。运行中 SPI 失败会关闭 heater 并在后续 DRDY 再尝试采样。

## `dev-real` 无负载真机验证清单（仅人工执行）

以下项目**不得由自动化测试执行**。只能在 `dev-real`、机械无负载、硬件急停
已验证可用且人员可立即断电时逐项执行；不安装负载、不验证抱闸（当前无抱闸），
也不得据此宣称已达到硬件安全：

- [ ] MIT 小幅零附近命令连续运行，示波器/logic analyzer 验证 motor_rt TX 为
  500Hz（控制帧；另行核对 20ms/50Hz SYNC/状态心跳），并记录 motor_rt cycle
  WCET 与周期 jitter。
- [ ] 停止刷新快照，验证 30ms 边界进入默认 `kd` 阻尼软停、100ms 边界协议级
  失能并清授权；单电机从快照缺席，验证仅该电机软停、其他电机不受影响。
- [ ] Agent 断连/重连，验证断连立即失能清授权、清旧命令；重连后内核重新读
  设备自报身份（CERB SDO 0x1008）并重绑定收敛 READY_DISABLED，恢复运动必须
  显式 SET_ENABLE(true)（编译期冻结语义）+ 新完整快照，期间不得自行恢复或重放。
- [ ] 身份识别台架核对：`ros2 topic echo micro_ros_up/motor_topology` 中各 CERB
  节点的 `model_name` 应与铭牌一致（R48/R58 不得互相误判）；拔掉某台设备后
  该行消失，重新上电后被重新识别；目录外型号应停留在未绑定并出
  PROFILE_REJECTED 事件。
- [ ] 验证 OFFLINE/厂家故障冻结的逐电机授权不会因重连自动复立（须显式
  SET_ENABLE(true) + 新快照）；
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
- [ ] 示波器或 candump 验证 CERB 所在总线每 20ms 仅一帧 `0x080` SYNC，设备
  20ms 返回上报帧；确认上报密度与 100ms OFFLINE 边界匹配实际设备行为。
  **同一趟一并核对 SYNC 的帧格式**：固件 2026-09-22 起发 FD+BRS 的 `0x080`，
  但厂家说明书只写明 SDO 是「CANFD 加速标准帧」，SYNC 无书面依据。真机若
  不接受 FD 的 SYNC（表现为发现不了任何电机、`rx_frame_count=0`），把
  `cerb_encode_discovery_sync` 的 `is_fd`/`is_brs` 改回 `false`。
- [ ] 断开/恢复 BMI088、阻塞 SPI 或停止 DRDY，验证 2ms transport bound、每秒
  初始化重试、heater off 与双缓冲不撕裂。

## 已知问题

- BMI088 收发固定走阻塞 `HAL_SPI_TransmitReceive`，当前不占用 DMA stream；若后续切换
  DMA，设备句柄/回调仍归 `lib/imu`，控制器资源须向 `lib/dma` 登记。
- 无 agent 时节点不注册、固件在 `kWaitAgent` 轮询，属正常。
- `loop()` 被 STM32FreeRTOS 的 idle hook 调用，只能放非阻塞代码。
- `can_bridge_msgs` 共 **21 个** `.msg`（13 个顶层 + 8 个 Target/Entry 子消息），唯一来源是
  `../micro_ros_agent/src/can_bridge_msgs`；固件侧只用 `extra_packages/can_bridge_msgs`
  符号链接复用，不存在第二份副本。
- CERB R48/R58 的 MIT 范围配置为位置 `±π rad`、速度 `±π rad/s`、力矩 `±11/±45 Nm`，
  必须与真机参数设置页一致。`motor_rt_task` 每 20 ms 发 `0x080` SYNC 维持设备状态心跳，
  SYNC 与 SDO/MIT 同为 **FD+BRS**：经典帧不经过数据相位，数据段位率配错时唯独 SYNC 还能通，
  会造成「链路正常」的假阳性。总线仍是混合的——真机 CERB 器件自己按字典发经典帧（SDO 应答）。

## 开放项与开发计划

**瓶颈是验证债，不是能力债**：能力基本已实现，绝大多数开放项缺的是真机/HIL 证据。
本节是开放项的**唯一清单**（原 `docs/plans/active/` 与 `docs/architecture/lib-library-audit.md`
于 2026-10-14 文档收敛时并入）。TBD 编号沿用原清单，不重编。

### 三条硬约束（动任何东西前先看）

1. **RAM_D1 余量 17,460 B**（310,220 / 327,680 = 94.7%）。唯一的直接省 RAM 手段是调小
   `MOTOR_CONTROL_MAX_TRAJECTORY_POINTS` / `MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS`
   （见 `lib/motor_control/include/motor_control.h`）。
2. **机构无抱闸，且软停帧本身是零位置刚度**（`kp = 0`）——30 ms 与 100 ms 两级都是「撒手」，
   对承重关节不是安全停止（见 TBD-SAFETY-03）。软件停机不替代硬件急停与机械限位。
3. **`libmicroros.a` 的 meta 槽位是库编译期事实**（publishers 10/12、subscriptions 4/6、
   services 0/5），`-D` 改不动；改 meta 或 `.msg` 只能走
   `../micro_ros_agent/tools/refresh_microros_messages.py`（增量、离线），不是 `-t clean_microros`。
4. **单条消息上限 512 B，且 best-effort 输出流只有一块全局静态缓冲、所有 publisher 共用**
   （`UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512`，输出流在 `rmw_init` 建一次）。所以「按总线拆话题」、
   「加一条诊断话题」、「加一个自定义服务」这类改动都会在 init 阶段撞墙；大消息（如多点轨迹）
   必须自行分段，段间还要考虑 XRCE 分片窗口。

### 开放项

> 「host 已覆盖」= 逻辑有自动测试、只缺真机证据；其余 `[ ]` 均需台架或设计动作。

**实时性与总线容量**

- [ ] **TBD-RT-01** 实测 `motor_rt_task` WCET、1 ms 周期 jitter、P0/P1 最坏发送延迟；含
  **TBD-RT-02** 轨迹步进与 mailbox 轨迹校验的最坏耗时（台架前只有静态上界）。
- [ ] **TBD-CAN-01** 最坏反馈流量下 RX ring / P2 配额 / TX 队列容量：记录 overflow、backlog、
  队列峰值、总线负载。
- [ ] **TBD-CAN-02（单总线 VAXOR 容量）** VAXOR 的 P1 不能多轴聚合，每总线每 1 ms 最多提交
  16 个物理帧；按 500 Hz 控制周期估算，单总线 32 台 VAXOR 已用满理想 P1 预算（尚未计 P0/P2）。
  参考负载：4 台 200 Hz 命令仅 4.35%，16 台/总线 26%——**总线负载不是约束，中断余量才是**。
  需明确拓扑上限，或重新评估调度预算。
- [ ] 三总线约 40 台 VAXOR/CERB 混合持续压测；量 USB CDC 状态上行带宽余量。
- [ ] 故障注入下 P0 不静默丢失（CAN 无 ACK、RX burst、下游硬件拒收；重入队/抢占逻辑 host 已覆盖）。
- [ ] 长时间热插拔、离线/恢复循环稳定性。
- [ ] **TBD-STORAGE-01（v3 双扇区擦写阻塞）** save 从擦 1 个扇区变擦 2 个，阻塞时间约翻倍。
  本文档与 `AGENTS.md` 里的 "~1 s" 是 v2 单扇区旧值，**v3 实测值待台架**（与「擦写中掉电」同轮记录）。

**安全语义（需真机）**

- [ ] **TBD-SAFETY-01** 以厂家文档 + 无负载台架确认两协议 30 ms 软停的实际动作（缺席软停、
  30/100 ms 时基与单电机隔离语义 host 已覆盖）。
- [ ] **TBD-SAFETY-02** 确定 RX ring overflow 触发安全降级的**阈值与恢复条件**——当前只有 TBD
  标记，机制未定义，属真缺能力而非仅缺证据。
- [ ] **TBD-SAFETY-03（软停是「撒手」，设计缺口）** 30/100 ms 两级软停分别走
  `cerb_encode_neutral` / `vaxor_encode_soft_stop`，二者只填 `kd` 与 `enable` ⇒ `kp = 0`、
  目标位置 0、前馈 0，即「零刚度 + 保持使能」。受控保持的能力其实已存在（轨迹通道的
  末点钳持 + `motor_control_hold_step` 每 2 ms 重发），缺的是**安全级联没有第三档去调用它**。
  另一前提：真正的前馈来源在上位，`robot_dynamics` 目前不实现重力补偿。
- [ ] 策略通道真机（`micro_ros_do/policy` 的 enable/kp/kd）：代数门（故障后重发同一代数
  必须被拒）、稀疏语义（未列出的电机保持原状）、断连重连后未重新武装不得自动复权。逻辑 host 已覆盖。
- [ ] 快照通道 500 Hz 命令率真机（把上位快照按 50 Hz 发，低于 30 ms 新鲜度阈值）。
- [ ] 快照拒绝真机注入：NaN/Inf、未知/重复/跨总线 ID、非法 mode；厂家故障锁存、心跳丢失
  100 ms OFFLINE 与恢复路径（级联逻辑 host 已覆盖）。
- [ ] CERB 清错帧真机（对新硬件参数表）；Vaxor 故障恢复（`system.clear_fault` 应回 `UNSUPPORTED`）。
- [ ] CERB 每总线 20 ms 单帧 SYNC、约 20 ms 状态上报、100 ms OFFLINE 边界实测；线圈温度
  字段（12B 反馈 `data[8..9]`，大端 int16）真机核对。
- [ ] BMI088 断线 / 阻塞 SPI / 停 DRDY：传输超时、降级重试、heater off、双缓冲不撕裂。
- [ ] 500 Hz 多轴快控复测（同总线 4 台）：按固件侧 `tx_frame_count` 差分确认约 550 帧/s
  （500 × `0x210` + 50 SYNC），并核对 `tx_hw_reject` / `p1_replaced` / `tx_dropped` /
  `rx_hw_lost` 无持续积压。此前 200 Hz 单轴台架数据是旧版本基线，不能用来证明本项。
- [ ] 堆高水位（残余）：参数槽退役后，堆上不应再有按电机数线性增长的常驻项。
- [ ] **TBD-STORAGE-02（升级迁移兜底）** 旧固件上 save 奇数次会让活动 blob 落在另一槽，
  需确认新固件仍能选对。

**参数链与 Profile**

- [ ] REFRESH_PARAMS 按总线一问一答 + 四个 SDO upload 回读：模拟器台架已验证字节序列，
  真机 `0x2539` 保存闭环仍待 `dev-real`。
- [ ] **TBD-PROFILE-02** CERB 规格书的 Kt 三套口径互不一致，反馈扭矩估计仍依赖 Kt 链
  （真值若只有名义一半，上层力矩会偏大 1 倍）。判别实验：静止重力载荷下同时读字典
  `0x6077`（力矩，0.01 N·m）与 `0x6078`（实际电流，mA），算 `Kt_eff = τ/(Iq×gear)`。
- [ ] Vaxor 80 速比型号无完整规格，型号目录刻意不收；规格到位并补 Profile 前不得开放控制。
- [ ] 建立上层型号库（`motor.yaml`）与固件内置 Profile 的一致性机制（零机制）。

**上层同步**

- [ ] 长示教串行播放未上真机（2148 点级）：段界位置无跳变、每段 `trajectory_accepted` 与
  `completed` 逐次到齐、`mailbox_rejected_trajectory=0`，并实测一次段界停顿时长。
- [ ] 多段组装（`segment_count>1`）仍未上真机：两侧发送方现在恒发单段，乱序/缺段路径未验。

CERB 设备对象事实与三条待厂商答复项见 `docs/architecture/motor-runtime-decision-log.md` §12。

### 阶段与门禁

顺序是「先还哪一笔债」，不是「还要做什么功能」。阶段出口即门禁全绿。

| 阶段 | 内容 | 门禁 |
|---|---|---|
| 0 测量底座 | GPIO 翻转探针（进出 `motor_rt_task` 服务拍各翻一次）+ 每拍 `DWT->CYCCNT` 差值最小统计，让 WCET/jitter 从测不出变成可测。复用现有结构，不新增大数组 | G0：示波器可见 1 ms 周期方波，`pio run` 绿 |
| 1A 实时与总线容量取证 | 开放项「实时性与总线容量」全部；前置 `dev-real`、无负载、硬件急停就绪 | G1A：该节开放项归零，单总线 VAXOR 上限有具体数字 |
| 1B 安全带 | following-error / 过温监测；TBD-SAFETY-02 的降级阈值与恢复条件 | G1B：新增 host 用例全绿 |
| 2 安全语义真机 | 开放项「安全语义」全部 | G2：有结案记录，软停真机行为写成结论 |
| 3 Profile 一致性 | `motor.yaml` ↔ 固件 Profile 一致性机制；Kt 结论入 Profile | G3：「改 `motor.yaml` 即红」可复现 |
| 4 架构收口 | ADR 0007 转 `accepted`；RAM 减压决策记录（做或不做都要记） | G4 |

判据一律以**固件侧计数**为准，不依赖仪器——台架 SLCAN 在约 880 帧/s 已证不可信。

### 不做清单

| 不做 | 理由 |
|---|---|
| 在 STM32 上做动力学 / MPC / WBC | Pinocchio 已是独立计算边界（ADR 0008），不进实时路径 |
| 拆出第二个实时任务（`SafetyTask`/`MotionTask`/`ServoBusTask` 分家） | 新增所有者、新锁、新队列；收益（检测延迟 ≤1 ms → ≤0.5 ms）相对 30 ms 软停阈值与约 500 ms 看门狗无意义，且 TCB/栈要落在已满 94.7% 的 RAM_D1 |
| `DebugShellTask` / CAN 调试桥 | 调试桥已删除；`src/syscalls.cpp` 用强符号空 `_write` 禁止 MCU 侧 printf；调试走诊断话题 + SWD |
| 在 MCU 侧维护 `joint_name` / 建 JointRegistry 关节语义 | ADR 0007 §1 禁止项；映射只存在于主机侧 |
| 复活参数服务器（`rclc_parameter`） | 2026-09-21 整体退役；`services 0/5` 是有意保留的余量，不降 |
| 恢复 name-to-ID 兼容层 | ARD-0007 禁止项 |
| 在 RAM 预算审计前加任何静态大对象 | RAM_D1 仅余 17,460 B |
| 拿到容量数据前动架构或做减压 | 数据可能直接推翻扩展的必要性 |
| 给未标定型号（Vaxor 80 速比）开控制 | 没有规格就没有正确的 Kt 与限值 |
| 由 `motor.yaml` 代码生成 Profile 表 | 引入构建期依赖与生成物入库问题，收益不值（用一致性测试替代） |

### 待裁定（启动对应阶段前必须答复）

| # | 问题 | 影响 |
|---|---|---|
| 1 | 目标规模是 14 台还是约 40 台？ADR 0007 写「约 40 台外置电机」，而 `motor_driver.yaml` 是 14 台 | 阶段 1A 的容量判据与阶段 4 的 ADR 收口判据全取决于它 |
| 2 | R58/R48 实际带不带抱闸？本文档写「机构当前无抱闸」，但 CERB MIT 控制字 bit6 就是抱闸释放且固件恒置 | 决定 1B 的正解是「重新夹紧抱闸」还是「kp 保持」，实现完全不同 |
| 3 | 硬件急停是否已有物理通道（板上按钮 / EXTI 引脚），还是只有外部断电？ | 决定软件急停在 1B 里是纯逻辑还是要动板级 |
| 4 | 是否接受在 TBD-SAFETY-01 出结论前先动安全边界？ | 阶段 1B 的范围开关 |
| 5 | RAM_D1 减压是否获授权？ | 不授权则接入新执行器家族永久锁死 |
| 6 | 上位 `robot_dynamics` 的重力前馈是否作为外部依赖写进路线？ | 直接决定受控保持能否真的不掉臂（见 TBD-SAFETY-03） |

### 库结构与测试边界

14 个库的目录归属、职责与依赖方向经审查整体符合：无库 include `src/`或任务实现，
`motor_control → motor_protocol`、`storage_projection → motor_control + param_store`、
`session_sync → control_mailbox + motor_control` 均为单向、无反向环。测试边界：

- 依赖目标板验证的路径：`fdcan` 的 `can_bsp.c`/`fdcan.c`、`imu` 的 SPI/HAL middleware、
  `w25q64` 的 OCTOSPI 命令层、`dma`、`ws2812`。
- `control_mailbox` 的共享锁并发、NULL 提交、clear 与计数持久性有 host 用例，板级临界区
  调度时序仍需台架确认。
- `param_store` v3 的 `model_name` / `identity_hash` / MIT 三标度 / `profile_version`
  是**预留未实现**字段，当前不应宣称已持久化。
- `motor_protocol` 的 CAN FD 属性、配置确认对象、软停与离线阈值仍有 TBD；`fdcan` 的过滤器
  与 P2 配额也有待冻结项。
- **「两协议 ID 段不重叠」由 `test/host/test_protocol_family_isolation.cpp` 的全 ID × DLC 扫描
  守住**：归属循环是「首个命中即 break」，真重叠不会报错、只会把电机按错协议解码，所以该不变量
  必须有测试盯着，不能只靠一句注释。

## 文档

| 文档 | 内容 |
|---|---|
| 本文件 | 工程事实 + 话题契约 + 开放项与开发计划 |
| [AGENTS.md](AGENTS.md) | 操作手册：构建、烧录、台架顺序与坑 |
| [docs/adr/0007](docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md) | 架构决策（顶部含 2026-09-18 勘误） |
| [docs/architecture/module-boundaries.md](docs/architecture/module-boundaries.md) | 模块依赖、执行上下文、数据所有权与测试边界 |
| [docs/architecture/motor-runtime-decision-log.md](docs/architecture/motor-runtime-decision-log.md) | 设计理由与变更史（§12 为 CERB 设备对象事实与待厂商项） |
| [lib/README.md](lib/README.md) | 库分层与各库契约细则 |
| [../docs/guides/fdcan1-bench-guide.md](../docs/guides/fdcan1-bench-guide.md) | 台架操作指南（仓库根） |

厂商参考资料（原件不改内文）平铺在 `docs/` 根：控制板原理图与管脚标注图、VAXOR/达妙
16–30 mm 协议手册与 25–30 mm 参数规格、CERB/巨蟹协议说明书与 CANopen 字典表、R48/R58 参数规格。

协议文档中的电机轴/输出轴、角度、转速、电流和力矩单位不可直接渗透到上层；换算边界统一在
`lib/motor_protocol` 与型号 Profile。
