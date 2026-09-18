# micro_ros_firmware

STM32H723VGT6 上的 **micro-ROS 固件**（PlatformIO + STM32duino + FreeRTOS），
通过 **serial over USB CDC**（PA11=USB_DM / PA12=USB_DP）与主机侧
`../micro_ros_agent/` 通信，桥接进 ROS 2 Humble 域。

当前业务为 **BMI088 IMU** 与 **三路 FDCAN**，底层功能移植自达妙官方例程
`docs/dm-mc02例程/CtrBoard-H7_IMU_TempCtrl`（例程整树未纳 git）：
SPI2 阻塞传输、POWER_5V/CS/EXTI、TIM3 加热与温度 PID 均保留；
Mahony 姿态解算与 micro-ROS 发布为本工程增强，不属于官方例程内容。
三路 FDCAN（FDCAN1/2/3）移植自
`docs/dm-mc02例程/CtrBoard-H7_FDCAN`：FD + BRS、标称
1Mbps / 数据 5Mbps、接收中断回调保留；官方例程的周期测试帧未移植。
电机协议（vaxor 25/30mm + CERB CAN FD/MIT）已收敛为 `lib/motor_protocol`
adapter 接缝：厂家字节布局、CAN-ID 推导与 FD 属性只存在于该库，内核只见 SI
关节侧物理量。电机运行时为 **ID 驱动、发现式**（ARD-0007）：固件
不内置任何电机表或 `joint_name`，设备经反馈帧自主发现（vaxor 20ms 状态心跳 /
CERB SYNC 上报心跳），型号由内核读设备 SDO 0x1008 自报身份、查内置型号目录
自动绑定；命令是最多 64 目标的
ID 化完整快照。反馈模型（2026-09-05 与实际设备核实）：两协议反馈均分
"命令实时回显（50Hz，跟随控制边界）"与"20ms 周期状态心跳（承担总线检测）"两部分；
OFFLINE = 心跳丢失 100ms。上层迁移与真机台架开放项见
`docs/plans/active/motor-runtime-validation.md`（唯一清单）。

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
| W25Q64 Flash | OCTOSPI2 Port 1：PA1 / PA3 / PB0 / PB2 / PE11 / PD11，Quad page-program / Quad I/O read | `storage_task` 独占：双 sector A/B 参数持久化（见「W25Q64」节） |
| 电机接口 | ID 化有界消息（3 个订阅 + 6 个发布） | 发现式运行时，无 `joint_name`/固定电机表（ARD-0007） |
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
| `start_task` | 1 | 256 | 创建 `led`/`micro_ros`/`sensor`/`storage`/`motor_rt` 五任务后自删除 |
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发，10ms bridge timer（100Hz）按域节拍发布：运动/IMU 数据 100Hz、状态 20Hz、拓扑 1Hz、温度 2Hz；断连自动重建 |
| `led` | 1 | 256 | 每 50ms 读取 micro-ROS 链路状态并作为 WS2812 唯一运行期写入者；红/绿闪/蓝 |
| `sensor` | 2 | 512 | 500ms 延时 → 加热 PWM → BMI088 单次初始化；失败时每 1s 降级重试，成功后由约 100Hz ACC DRDY 触发采样/Mahony/PID（IMU 非关键路径：低于 motor_rt，栈台架 TBD-RT-01 复测） |
| `storage` | 1 | 1024 | W25Q64 唯一所有者：W25 参数持久化（boot 读双 sector blob；设备写事务去抖 500ms 自动落盘 + `system.save` 脉冲，见「W25Q64」节） |
| `motor_rt` | 3 | 1024 | FDCAN1/2/3 初始化；每 1ms 有界服务（RX drain/内核服务/SYNC 心跳/TX 路由），每 20ms 控制边界消费完整快照与配置 FIFO |

## 代码组织

业务按“大任务”分域，ROS 实体不堆在 `ros_bridge`：

| 模块 | 职责 |
|---|---|
| `ros_bridge` | agent 通信生命周期：node/executor/timer，断连整体重建；定时器依次轮询各域桥 |
| `imu_bridge` | `micro_ros/imu/data` 与 `micro_ros/imu/temp` 的缓冲/填充/发布 |
| `motor_bridge` | ID 化下行订阅（command/trajectory/policy）+ 上行发布（motion/status/topology/events/trajectory_state/diagnostics） |
| `motor_rt_task` | 唯一电机实时任务：1ms 服务 + 20ms 控制边界，内核唯一调用者（消费快照/轨迹/策略/配置） |
| `session_sync` | 会话边沿/电平→内核授权的纯派生（20ms 边界调用，host 可测） |
| `storage_projection` | 内核参数视图↔W25 blob 纯投影（持久化字段语义，host 可测） |
| `heater` | TIM3 加热 PWM 所有权 + 纯 PID（host 可测） |
| `lib/fdcan` | FDCAN 初始化/中断回调/每总线 SPSC RX ring/P0-P1-P2 TX 调度 |
| `lib/imu` | BMI088 驱动/通信层、SPI2 阻塞传输、Mahony/SI 聚合（单一 IMU 域） |
| `lib/motor_protocol` | adapter 接缝 + vaxor/CERB 编解码（厂家字节唯一归属地） |
| `lib/motor_control` | ID 驱动运行时内核：注册表/型号目录/九态生命周期/安全级联/配置事务 |
| `lib/control_mailbox` | ID 完整命令快照、多点轨迹、逐电机策略、低频配置 FIFO 与连接/授权边沿（纯逻辑） |
| `lib/board` | 板级共享常量（ISR syscall 阈值），板级库经它免依赖 FreeRTOS |
| `lib/dma` | DMA1/BDMA 控制器级资源（时钟、NVIC、占用登记）；设备配置由各设备库自持 |
| `lib/w25q64` | OCTOSPI2 W25Q64 驱动；唯一所有者为 `storage_task`（见「W25Q64」节） |
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
│   ├── storage_task.h         # W25Q64 参数持久化任务接口（storage_task_start）
│   ├── param_bridge.h         # ROS 2 参数服务器桥接口
│   └── motor_rt_task.h        # 电机实时任务接口（motor_rt_task_start）
├── lib/                       # 板级/协议库（PlatformIO 自动发现）
│   ├── imu/                   # BMI088 驱动 + SPI2 阻塞传输 + Mahony + SI 聚合
│   ├── fdcan/                 # 三路 FDCAN/中断回调/SPSC RX ring/P0-P1-P2 TX 调度
│   ├── motor_control/         # ID 驱动运行时内核（host 可测）
│   ├── control_mailbox/       # 固定容量 mailbox 与边沿锁存（host 可测）
│   ├── motor_protocol/        # adapter 接缝 + vaxor/CERB 编解码（纯逻辑）
│   ├── dma/                   # DMA1/BDMA 控制器级资源（设备配置由各库自持）
│   ├── w25q64/                # OCTOSPI2 W25Q64 驱动（storage_task 独占访问）
│   └── ws2812/                # WS2812 彩灯驱动（SPI6 MOSI=PA7）
└── src/
    ├── main.cpp               # Arduino 入口与 start_task 编排
    ├── micro_ros_task.cpp     # micro-ROS agent 状态机
    ├── led_task.cpp           # WS2812 状态显示任务 + host 可测的颜色策略
    ├── ros_bridge.cpp         # node/executor/timer 生命周期 + 域桥编排
    ├── imu_bridge.cpp         # IMU/温度发布（micro_ros/imu/data、temp）
    ├── motor_bridge.cpp       # 电机 ID 化话题桥（订阅/发布，见"联调"节）
    ├── param_bridge.cpp       # ROS 2 参数服务器桥（静态量 + 运维动作）
    ├── sensor_task.cpp        # 传感器任务：EXTI/信号量/温度 PID/加热 PWM
    ├── storage_task.cpp       # W25Q64 唯一所有者：boot 加载双 sector blob + save 落盘
    ├── motor_rt_task.cpp      # 电机实时任务：1ms 服务 + 20ms 控制边界
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

`storage_task`（P1/栈 1024 words）是 W25Q64 的唯一所有者。boot 同步
`w25q64_init()` 并读双 sector A/B（地址 0x0/0x1000），选 CRC 有效且 seq
最新者；无芯片/ID 不符静默降级为无持久化，不阻断启动。blob v2 =
32B 头 + 64×48B 记录（3104B）；v1 旧布局记录 32B，加载时自动就地迁移。
保存有两个触发源：设备写事务成功后去抖 500ms 自动落盘、`system.save`
手动脉冲。save 总擦写非活动 sector，擦/写后读回 memcmp 校验，失败不翻转
活动 sector（旧槽仍可启动）。擦写阻塞可达 ~1s，只在 storage 任务内执行，
IWDG 由 motor_rt 独立喂。

## 与 micro_ros_agent 联调

```bash
source /opt/ros/humble/setup.bash
source ../micro_ros_agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

建链后可见电机话题（消息均属 `can_bridge_msgs`，源在
`../micro_ros_agent/src/can_bridge_msgs`）：

订阅（下行，3 个话题）：

- `/motor_command_array`（`MotorCommandArray`，best-effort）：`motor_id +
  position/velocity/effort` 完整快照；NaN/Inf、未知或重复 `motor_id` 整体拒绝。
  缺席的受控电机立即软停，禁止沿用旧目标。
- `/motor_trajectory`（`MotorTrajectory`，best-effort）：多点轨迹，见下节。
- `/motor_policy`（`MotorPolicyArray`，reliable）：逐电机 enable/kp/kd，见下节。

### 多点轨迹（`/motor_trajectory`）

轨迹是完整快照的**时序补充**通道，不是替代品：快照给"现在在哪"，轨迹给
"未来一段时间怎么走"。MCU 在 20ms 控制边界按 `time_from_start_ms` 做段内
**线性插值**，速度取段斜率（不直通路点里的 `velocity` 字段——上层常把它填 0，
直通会让 MIT 律的 Kd 朝 0 速刹车），前馈 `effort` 段内保持左端点值。时间超出
末点即**钳末点保持，绝不外推**。轨迹起点是 MCU 本地受理 tick，消息里的
`sync_time_ns` 只用于跨系统日志对齐（ARD §12）。

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
- **容量**：单条轨迹最多 256 个路点 / 64 台电机（`MOTOR_CONTROL_MAX_TRAJECTORY_
  POINTS`），更长的时间线由上层分段链式下发（段边界速度不连续，MIT 的 Kd 平滑）。
  静态占用约 31 KB（ROS 接收体 + mailbox 双缓冲 + 转换中间 + 内核锁存，20 B/点），
  改小上面两个常量是唯一的省 RAM 手段。**RAM 口径**：`.data`+`.bss` 全部落在
  320 KiB 的 RAM_D1 上，实测占用 **63.0%**（206,324 B / 327,680 B）。PlatformIO
  报的 35.7% 分母是整芯片 577,536 B，会把紧张度藏起来——判断余量要用 RAM_D1
  口径。参数槽另占 DTCM 堆（见「参数」一节）。

### 故障清除（`system.clear_fault`）

参数写"电机 id"触发，只清**可清的**故障，按**成因**区分：

- **身份冲突锁存**（同 `motor_id` 跨总线/协议身份不符，`CONFIG_CONFLICT`）：
  ARD §9.3 刻意不可解除，**拒绝**（复位只能整机重启）。
- **设备参数未初始化**（`last_error=40/41`，同为 `CONFIG_CONFLICT` 生命周期）：
  改走一次 `REFRESH_PARAMS` 事务重跑回读与比对。真不一致仍判 `MISMATCH` 保持
  锁存——清除只负责"允许重走授权链"，**不伪造一致**。
- **厂家故障**：发厂家清错帧（CERB 的 MIT 控制字 bit5）。是否真清掉由设备后续
  状态帧的 `fault_active` 判定，走既有 `FAULT → READY_DISABLED` 路径，无新确认
  机制。Vaxor 控制帧没有清错位 → 内核回 `UNSUPPORTED`（不猜厂家字节）。
- 清错帧是控制类帧，**仅非运动态受理**（与设备侧限位写同口径）；清完仍须显式
  **重新武装**（`/motor_policy` 的 `enabled=true`，且代数须前进）+ 新的完整
  快照/轨迹才可能回运动（ARD §36）。

只读完成序号 `system.clear_fault_seq`，用法同 `set_zero_seq`（写脉冲 → 轮询
seq 收敛，不要只看 `last_error`）。**代价**：系统参数 8 → 10。

### 参数树放静态量，动态量走 `/motor_policy`

2026-09-18 起参数面按「静态 / 动态」重新划界——**参数面 = 静态配置与运维动作，
话题 = 动态控制与状态**：

- **参数树**（每电机 4 项 + `system.*` 约 10 项）：`m<id>.pos_min/pos_max/`
  `vel_limit/torque_limit` 与 `system.*`。限位/限速/限流「只初始化一次」，
  正是参数的本职。
- **`/motor_policy` 话题**：`enable`/`kp`/`kd`。这三项变化快，且 ARD §19.4 本就
  禁止它们进 100Hz 命令快照。**稀疏语义**：只影响列出的电机，其余保持原状。
  **代数门**：「未武装 → 武装」要求 `policy_generation` 严格前进，挡住「周期
  重发同一条策略」在故障后自动复权（挡不住每次都递增代数的发布者——那与人工
  逐次授权不可区分，ARD §36 的「显式」在机制层面到此为止）。
- **为什么改**：rclc 参数服务器每槽约 269 B（审计实测 `sizeof(Parameter)=112`、
  `descriptor=68` + 50 B 名），且 init 时**一次性全部分配**。7 项/台在 48 台上限
  下要 346 槽 ≈ 89 KB，而参数槽所在的 DTCM 堆池只有约 116 KiB（任务栈另占
  44.5 KiB）。4 项/台时 `PARAM_BRIDGE_MAX_PARAMS = 208` 即可覆盖，占用约 56 KB。
- `param_field_t` 仍是 7 项且**不要改**——它是 Flash blob 的 `present` 位图契约；
  退役的三项只是不再被注册，记录布局不变、无需迁移。

下行配置通道（`motor_enable`/`motor_policy`/`motor_config`/`motor_bind` 与
`motor_config_result`）已删除：型号绑定由内核读设备自报身份自动完成
（2026-09-16 起，静态拓扑表已删）——发现建档/重连后经事务读 CERB SDO 0x1008
（厂家名称）并与型号目录整串精确匹配；vaxor 无身份读对象，回退到"协议内唯一
型号"。未命中目录的设备保持未绑定不可控。授权/增益/限位取内核安全默认（建档
`policy_enable` 默认 true、全局授权 mailbox 引导豁免、增益回退 profile
`default_kp/default_kd`、CERB 限位在驱动器侧配置）。

设备侧管理对象（`0x607D:01/:02` 位置限位、`0x6080` 电机端最大转速、
`0x2538` 电机端峰值电流）的写 + 读回核验留在内核/adapter 层（写仅限未运动
状态，读回不一致收敛 `UNCONFIRMED`），ROS 侧入口是参数服务器（见下）。

**参数单位（2026-09-17）**：ROS 参数对外用工程单位——位置**度**、速度**度/秒**、
力矩**安培**（设备里的对象就是 cnt / RPM / mA）；rad、rad/s、N·m 只留在内核与
W25 记录里，换算在 `param_bridge` 边界做一次（`param_units.h`）。力矩用 A 是因为
设备侧权威对象是 `0x2538`（电机最大电流），折成 N·m 要过一次系数，而该系数存在
未决争议（见 `docs/architecture/motor-runtime-decision-log.md` §6.4/§6.6）。

**外部写入不得超过固定硬件参数表（2026-09-17）**：型号目录
（`lib/motor_control/src/motor_control_profile.c`，数值取自规格书
`docs/CE-RB-R{58,48}-101-*-参数规格.pdf`）是唯一基准。写入按四维取更严者钳制——
扭矩/电流（峰值扭矩 45/11 N·m、峰值电流 10.8/3.2 A）、速度（≤ 额定 π rad/s）、
位置（≤ ±π）。**限值对象 0x2538 的换算系数用规格书峰值对**（45/10.8、11/3.2），
所以"写峰值扭矩"正好写出"峰值电流"；反馈扭矩估计仍走 Kt×减速比×效率。

**设零点（2026-09-17）**：参数服务器 `system.set_zero = <电机 id>` 触发，内核写
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
`system.refresh_seq`（每次回读收尾 +1），不要只看 `last_error`（分不清轮次）。

**设备参数以驱动板为准（2026-09-17）**：上电/重连绑定完成后，内核自动对
CERB 发一次 `REFRESH_PARAMS` 回读四个管理对象，与 W25 里的固件记录**逐槽位
比设备端原值**（cnt/RPM/mA；比关节侧 SI 会在换算系数配错时两边一起错）：

- 一致（或本机无该台记录）→ 初始化通过，正常可控；
- **初始化未完成一律不可使能**（`PENDING` 回读未收尾 / `MISMATCH` 不一致 /
  `UNVERIFIED` 回读失败）：进 `CONFIG_CONFLICT`、拒绝 `SET_ENABLE`、不出运动帧，
  `system.last_error=40`（不一致）或 `41`（未核对）+ `last_error_motor` 指出是
  哪一台；从参数页写回该台参数（写事务同时更新设备与记录）或重连重读即可恢复；
- **运行中回读发现不一致 → 软停**（阻尼帧、保留使能），不硬失能；
- 自动回读失败会退避重试（≤3 次、间隔 500ms；读是只读事务，重试无副作用），
  用尽才判 `41`，避免一次总线抖动就让该台使能不了；
- 设备侧写成功后记录同步落盘（去抖 500ms）：写就是设备和驱动板一起写。
- W25 blob 升 v2（多 4 个设备端原值），v1 就地迁移，不丢已存参数。

发布（上行，6 个话题；均为 ID 化，缺失浮点 = NaN）：

- `micro_ros/motion_states`（`MotorMotionStateArray`）：运动状态（100Hz，
  由 10ms bridge timer 驱动）。
- `micro_ros/motor_status_array`（`MotorStatusArray`）：诊断（生命周期/故障码/
  保护位/事务状态）。
- `micro_ros/motor_topology`（`MotorTopology`，1s 节流）：发现式拓扑
  （`motor_id`/总线/协议/型号/能力/冲突标志）。
- `micro_ros/motor_events`（`MotorEvent`）：生命周期/安全事件流（含身份识别
  的 PROFILE_BOUND / PROFILE_REJECTED）。带 `event_seq`（单调递增，相邻跳号即
  说明事件环满过；累计丢弃数见 diagnostics）。
- `micro_ros/trajectory_state`（`MotorTrajectoryState`，10Hz）：轨迹执行状态——
  `trajectory_id`、active/completed、已执行/总时长、受控集合。轨迹是 best-effort
  下发的，没有它就只能靠猜「我发的计划受理了吗、跑到哪了」。
- `micro_ros/diagnostics`（`MotorDiagnostics`，1Hz）：固件级计数器一次性曝光
  （帧计数/解码错误/快照与配置拒绝/事件与 TX 丢弃/P0 重试/轨迹与策略收支/
  每总线 ring 溢出/堆余量）。这些此前只存在于芯片里，实时性与总线容量的问题
  只能靠 JTAG 定位。

**三份上行状态不重叠**：topology = 身份（谁在、哪条总线、什么协议、型号、能力、
冲突、`schema_version`）；status = 状态与电气（`lifecycle`、在线、温度/电流/电压、
故障码、保护/限位位）；motion_states = 运动学（位置/速度/力矩、`state_age_ms`、
`target_source`）。`target_source` 让上层看见「我的快照为什么被忽略」。

`motor_id ↔ joint_name` 的映射、方向与零位是**上层职责**（上层迁移进行中，
见 HANDOFF「剩余工作」）；固件不做 name-to-ID 适配。

## 控制安全状态与恢复语义

- `motor_rt_task` 每 1ms 服务、每 20ms（50Hz）控制边界消费一次完整快照；
  内核是唯一生命周期裁决者，任务层是唯一正常 FDCAN 发送者（P0/P1/P2 分级）。
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
  50Hz，并记录 motor_rt cycle WCET 与周期 jitter。
- [ ] 停止刷新快照，验证 30ms 边界进入默认 `kd` 阻尼软停、100ms 边界协议级
  失能并清授权；单电机从快照缺席，验证仅该电机软停、其他电机不受影响。
- [ ] Agent 断连/重连，验证断连立即失能清授权、清旧命令；重连后内核重新读
  设备自报身份（CERB SDO 0x1008）并重绑定收敛 READY_DISABLED，恢复运动必须
  显式 SET_ENABLE(true)（编译期冻结语义）+ 新完整快照，期间不得自行恢复或重放。
- [ ] 身份识别台架核对：`ros2 topic echo micro_ros/motor_topology` 中各 CERB
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
- [ ] 断开/恢复 BMI088、阻塞 SPI 或停止 DRDY，验证 2ms transport bound、每秒
  初始化重试、heater off 与双缓冲不撕裂。

## 已知问题与后续

- BMI088 收发固定走阻塞 `HAL_SPI_TransmitReceive`，当前不占用 DMA stream；
  若后续切换 DMA，设备句柄/回调仍归 `lib/imu`，控制器资源须向 `lib/dma` 登记。
- 无 agent 时节点不注册、固件在 `kWaitAgent` 轮询，属正常。
- `loop()` 被 STM32FreeRTOS 的 idle hook 调用，只能放非阻塞代码。
- 电机运行时已完成 ID 驱动化（ARD-0007，剩余项见 HANDOFF「剩余工作」）；9 个
  `can_bridge_msgs` 消息（5 个顶层消息 + Target/Entry 子消息）由
  `../micro_ros_agent/src/can_bridge_msgs` 唯一维护，
  固件侧只通过 `extra_packages/can_bridge_msgs` 符号链接复用。
- CERB R48/R58 的 MIT 范围配置为位置 `±π rad`、速度 `±π rad/s`、
  力矩 `±11/±45 Nm`，必须与真机参数设置页一致；`motor_rt_task` 每 20ms 发送
  `0x080` SYNC 维持设备 20ms 状态心跳（总线检测）。
- RX burst 下的实际 motor_rt WCET/jitter、ring 容量余量及 30/100ms 动作边界
  尚需按上述清单在 `dev-real` 无负载测量；host/固件编译不能替代真机时序证据。
