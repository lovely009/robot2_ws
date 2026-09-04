# micro_ros_firmware —— Agent 指南

STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS，serial over
USB CDC 进 ROS 2 humble）。当前业务为 **BMI088 IMU** + **三路 FDCAN**：官方
`CtrBoard-H7_IMU_TempCtrl` / `CtrBoard-H7_FDCAN` 底层功能移植 + Mahony/ROS 发布增强。
**权威文档：`README.md`**，改动前先读；本文件只列 Agent 即用要点。

## 构建与烧录（本机）

`pio` 不在 PATH，用全路径：

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 编译
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录
~/.platformio/penv/bin/pio device monitor # 串口监视 115200（USB CDC 下波特率无效）
```

## 关键决策（改动前确认，勿破坏）

- `framework = arduino`：`micro_ros_platformio` 官方只支持 Arduino 框架。
- `board_microros_distro = humble`、`board_microros_transport = serial`
  （物理层 USB CDC，agent 以 `/dev/ttyACM0` 对接）。
- `STM32FreeRTOS`：调度器接管 SysTick；micro-ROS 与 IMU 都是独立任务。
- USB 引脚 **PA11=USB_DM / PA12=USB_DP**；状态 LED 是 **WS2812**（SPI6 MOSI=PA7）。
- 节点名 `micro_ros_node`，话题 `micro_ros/imu/data`、`micro_ros/imu/temp`
  （在 `src/imu_bridge.cpp` 顶部）。
- JointState 话题 `/joint_commands`（`sensor_msgs/msg/JointState`）常驻编译；固件在 FDCAN1 固定内置
  vaxor ID 1/2 与 CREB R58/R48 ID 1/2 四个默认测试电机，不支持动态拓扑配置。
  CREB 的 CAN ID 区间与 vaxor 不同，相同节点号可在同一总线共存。
- `micro_ros/motor_enable` 是显式运动授权：`false` 清空 pending 命令并失能；
  断连重连后必须先发 `true`、再发新命令，禁止重放旧命令。
- `/joint_commands` 只负责位置/速度/力矩目标，不再推断或切换模式。
  `micro_ros/motor_mode`（`MotorMode`）独立切换 vaxor 的 MIT/速度/位置模式；
  模式改变会先停机并清除旧目标，必须再发新命令。CREB 固定 MIT，拒绝其他模式。
- `micro_ros/motor_gains`（`MotorGains`）按关节更新 `kp/kd`，只影响后续命令，
  不刷新命令时效、不触发运动；断连保留 RAM 配置，MCU 重启恢复固件默认值。
- CREB 仅使用 MIT：R58/R48 的 P/V/T Max 为 `π rad`、`π rad/s`、
  `45/11 Nm`；使能后等待至少 100ms 才发送目标。control task 每 10ms 在
  CREB 总线上发送一次标准 `0x080` SYNC，触发 `0x300 + ID` 反馈。
- 反馈话题 `micro_ros/JointState`（`sensor_msgs/msg/JointState`）发布默认
  测试电机解析后的位置/速度/力矩。
- 状态话题 `micro_ros/motor_status`（`can_bridge_msgs/msg/MotorStatus`）逐电机
  发布 vaxor 20 字节状态帧中的电压、故障码、电机状态、校准状态、模式和类型；
  12 字节普通反馈不产生状态快照，每个新状态快照只发布一次。
- 状态灯由 `led_task` 独占：上电/未连接=红，连接成功=绿 1Hz 闪，断连=蓝；每 50ms 轮询。
- FDCAN1/2/3 = PD0/PD1、PB5/PB6、PD12/PD13（FD + BRS，标称 1M/数据 5M）。

## 高频踩坑（务必先看）

- **IMU 初始化只能在任务里做**：调用 `imu_try_init()`，每次只尝试一次；失败后
  sensor task 每 1s 降级重试。`BMI088_delay_ms` 用 `vTaskDelay`，调度器未启动
  时调用会非法；不要在 `setup()`/`systemInit()` 初始化 IMU。
- **IMU SPI 失败必须有界**：每字节 HAL timeout 为 2ms，首个失败锁存至当前
  初始化/采样结束；失败时 heater 置 0。首个有效样本必须由真实 ACC DRDY 唤醒，
  sensor/ROS 之间只传递双缓冲 snapshot 副本，不暴露内部源指针。
- **EXTI 用框架 API**：`sensor_task.cpp` 通过 `attachInterrupt(PE10, ...)` 挂
  ACC_INT，不要手写 `HAL_GPIO_EXTI_Callback`/`EXTI15_10_IRQHandler`。
- **TIM3 用 HardwareTimer**：`sensor_task.cpp` 里 `HardwareTimer` 封装 PWM，
  不要再手写 `TIM_HandleTypeDef` 初始化。
- **DMA 保留但未用于收发**：`lib/dma/src/dma.c` 配置 DMA1_Stream0/1 并链接 SPI2，
  BMI088 仍走阻塞 `HAL_SPI_TransmitReceive`（与官方一致）；改 DMA 传输时
  必须给 IMU 任务补唤醒语义。
- **USB CDC 宏**：H723 只有 `USB_OTG_HS`，必须保留两个
  `PIO_FRAMEWORK_ARDUINO_*` 构建宏；删掉则 USB 不枚举。
- **时钟覆盖**：`src/system_clock.cpp` 强定义覆盖 WEAK 符号；PLL1Q=2 提供
  SPI2 240MHz，PLLQ 改开会连带改变 SPI2 速率。
- **FDCAN 时钟**：`PLL2Q=80MHz` 独立供 FDCAN；不要为 FDCAN 改 PLL1Q，
  否则会连带改变 SPI2 速率。
- **HAL 模块**：DMA/SPI/TIM 由 STM32duino 默认使能；FDCAN 与 W25Q64 OSPI 分别
  需要显式 `-DHAL_FDCAN_MODULE_ENABLED`、`-DHAL_OSPI_MODULE_ENABLED`
  （`platformio.ini` 已加）。任一 HAL 构建宏变更后必须执行
  `pio run -t clean` 全量重建。
- **W25Q64**：保留 `lib/w25q64` 及 geometry host test，但不执行启动探测，禁止在上电路径自动擦写 Flash。
- **FDCAN RX 归属**：ISR 只写静态队列（CAN1=32、CAN2/3=8，满时 drop-oldest），
  control task 是唯一消费者，每周期每总线最多 drain 8 帧；ROS 桥只能读取已解码的
  电机反馈/状态，不得让 bridge 或第二任务直接消费主队列。
- **FDCAN IRQ**：优先级固定为 5，必须满足
  `priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 才能调用
  `xQueue*FromISR`；队列只在 `bsp_can_init()` single-init，不在重连时重建。
- **改 HAL 构建宏后必须 `pio run -t clean` 全量重建**：SrcWrapper 的
  `stm32yyxx_hal_fdcan.c.o`、`stm32yyxx_hal_ospi.c.o` 不会因 build flag 变化自动重编。
- **FreeRTOS 下 `loop()` 只能非阻塞**：业务放 `sensor`/`control`/`micro_ros` 任务。
- **任务栈/周期**：`start_task` P1/256 words；`sensor` P4/1024 words（DRDY）；
  `control` P3/1024 words（10ms/100Hz）；`micro_ros` P2/8192 words（20ms
  bridge timer）。调栈/周期后同步改 README。
- **控制发送唯一性**：ROS callback 只写完整 ID 快照 latest-wins mailbox；control task
  是唯一正常 FDCAN 发送者。
  100ms soft stop 使用 MIT 默认 `kd` 阻尼，300ms hard stop；vaxor/CREB
  各自发送 stop/disable，其他新鲜电机继续运行。
- **类型坑**：本平台 `int32_t` 是 `long`，模板推导（如 `std::clamp`）须显式
  指定类型参数。
- **git pull 分支偏离**：残留 `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/build/`
  会让 download_dev_environment 的 `git pull` 失败 -> 删除该 build/ 目录后重跑。
- **自定义消息只用同仓库符号链接**：
  `extra_packages/can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs`。
  全新 clean bootstrap 仍有下载依赖/ROS 2 源的网络要求；常规验证不要运行
  `clean_microros`，使用缓存依赖执行 `pio run -t clean && pio run`。

## 结构（简）

- `platformio.ini`：构建配置（USB CDC 宏 + FDCAN/OSPI HAL 宏）
- `boards/stm32h723vgt6.json`：变体/`variant_h`/`-DARDUINO_GENERIC_H723VGTX`/VID-PID
- `src/main.cpp`：Arduino 入口与 `start_task`（创建 `led`/`micro_ros`/`sensor`/`control`）
- `src/micro_ros_task.cpp`：micro-ROS agent 连接状态机
- `src/led_task.cpp`：WS2812 状态显示任务 + host 可测的状态到颜色映射
- `src/ros_bridge.cpp`：node/executor/timer 生命周期，编排各域桥
- `src/imu_bridge.cpp`：IMU/温度发布（`micro_ros/imu/data`、`micro_ros/imu/temp`）
- `src/motor_bridge.cpp`：电机命令、模式、增益与反馈/状态 ROS 桥
- `src/sensor_task.cpp`：传感器任务（EXTI/信号量/温度 PID/加热 PWM）
- `src/control_task.cpp`：控制任务（三路 FDCAN 初始化；官方测试帧默认关闭，
  由 `ENABLE_CANFD_DEMO_LOOP` 宏开启；JointState → 默认测试电机协议帧）
- `src/system_clock.cpp`：覆盖 `SystemClock_Config`（HSE/HSI48/SPI2/SPI6 时钟树）
- `lib/imu`：BMI088 驱动/通信层/Mahony/SI 聚合（依赖 `lib/dma`）
- `lib/dma`：DMA1 配置 + `HAL_SPI_MspInit` + DMA IRQ 转发（当前 SPI2，预留其他外设复用）
- `lib/fdcan`：FDCAN 初始化/过滤/收发/中断回调/三路静态 RX 队列（官方例程移植）
- `lib/motor_control`：100/300ms timeout、故障/恢复及 vaxor/CREB action 状态机
- `lib/control_mailbox`：ID 完整命令快照、低频配置 FIFO 与授权/连接边沿
- `lib/motor_protocol`：vaxor/CREB 电机协议编解码（从 ros2_ws
  `motor_drive/protocol.cpp` 移植，`-fno-exceptions` 下异常改返回值）
- `lib/ws2812`：板载状态灯（SPI6 MOSI=PA7）
- `lib/w25q64`：OCTOSPI2 W25Q64 驱动；运行期访问必须指定单一所有者，不能由多个
  任务并发使用同一 OSPI 句柄

`src/` 只放任务编排与 ROS 桥；硬件驱动/协议库在 `lib/`，PlatformIO 会自动
发现并链接。新增跨库依赖时同步补对应的 `library.json`。

## 代码规范

- C++17 + C（STM32duino HAL）混合；2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类）、`k` 前缀常量。
- 中文注释写意图（数据流/单位/安全边界）；micro-ROS / ROS 2 API 保留英文。
- 硬件映射集中：引脚/话题/节点名在 `main.cpp` 或各域桥/任务文件顶部常量，
  不散落。
- FDCAN ISR → 静态队列 → control task sole consumer 是固定所有权边界；业务域桥
  只能通过 `control_task_get_joint_state()` / `control_task_get_motor_status()` 读取
  已解码副本，不得直接访问 `can_bsp` 内部队列或 HAL 发送接口。
- `dev-real` 交接只能做无负载真机验证且硬件急停必须可用；必须实测 RX burst 下
  control WCET/jitter、100Hz 周期、100/300ms 边界和 overflow/backlog。当前机构
  无抱闸，软件 stop 不能替代硬件急停、机械限位或供电隔离。
