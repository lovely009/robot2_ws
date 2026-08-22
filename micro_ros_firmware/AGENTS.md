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
- CAN1 调试桥话题 `micro_ros/can1/send` / `micro_ros/can1/receive` 由
  `ENABLE_CAN1_DEBUG_BRIDGE` 宏控制（`platformio.ini` 默认开启，类型
  `can_bridge_msgs/msg/CANFrame`）。
- JointState 话题 `/joint_commands`（`sensor_msgs/msg/JointState`）由
  `ENABLE_MOTOR_COMMAND_BRIDGE` 控制（默认开启）；固件只内置
  `left_shoulder_yaw_joint`（RP）与 `left_wrist_yaw_joint`（vaxor，节点
  ID=2，当前 CAN1 实际接入）两个默认测试电机，完整拓扑待参数服务。
- vaxor 模式由 JointState 字段自动选择：只发 `position`=位置模式、只发
  `velocity`=速度模式、只发 `effort`=MIT 前馈；模式切换先发停机帧再启动。
- 反馈话题 `micro_ros/JointState`（`sensor_msgs/msg/JointState`）发布默认
  测试电机解析后的位置/速度/力矩。
- 状态灯：上电/未连接=红，连接成功=绿 1Hz 闪，断连=蓝（`main.cpp` 状态机）。
- FDCAN1/2/3 = PD0/PD1、PB5/PB6、PD12/PD13（FD + BRS，标称 1M/数据 5M）。

## 高频踩坑（务必先看）

- **IMU 初始化只能在任务里做**：`BMI088_delay_ms` 用 `vTaskDelay`，调度器未启动
  时调用会非法；不要在 `setup()`/`systemInit()` 里调 `imu_init()`。
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
- **HAL 模块**：DMA/SPI/TIM 由 STM32duino 默认使能；FDCAN 需要显式
  `-DHAL_FDCAN_MODULE_ENABLED`（`platformio.ini` 已加）。
- **调试桥限制**：`CANFrame.id` 是 `uint16`，29 位扩展 ID 不能完整回显；
  RX/TX 元素已扩到 64B，RP 广播与 vaxor 12/20B 帧可正常承载。
- **改 HAL 构建宏后必须 `pio run -t clean` 全量重建**：SrcWrapper 的
  `stm32yyxx_hal_fdcan.c.o` 不会因 build flag 变化自动重编。
- **FreeRTOS 下 `loop()` 只能非阻塞**：业务放 `sensor`/`control`/`micro_ros` 任务。
- **任务栈**：`micro_ros` 8192、`sensor`/`control` 1024（单位：字）；调栈后同步改 README。
- **类型坑**：本平台 `int32_t` 是 `long`，模板推导（如 `std::clamp`）须显式
  指定类型参数。
- **git pull 分支偏离**：残留 `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/build/`
  会让 download_dev_environment 的 `git pull` 失败 -> 删除该 build/ 目录后重跑。

## 结构（简）

- `platformio.ini`：构建配置（USB CDC 宏 + `HAL_FDCAN_MODULE_ENABLED`）
- `boards/stm32h723vgt6.json`：变体/`variant_h`/`-DARDUINO_GENERIC_H723VGTX`/VID-PID
- `src/main.cpp`：`systemInit` + `start_task`（创建 `micro_ros`/`sensor`/`control`）
- `src/ros_bridge.cpp`：node/executor/timer 生命周期，编排各域桥
- `src/imu_bridge.cpp`：IMU/温度发布（`micro_ros/imu/data`、`micro_ros/imu/temp`）
- `src/can_debug_bridge.cpp`：CAN1 调试桥（`micro_ros/can1/send`/`receive`）
- `src/motor_bridge.cpp`：电机命令/反馈桥（`/joint_commands`、`micro_ros/JointState`）
- `src/sensor_task.cpp`：传感器任务（EXTI/信号量/温度 PID/加热 PWM）
- `src/control_task.cpp`：控制任务（三路 FDCAN 初始化；官方测试帧默认关闭，
  由 `ENABLE_CANFD_DEMO_LOOP` 宏开启；JointState → 默认测试电机协议帧）
- `src/system_clock.cpp`：覆盖 `SystemClock_Config`（HSE/HSI48/SPI2/SPI6 时钟树）
- `lib/imu`：BMI088 驱动/通信层/Mahony/SI 聚合（依赖 `lib/dma`）
- `lib/dma`：DMA1 配置 + `HAL_SPI_MspInit` + DMA IRQ 转发（当前 SPI2 专用）
- `lib/fdcan`：FDCAN 初始化/过滤/收发/中断回调/接收快照（官方例程移植）
- `lib/motor_protocol`：电机协议编解码（RP/25-30mm，ros2_ws
  `motor_drive/protocol.cpp` 移植；同一文件内分 RP/vaxor 两部分，
  `-fno-exceptions` 下异常改返回值）
- `lib/ws2812`：板载状态灯（SPI6 MOSI=PA7）

`src/` 只放任务编排与 ROS 桥；硬件驱动/协议库在 `lib/`，PlatformIO 会自动
发现并链接。`lib/imu` 依赖 `lib/dma`（见 `lib/imu/library.json`），新增跨库
依赖时同步补 `library.json`。

## 代码规范

- C++17 + C（STM32duino HAL）混合；2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类）、`k` 前缀常量。
- 中文注释写意图（数据流/单位/安全边界）；micro-ROS / ROS 2 API 保留英文。
- 硬件映射集中：引脚/话题/节点名在 `main.cpp` 或各域桥/任务文件顶部常量，
  不散落。
- 业务域桥只通过 `can1_rx_snapshot()` 取 CAN1 帧，不直接访问 `can_bsp`
  内部缓冲；新增 CAN 消费方时沿用“记序号、比变化”的轮询模式。
