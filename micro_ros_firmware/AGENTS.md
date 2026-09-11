# micro_ros_firmware —— Agent 指南

STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS，serial over
USB CDC 进 ROS 2 humble）。当前业务为 **BMI088 IMU** + **三路 FDCAN**：官方
`CtrBoard-H7_IMU_TempCtrl` / `CtrBoard-H7_FDCAN` 底层功能移植 + Mahony/ROS 发布增强。
**权威文档：`README.md`**，改动前先读；本文件只列 Agent 即用要点。

> **电机运行时与后续验收：**
> [`docs/handoffs/HANDOFF-motor-runtime-refactor.md`](docs/handoffs/HANDOFF-motor-runtime-refactor.md)。
> 固件侧 ID 驱动运行时已经完成；剩余工作是上层 `motor_hardware_interface`
> 的 joint↔motor_id 映射与真机台架验证。上层迁移启动时先更新同仓库
> `../micro_ros_agent/` 的权威消息包，再修改 `ros2_ws/`。
> 运行时设计以 HANDOFF、`docs/architecture/ARD-0007-micro-ros-motor-runtime.md` 和
> ADR 0007 为准；不得恢复旧四电机表或 `JointState` 命令路径。

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
- 电机接口为 ID 化完整快照（ARD-0007）：下行 `/motor_command_array`
  （`can_bridge_msgs/MotorCommandArray`，最多 64 个 `motor_id + pos/vel/effort`，
  best-effort）+ `micro_ros/motor_enable`（全局授权）；上行
  `micro_ros/motion_states`（`MotorMotionStateArray`，缺失量 NaN）、
  `micro_ros/motor_status_array`、`micro_ros/motor_topology`（1s 节流）、
  `micro_ros/motor_events`、`micro_ros/motor_config_result`。无 `joint_name`、
  无固定电机表；电机经反馈帧自主发现（vaxor 自主上报 / CERB SYNC 上报），
  型号由上层经 Profile 绑定接口确认（型号目录无 Vaxor-80）。
- `micro_ros/motor_enable` 是显式全局运动授权：`false` 清空 pending 命令并协议级
  失能；断连重连后必须先发 `true`、再发新完整快照，禁止重放旧命令。
  `micro_ros/motor_policy`（`MotorPolicy`）携带逐电机 `SET_ENABLE`：OFFLINE/故障
  清除后运动不会自动恢复，必须显式重发该电机的 Enable。
- `micro_ros/motor_policy` 还携带 `SET_GAINS`（按 `motor_id` 钳到 Profile 上限，
  只影响后续命令、不刷新命令时效）与 `SET_POSITION_LIMITS`（Vaxor 软件限位，
  打包前钳制）。`micro_ros/motor_config`（`MotorConfig`，带 `request_id`）承载
  SET_MODE 事务（vaxor stop/start 两阶段、结果收敛 UNCONFIRMED；CERB 恒 MIT）
  与 CERB 限位（厂家管理对象未决，当前一律 UNSUPPORTED），结果经
  `micro_ros/motor_config_result` 异步返回。
- 反馈模型（2026-09-05 核实）：vaxor 12B 回显帧=命令实时反馈（100Hz）、20B 状态帧
  =20ms 自主心跳；CERB 反馈帧=命令实时状态（100Hz）、`0x080` SYNC 上报帧=20ms
  状态心跳（承担总线检测）。OFFLINE=心跳丢失 100ms。`motor_rt_task` 每 10ms 发
  一次 SYNC 维持 CERB 上报，禁止把 SYNC 当控制触发。
- CERB 仅使用 MIT：R48/R58 的 P/V/T Max 为 `π rad`、`π rad/s`、
  `11/45 Nm`；使能后等待至少 100ms（adapter `enable_settle_ms`）才发送目标。
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
- **DMA 分工（2026-09-07 低耦合口径）**：`lib/dma` 只管控制器级资源（DMA1 +
  BDMA/DMAMUX2 时钟、NVIC、占用登记），由 `main` 板级初始化统一使能（先于
  一切设备传输）。设备配置归各设备库自持：`lib/imu` 阻塞收发**不用 DMA**
  （8 字节寄存器读无收益，句柄/链接已移除），`lib/ws2812` 用 BDMA_Ch0 发送
  （帧缓冲必须在 `.bdma_noinit`——BDMA 仅能访问 DTCM/SRAM4，链接脚本见
  `ldscript/stm32h723vgt6.ld`）。
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
- **FDCAN RX 归属**：ISR 只盖时戳并把原始帧写入每总线 SPSC ring（容量 64，
  满时丢新帧并计数，不解码），`motor_rt_task` 是唯一消费者，每 1ms 每总线最多
  drain 16 帧；ROS 桥只能读取内核状态副本，不得让 bridge 或第二任务直接消费 ring。
- **FDCAN IRQ**：优先级固定为 5，必须满足
  `priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`；ring/调度器只在
  `bsp_can_init()` single-init，不在重连时重建。
- **改 HAL 构建宏后必须 `pio run -t clean` 全量重建**：SrcWrapper 的
  `stm32yyxx_hal_fdcan.c.o`、`stm32yyxx_hal_ospi.c.o` 不会因 build flag 变化自动重编。
- **FreeRTOS 下 `loop()` 只能非阻塞**：业务放 `sensor`/`control`/`micro_ros` 任务。
- **任务栈/周期**：`start_task` P1/256 words；`sensor` P2/512 words（DRDY；
  IMU 非关键路径，不得抢占 motor_rt）；
  `motor_rt` P3/1024 words（1ms 服务 + 10ms 控制边界）；`micro_ros` P2/8192 words
  （bridge timer）。调栈/周期后同步改 README。
- **控制发送唯一性**：ROS callback 只写完整 ID 快照 latest-wins mailbox；`motor_rt_task`
  是唯一正常 FDCAN 发送者（内核 → TX 调度器 → 硬件，P0/P1/P2 分级）。
  快照缺席/30ms 无快照 = MIT 默认 `kd` 阻尼软停；100ms 无快照、Agent 断链或
  撤权 = 协议级失能并清授权（须新授权 + 新完整快照才可能回 ACTIVE）；
  其他新鲜电机继续运行。
- **类型坑**：本平台 `int32_t` 是 `long`，模板推导（如 `std::clamp`）须显式
  指定类型参数。
- **`_write` 链接兜底**：固件代码不使用 Print/printf，框架弱 `_write`
  （Print.cpp.o）不会被链接器提取；libmicroros 的 rcutils/rosidl stdio 路径
  强引用它 → `src/syscalls.cpp` 提供强定义空汇。新增 stdio 用户须重审该文件，
  禁止把动态日志接上 USB CDC。
- **git pull 分支偏离**：残留 `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/build/`
  会让 download_dev_environment 的 `git pull` 失败 -> 删除该 build/ 目录后重跑。
- **GitHub 直连超时**：micro-ROS 依赖重下（regen）在国内网络下易超时；本机已把
  `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/microros_utils/repositories.py`
  的 github.com 前缀替换为 `https://ghfast.top/https://github.com` 镜像（补丁仅在本机
  .pio 内，未入库）。重下 libmicroros 前先确认该补丁仍在；只删 `libmicroros/` 时
  无需联网（缓存命中）。
- **自定义消息只用同仓库符号链接**：
  `extra_packages/can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs`。
  全新 clean bootstrap 仍有下载依赖/ROS 2 源的网络要求；常规验证不要运行
  `clean_microros`，使用缓存依赖执行 `pio run -t clean && pio run`。
- **改 .msg 后必须手动清 libmicroros**：micro_ros_platformio 的 extra_packages
  copytree 是 `dirs_exist_ok` 合并（`build/mcu/src/`），已删除的旧消息会残留、
  新消息头不会主动再生——否则出现 `motor_bind.h: No such file`（或反向的旧头
  残留）。改/删消息后删
  `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/{libmicroros,build/mcu}`
  再 `pio run`。2026-09-11 实测两条路径的坑：不删缓存的增量重建（含
  `../micro_ros_agent/tools/refresh_microros_messages.py`）可能因 copytree 合并陈旧状态报
  `Error running link command: No such file or directory`；删 `build/mcu`
  则触发依赖经 ghfast.top 镜像**重新克隆（需网络）**，代理断流/TLS 握手
  失败时清除半成品重试即可。

## 结构（简）

- `platformio.ini`：构建配置（USB CDC 宏 + FDCAN/OSPI HAL 宏）
- `boards/stm32h723vgt6.json`：变体/`variant_h`/`-DARDUINO_GENERIC_H723VGTX`/VID-PID
- `src/main.cpp`：Arduino 入口与 `start_task`（创建 `led`/`micro_ros`/`sensor`/`motor_rt`）
- `src/micro_ros_task.cpp`：micro-ROS agent 连接状态机
- `src/led_task.cpp`：WS2812 状态显示任务 + host 可测的状态到颜色映射
- `src/ros_bridge.cpp`：node/executor/timer 生命周期，编排各域桥
- `src/imu_bridge.cpp`：IMU/温度发布（`micro_ros/imu/data`、`micro_ros/imu/temp`）
- `src/motor_bridge.cpp`：电机 ID 化命令/Policy/Config 订阅与 motion/status/
  topology/events/config_result 发布
- `src/sensor_task.cpp`：传感器任务（EXTI/信号量/温度 PID/加热 PWM）
- `src/motor_rt_task.cpp`：唯一电机实时任务（1ms 服务 + 10ms 控制边界；RX drain、
  内核服务、SYNC 心跳、TX 路由、事件搬运、mailbox 消费）
- `src/system_clock.cpp`：覆盖 `SystemClock_Config`（HSE/HSI48/SPI2/SPI6 时钟树）
- `lib/imu`：BMI088 驱动/通信层/Mahony/SI 聚合（自持 SPI2 的 `HAL_SPI_MspInit`，
  不用 DMA，不依赖其他 lib）
- `lib/dma`：DMA1 控制器级资源门卫（时钟、流 NVIC、占用登记、优先级合规断言）；
  设备配置归各设备库，跨设备接线在任务层
- `lib/fdcan`：FDCAN 初始化/过滤/中断回调/每总线 SPSC RX ring/P0-P1-P2 TX 调度/
  硬件级统计（官方例程移植收敛）
- `lib/motor_control`：ID 驱动运行时内核——注册表/型号目录/九态生命周期/安全级联/
  配置事务/状态事件副本（纯 C，host 可测）
- `lib/motor_protocol`：adapter 接缝（`motor_protocol_adapter.h`）+ vaxor/CERB
  adapter 与纯 codec（厂家字节/CAN-ID 推导/FD 属性唯一归属地；自 ros2_ws
  `motor_drive/protocol.cpp` 移植，`-fno-exceptions` 下异常改返回值）
- `lib/control_mailbox`：ID 完整命令快照、低频配置 FIFO 与授权/连接边沿
- `lib/motor_event_relay`：内核事件中继环（满丢最旧保最新并计数，host 可测）
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
- FDCAN ISR → SPSC ring → `motor_rt_task` 唯一消费者是固定所有权边界；业务域桥
  只能通过 `motor_rt_copy_motion_states()` / `motor_rt_copy_statuses()` /
  `motor_rt_copy_topology()` / `motor_rt_drain_events()` 读取只读副本，
  不得直接访问 `can_bsp` 内部 ring/调度器或 HAL 发送接口。
- `dev-real` 交接只能做无负载真机验证且硬件急停必须可用；必须实测 RX burst 下
  motor_rt WCET/jitter、100Hz 周期、30/100ms 快照级联边界和 overflow/backlog。
  当前机构无抱闸，软件 stop 不能替代硬件急停、机械限位或供电隔离。
