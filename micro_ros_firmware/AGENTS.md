# micro_ros_firmware —— Agent 指南

STM32H723VGT6 micro-ROS **空工程**（PlatformIO + STM32duino + FreeRTOS，serial over
USB CDC 进 ROS 2 humble，参考 husarion/micro_ros_stm32_template）。**权威文档：
`README.md`**（决策/引脚/时钟/联调），本文件只列 Agent 即用要点；改动前先读 README。

当前为**空白脚手架**：仅节点初始化 + FreeRTOS 任务编排，无业务逻辑；后续业务在
`src/main.cpp` 的 `app_task` 与 `src/ros_bridge.cpp` 增量扩展。

## 构建与烧录（本机）

`pio` 不在 PATH，用全路径：

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 构建（首次下载 STM32duino + 构建 libmicroros，耗时较长，需外网）
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录（需 stlink udev 规则）
~/.platformio/penv/bin/pio device monitor # 串口监视 115200（USB CDC 下波特率无效）
```

## 关键决策（改动前确认，勿破坏）

- `framework = arduino`：`micro_ros_platformio` 官方只支持 Arduino 框架（stm32cube 无官方集成）。
- `board_microros_distro = humble`、`board_microros_transport = serial`（物理层 USB CDC，
  agent 以 `/dev/ttyACM0` 对接）。
- `STM32FreeRTOS`：与 Arduino 核心共存，调度器接管 SysTick；micro-ROS 建链/收发在
  独立 `micro_ros` 任务（优先级 2）中运行，业务占位在 `app` 任务（优先级 1）。
- USB 引脚 **PA11=USB_DM / PA12=USB_DP**；状态 LED 是 **WS2812 彩灯**（SPI6 MOSI=PA7，
  非 GPIO）。
- 节点名 `micro_ros_node`（在 `src/ros_bridge.cpp` 顶部常量，改一处即可）。

## 高频踩坑（务必先看）

- **USB CDC 宏**：H723 只有 `USB_OTG_HS`（无独立 `USB_OTG_FS`），必须
  `-DPIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED_FULLMODE`（FS 模式 + 内嵌 PHY）。启用后 generic
  `Serial` 映射为 `SerialUSB`（继承 `Stream` 而非 `HardwareSerial`），引用须用 `auto &`。
- **FreeRTOS 下 `loop()` 只能非阻塞**：STM32FreeRTOS 的 `vApplicationIdleHook` 会调
  `loop()`，里面不得 `vTaskDelay`/阻塞；业务放 `app_task`，勿写进 `loop()`。
- **任务栈**：micro-ROS 任务栈 8192（RMW 较大），优先级高于业务任务，避免收发被饿死。
- **HAL 模块使能走命令行 `-D` 宏**：STM32duino 的 HAL 模块（如 `HAL_FDCAN_MODULE_ENABLED`）
  须加进 `platformio.ini` build_flags，不能改 `stm32h7xx_hal_conf_default.h`（模块列表在
  `#if 0` 死代码里）。当前空工程不需要额外 HAL 模块。
- **时钟覆盖**：`src/system_clock.cpp` 强定义覆盖 WEAK `SystemClock_Config`（HSE 24MHz 主时钟 +
  HSI48/USB + SPI6/HSE）。删掉则 USB 不枚举、WS2812 不亮。
- **类型坑**：本平台 `int32_t` 是 `long`，模板推导（如 `std::clamp`）须显式指定类型参数。
- **git pull 分支偏离**：残留 `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/build/` 会让
  download_dev_environment 的 `git pull` 失败 → 删除该 build/ 目录后重跑。
- **烧录/串口权限**：`libusb_open() failed with LIBUSB_ERROR_ACCESS` = 缺 stlink udev 规则
  （0483:3748，`MODE="0666" GROUP="plugdev"`）；`/dev/ttyACM0` 打不开 = 缺 CDC udev 规则
  （0483:5740）。`sudo -A`（sudoplz）可免密执行 root 命令；**sudo 密码交互须用户本人在终端输入**。

## 代码规范

- C++17（`std=gnu++17`）+ C（STM32duino HAL）混合；2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类）、`k` 前缀常量。
- 中文注释写意图（数据流/单位/安全边界）；micro-ROS / ROS 2 API 保留英文。
- 硬件映射集中：引脚/话题/节点名在 `main.cpp` 或 `ros_bridge.cpp` 顶部常量，不散落。

## 结构（简）

- `platformio.ini`：构建配置（USB CDC 宏）
- `boards/stm32h723vgt6.json`：板卡变体/`variant_h`/`-DARDUINO_GENERIC_H723VGTX`/USB VID-PID
- `src/main.cpp`：板级初始化 + FreeRTOS 任务编排（`micro_ros`/`app` 任务）
- `src/ros_bridge.cpp`：ROS 实体（node/executor 生命周期，空白脚手架）
- `src/ws2812.c`：板载状态灯（SPI6 MOSI=PA7）
- `src/system_clock.cpp`：覆盖 `SystemClock_Config`（HSE/HSI48/SPI6 时钟树）
