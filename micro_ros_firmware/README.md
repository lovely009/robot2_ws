# micro_ros_firmware

STM32H723VGT6 上的 **micro-ROS 空工程**（PlatformIO + STM32duino + FreeRTOS），
通过 **serial over USB CDC**（PA11=USB_DM / PA12=USB_DP）与主机侧
`../micro_ros_agent/` 通信，桥接进 ROS 2 Humble 域。

工程参考 [husarion/micro_ros_stm32_template](https://github.com/husarion/micro_ros_stm32_template)
的任务编排方式：**FreeRTOS 调度 + micro-ROS 独立任务建链/收发**。当前为
**空白脚手架**，仅包含节点初始化的最小骨架，无业务逻辑；后续业务
（CAN/电机/IMU 等）在 `src/ros_bridge.cpp` 与 `src/main.cpp` 的占位任务里增量扩展。

## 关键决策

| 决策 | 取值 | 理由 |
|---|---|---|
| 框架 | `framework = arduino`（STM32duino） | `micro_ros_platformio` 官方只支持 Arduino 框架；`stm32cube` 无官方集成，需自写传输，故弃用 |
| 实时内核 | `STM32FreeRTOS`（stm32duino） | 与 Arduino 核心共存，调度器接管 SysTick，micro-ROS 在独立任务中运行；与模板一致 |
| ROS 2 发行版 | `board_microros_distro = humble` | 与 `micro_ros_agent` 源码（humble）对齐 |
| 传输 | `board_microros_transport = serial`（物理层 USB CDC） | micro-ROS 侧保持 serial 传输；STM32duino 启用 USB CDC 后 generic `Serial` 即映射为 `SerialUSB`，agent 以 `/dev/ttyACM0` 对接 |
| USB 引脚 | **PA11=USB_DM / PA12=USB_DP** | H723 的 USB OTG HS 内嵌 PHY（FS 模式）固定引脚 |
| USB 构建宏 | `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC` + `PIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED_FULLMODE` | 前者触发 USBDevice 库（`USBD_USE_CDC`+`USBCON`）；后者因 **H723 只有 `USB_OTG_HS`（无独立 `USB_OTG_FS`）**，须以 FS 模式 + 内嵌 PHY 使用（`USE_USB_HS`+`USE_USB_HS_IN_FS`） |
| FreeRTOS 与 SysTick | `osSystickHandler()` 弱符号覆盖 | STM32duino 的 `SysTick_Handler()` 调用弱 `osSystickHandler()`，FreeRTOS 覆写之，`millis()`/`micros()` 与调度共存 |
| 状态 LED | **PA7（WS2812 彩灯，SPI6 MOSI 驱动）** | 板载指示灯为 1 颗 WS2812 RGB 灯珠，数据线接 SPI6 MOSI（PA7，AF8） |
| 板卡变体 | `STM32H7xx/H723V(E-G)(H-T)_H730VB(H-T)_H733VG(H-T)` + `variant_generic.h` | STM32duino 官方 GENERIC_H723VGTX 的变体（旧式变体名已废弃） |

> **为何改板卡 JSON**：STM32duino 变体源码被 `ARDUINO_GENERIC_H723VGTX` 宏保护，
> 自定义板卡默认不定义该宏，会导致 `digitalPin`/`SystemClock_Config` 链接失败。
> `boards/stm32h723vgt6.json` 已在 `extra_flags` 中加入该宏，并显式指定
> `arduino.variant_h = variant_generic.h`，另加 `build.hwids` 提供 USB VID/PID。

> **为何覆盖 `SystemClock_Config`**：已安装的 STM32duino 4.21200.0 中 H723V 变体的
> `generic_clock.c` 未使能 HSI48、未选择 USB 时钟源，USB CDC 无法工作。
> `src/system_clock.cpp` 以强定义覆盖 WEAK 符号，采用官方 CtrBoard-H7 例程时钟树：
> 主时钟 HSE 24MHz → PLL1 → 480MHz SYSCLK（HCLK 240MHz / APB 120MHz）；使能 HSI48
> 并选为 USB 时钟（`RCC_USBCLKSOURCE_HSI48`）；SPI6 内核时钟 = HSE（WS2812）。

## 任务编排

`src/main.cpp` 在 `setup()` 里完成板级初始化并创建 FreeRTOS 任务，末尾
`vTaskStartScheduler()`：

| 任务 | 优先级 | 栈 | 职责 |
|---|---|---|---|
| `micro_ros` | 2 | 8192 | 等 agent → 建实体 → 收发（连接状态机），断连自动重建 |
| `app` | 1 | 2048 | 业务占位，后续在此增量扩展 |

`loop()` 会被 STM32FreeRTOS 的 `vApplicationIdleHook` 调用，故必须保持非阻塞。

## 目录结构

```
micro_ros_firmware/
├── platformio.ini             # 构建配置（arduino/humble/serial+USB CDC，lib_deps 拉取 micro_ros_platformio + STM32FreeRTOS）
├── boards/
│   └── stm32h723vgt6.json     # 自定义板卡定义（变体/宏/USB VID-PID/烧录协议）
├── include/
│   ├── ros_bridge.h           # micro-ROS 节点生命周期接口
│   └── ws2812.h               # 板载 WS2812 彩灯驱动接口
└── src/
    ├── main.cpp               # 空工程入口：板级初始化 + FreeRTOS 任务编排
    ├── ros_bridge.cpp         # micro-ROS node/executor 生命周期（空白脚手架）
    ├── ws2812.c               # WS2812 驱动（SPI6 MOSI=PA7，HAL 编码）
    └── system_clock.cpp       # 覆盖 SystemClock_Config：HSE 主时钟 + HSI48/USB + SPI6/HSE
```

构建产物 `.pio/`、`libmicroros/` 已加入 `.gitignore`。

## 前置条件

- PlatformIO Core ≥ 6.1.0（本机：`~/.platformio/penv/bin/pio`，v6.1.19）
- `git`、`cmake`、`python3-pip`（micro_ros_platformio 首次构建会下载并编译
  micro-ROS 库，需要外网访问 GitHub）
- ST-Link 调试器（烧录用 `stlink` 协议）

## 构建与烧录

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 首次下载 STM32duino + STM32FreeRTOS + 构建 libmicroros
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录
~/.platformio/penv/bin/pio device monitor # 串口监视（115200，USB CDC 下波特率无效）
```

> 若想用 `pio` 命令而非全路径，可 `source ~/.platformio/penv/bin/activate`。

## 与 micro_ros_agent 联调

主机侧启动 agent（serial 传输，USB CDC 设备 `/dev/ttyACM0`）：

```bash
source /opt/ros/humble/setup.bash
source ../micro_ros_agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

> 若 USB 设备号不是 `/dev/ttyACM0`，用 `ls /dev/ttyACM*` 确认；也可
> `udevadm info -q property -n /dev/ttyACM0` 查看 VID/PID（应为 0483:5740）。

建链后应能观测到节点 `micro_ros_node`（空工程无话题，业务扩展后在此加话题）。

## 硬件接线

USB CDC 传输（PA11=USB_DM / PA12=USB_DP），无需 USB-TTL 转换器，直接接主机 USB：

| 信号 | MCU 引脚 |
|---|---|
| USB_DM | PA11 |
| USB_DP | PA12 |
| GND | 与主机共地（USB 线已含） |
| 状态灯 | PA7（WS2812 彩灯，SPI6 MOSI 驱动，无需限流电阻） |

> 换回 UART：在 `platformio.ini` 去掉两个 USB CDC 宏，并把 `src/main.cpp` 顶部
> `kMicrorosSerial` 改回具体 `HardwareSerial`（如 `Serial1`=USART1 PA9/PA10、
> `Serial2`=USART2 PA2/PA3、`Serial3`=USART3 PB10/PB11），经 USB-TTL 接主机。

## 已知问题与后续

- `variant_generic.h` 是 STM32duino 通用引脚映射，不含机器人专用外设；
  若需自定义 GPIO/外设，可在 `include/` 增加自有变体头并在板卡 JSON 切换 `variant_h`。
- micro-ROS 无 agent 时节点不注册、固件在 `kWaitAgent` 轮询，属正常。
- USB CDC 依赖主机枚举：未接主机 USB 时固件停留在 `kWaitAgent`，接上并启动 agent 后自动建链。
- `loop()` 被 STM32FreeRTOS 的 idle hook 调用，只能放非阻塞代码。
