# dma

## 审查结论

**符合职责边界；硬件路径没有 host 覆盖。** 该库只管理 DMA 控制器级资源，设备传输配置仍由设备库持有，和 `lib/README` 的定义一致。

## 职责与边界

- `MX_DMA_Init()` 开启 DMA1、BDMA/DMAMUX2 时钟并配置 BDMA Channel0 的 NVIC 优先级。
- `BOARD_ISR_SYSCALL_THRESHOLD` 是 ISR 优先级门槛的唯一来源。
- WS2812 自持 SPI6 TX 的 DMA 句柄、handler 和帧缓冲；BMI088 当前使用阻塞 SPI2，不占 DMA。
- 本库不配置 IMU/LED 的业务传输，不依赖任务、ROS 或电机协议。

## 文件与接口

| 文件 | 作用 |
|---|---|
| `include/dma.h` | HAL 依赖和 `MX_DMA_Init()` 声明 |
| `src/dma.c` | DMA1/BDMA 时钟、IRQ 配置和资源登记注释 |
| `library.json` | PlatformIO 元数据 |

## 依赖与所有权

`dma.h` 依赖 STM32 HAL，`dma.c` 额外依赖 `board_irq.h`。`main.cpp` 在设备初始化前调用 `MX_DMA_Init()`；设备库不得互相占用或调用彼此的句柄。

## 验证与缺口

- PlatformIO 固件构建通过。
- 没有 host 单测，原因是 HAL/NVIC 行为只能在目标板验证。
- 资源表目前是注释和配置的组合契约；新增 DMA 使用者必须同步更新占用表、IRQ 优先级和链接脚本中的缓冲区约束。

## 后续建议

`library.json` 已显式声明 `include/`、`src/` 目录与 `board` 依赖。新增 DMA 使用者仍需在资源表和台架验证中确认通道不冲突。
