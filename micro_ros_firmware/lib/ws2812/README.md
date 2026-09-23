# ws2812

## 审查结论

**职责符合；这是目标板物理发送库，暂无独立 host 测试。** 状态到颜色的策略留在 `src/led_task.cpp`，本库只编码并发送 RGB 波形。

## 职责与接口

- `ws2812_init()` 初始化 SPI6、PA5/PA7 和 BDMA1 Channel0。
- `ws2812_submit_rgb(r, g, b)` 把 RGB 按 WS2812 要求编码为 GRB，非阻塞提交 DMA；上一帧未完成时返回 false。
- 帧缓冲位于 `.bdma_noinit`/RAM_D3，帧尾保留低电平复位时间。
- IRQ handler 由本库持有：`BDMA_Channel0_IRQHandler()` 和 `SPI6_IRQHandler()`。

## 依赖与所有权

前置条件是 `main.cpp` 先调用 `MX_DMA_Init()`；DMA 控制器级时钟/NVIC 归 `lib/dma`，SPI6 句柄、BDMA 配置和缓冲区归本库。运行期唯一调用者是 `led_task`，库不读取链路状态、不判断安全状态。

## 验证与缺口

PlatformIO 固件构建通过。当前没有 host 目标，因为 SPI6 EOT、BDMA 完成中断、缓存/链接段和 WS2812 电气波形无法由纯 C 单测证明。目标板验证应检查颜色顺序、忙返回、DMA 完成后再次提交和复位脉冲。

## 后续建议

保持 `MX_DMA_Init()` 的初始化顺序契约；如果增加多灯珠或帧队列，需重新评估固定缓冲大小、BDMA 可访问内存和 `led_task` 所有权。
