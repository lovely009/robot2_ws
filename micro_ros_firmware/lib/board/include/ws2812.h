// WS2812 彩灯驱动接口（板载 1 颗 RGB 灯珠，SPI6 MOSI=PA7 + BDMA 发送）
//
// 用法：
//   ws2812_init();               // 初始化 SPI6+BDMA（PA5=SCK, PA7=MOSI, AF8）；
//                                // 前置：main 板级初始化已调 MX_DMA_Init
//   ws2812_submit_rgb(r, g, b);  // 非阻塞提交新颜色；忙返回 false，
//                                // 调用方下个周期重试
//
// 实现见 ws2812.c（SPI6 @6MHz 8 位编码 + BDMA1_Ch0 DMA 发送）。

#ifndef WS2812_H
#define WS2812_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 SPI6 + BDMA1_Ch0 发送通道（内核时钟 HSE 24MHz /4 = 6MHz）；
// 前置契约：main 板级初始化已先调 MX_DMA_Init（BDMA/DMAMUX2 时钟与 NVIC）
void ws2812_init(void);

// 非阻塞提交 RGB 颜色（0-255，内部按 GRB 顺序编码）；
// 返回 false = 上一帧 DMA 未完，调用方稍后重试
bool ws2812_submit_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif  // WS2812_H
