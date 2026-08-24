// WS2812 彩灯驱动接口（板载 1 颗 RGB 灯珠，SPI6 MOSI=PA7 驱动）
//
// 用法：
//   ws2812_init();            // 初始化 SPI6（PA5=SCK, PA7=MOSI, AF8）
//   ws2812_set_rgb(r,g,b);    // 设置颜色（内部按 GRB 顺序编码）
//
// 实现见 ws2812.c（官方 CtrBoard-H7_WS2812 例程同源方案）。

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 SPI6 驱动 WS2812（内核时钟 HSE 24MHz /4 = 6MHz）
void ws2812_init(void);

// 设置 RGB 颜色（0-255），内部按 GRB 顺序编码并发送
void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif  // WS2812_H