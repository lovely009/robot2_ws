// WS2812 彩灯驱动（SPI6 编码，官方 CtrBoard-H7_WS2812 例程同源方案）
//
// 原理：
//   - SPI6 内核时钟 = HSE 24MHz（SystemClock_Config 已设 RCC_SPI6CLKSOURCE_HSE），
//     预分频 /4 → SPI 时钟 6MHz
//   - 每 8 个 SPI 位编码 1 个 WS2812 位：0→0x60（2 高 6 低 ≈ 333ns 高），
//     1→0x78（4 高 4 低 ≈ 667ns 高），位周期 1.33us（WS2812 容差内）
//   - 24 位 GRB 颜色 = 24 字节；帧尾 100 字节 0x00 低电平复位（>50us）
//
// 注：此前用 SPI1 @2.125MHz + 3 位编码（0→0x80 高 470ns）卡在 WS2812 阈值
// （~0.5us）边缘，0 位被误判为 1 → 全白。官方 6MHz + 8 位编码 0 位仅 333ns，安全。

#include "ws2812.h"
#include "stm32_def.h"      // Error_Handler 宏
#include "stm32h7xx_hal.h"  // HAL SPI6 / GPIO

static SPI_HandleTypeDef hspi6;

void ws2812_init(void) {
  // SPI6 内核时钟源已在 SystemClock_Config 设为 HSE（24MHz）
  __HAL_RCC_SPI6_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA5=SPI6_SCK, PA7=SPI6_MOSI（AF8）
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &gpio);

  // HAL 把 SPI 的 CFG1/CFG2 寄存器一次性写入，故需显式配置关键字段；
  // 其余字段（CRC/TIMode/NSSPolarity 等）默认 0，结构体清零后省略。
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  // 24MHz/4 = 6MHz
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK) {
    Error_Handler();
  }
}

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  // WS2812 数据顺序为 GRB（绿红蓝）；0→0x60, 1→0x78（8 SPI 位/WS2812 位）
  uint8_t txbuf[24];
  for (int i = 0; i < 8; i++) {
    txbuf[7 - i] = (((g >> i) & 0x01) ? 0x78 : 0x60);
    txbuf[15 - i] = (((r >> i) & 0x01) ? 0x78 : 0x60);
    txbuf[23 - i] = (((b >> i) & 0x01) ? 0x78 : 0x60);
  }
  uint8_t zero = 0x00;
  // WS2812 是单线时序协议，位宽对中断/任务抢占敏感：发送期间关中断，
  // 保证 24 位颜色 + 复位低电平连续输出，避免被高优先级任务/ISR 打断
  // 导致灯珠锁存到错误数据（表现为突然全白/很亮闪一下）。
  // 关中断时长 ≈ 124 字节 @ 6MHz ≈ 165us，对 100Hz 控制/FDCAN 影响可忽略。
  __disable_irq();
  HAL_SPI_Transmit(&hspi6, &zero, 0, 0xFFFF);  // 空传输，确保 SPI 就绪
  while (hspi6.State != HAL_SPI_STATE_READY) {}
  HAL_SPI_Transmit(&hspi6, txbuf, 24, 0xFFFF);
  for (int i = 0; i < 100; i++) {
    HAL_SPI_Transmit(&hspi6, &zero, 1, 0xFFFF);  // 复位：>50us 低电平
  }
  __enable_irq();
}