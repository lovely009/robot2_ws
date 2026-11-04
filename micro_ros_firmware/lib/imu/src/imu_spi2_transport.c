// BMI088 的 SPI2 板级资源层：配置时钟源与引脚。
// 当前收发使用阻塞 HAL_SPI_TransmitReceive，不占用 DMA 也不使能 SPI2_IRQn；
// 若后续改用 DMA，设备句柄和回调仍归本库，控制器资源向 src/dma.c 登记。
// 设备层必须显式调用 imu_spi2_board_init；HAL_SPI_MspInit 仅作幂等兜底。

#include "imu_spi2_transport.h"
#include "stm32h7xx_hal.h"
#include "stm32_def.h"      // Error_Handler
#include <stdbool.h>

static bool g_spi2_board_done = false;

void imu_spi2_board_init(void) {
  if (g_spi2_board_done) {
    return; // 幂等：init 重试/回调兜底双路径都收敛到首次配置
  }
  g_spi2_board_done = true;

  // SPI123 时钟源（PLL1Q 240MHz）由 system_clock.cpp 单点选择，此处只开时钟门。
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // PC1=SPI2_MOSI, PC2_C=SPI2_MISO, PB13=SPI2_SCK（AF5）
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &gpio);
  // 不使能 SPI2_IRQn：本库用阻塞收发，不产生 SPI 事件。
}

// HAL 回调兜底：正常路径已显式初始化，幂等保护使重复调用为空操作。
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  if (hspi == NULL) {
    return;
  }
  // Instance 为外设寄存器结构指针，不能用 switch（非整型）。
  if (hspi->Instance == SPI2) {
    imu_spi2_board_init();
  }
  // 未登记的 SPI：不初始化（外设自负责报错）。
}
