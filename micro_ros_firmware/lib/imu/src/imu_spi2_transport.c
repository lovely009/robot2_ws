// BMI088 的 SPI2 板级资源层：配置时钟源、引脚与 NVIC。
// 当前收发使用阻塞 HAL_SPI_TransmitReceive，不占用 DMA；若后续改用 DMA，
// 设备句柄和回调仍归本库，控制器资源向 lib/dma 登记。
// 设备层必须显式调用 imu_spi2_board_init；HAL_SPI_MspInit 仅作幂等兜底。

#include "imu_spi2_transport.h"
#include "stm32h7xx_hal.h"
#include "stm32_def.h"      // Error_Handler
#include <STM32FreeRTOS.h>  // configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（断言用）
#include <stdbool.h>

// SPI2 中断优先级：与 lib/dma 的控制器级政策一致
// （6/0，低于 syscall 门槛 5，数值更大）。
#define IMU_SPI2_IRQ_PRIORITY 6U

_Static_assert(IMU_SPI2_IRQ_PRIORITY >=
                   configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "SPI2 IRQ priority must not preempt the FreeRTOS syscall threshold");

static bool g_spi2_board_done = false;

void imu_spi2_board_init(void) {
  if (g_spi2_board_done) {
    return; // 幂等：init 重试/回调兜底双路径都收敛到首次配置
  }
  g_spi2_board_done = true;

  // SPI2 时钟源与官方一致：PLL
  RCC_PeriphCLKInitTypeDef periph_clk = {0};
  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  periph_clk.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
    Error_Handler();
  }

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

  // 保留官方中断使能；阻塞收发路径不会触发 SPI 事件
  HAL_NVIC_SetPriority(SPI2_IRQn, IMU_SPI2_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
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
