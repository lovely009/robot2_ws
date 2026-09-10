// SPI2/BMI088 传输自持层（低耦合高内聚：设备传输配置归设备库）。
//
// 内容：SPI2 板级资源（引脚/时钟源/SPI2_IRQn）与 HAL_SPI_MspInit 兜底。
// 收发为阻塞 HAL_SPI_TransmitReceive（8 字节寄存器读，DMA 无收益——
// 2026-09-07 决策，DMA 句柄/链接/流 IRQHandler 已移除；lib/dma 占用表中
// DMA1_Stream0/1 已释放。若未来启用 DMA：句柄归本文件，控制器资源归
// lib/dma 登记）。
//
// 弱符号陷阱（2026-09-08）：本文件曾仅以 HAL_SPI_MspInit 强符号对外。
// SrcWrapper 的 stm32yyxx_hal_spi.c.o 以直接对象文件先于 libimu.a 成员
// 参与链接，其 __weak 空体（0x2 字节）先满足符号引用，链接器不再回绑
// archive 强定义——成员整体不提取，SPI2EN 永不置位，BMI088 首笔传输
// 超时锁存（实机定位：APB1LENR bit14=0、SPI2 寄存器读全 0）。修复：
// 板级配置导出为 imu_spi2_board_init()（见 include/imu_spi2_transport.h），
// 由 bmi088_middleware.c 在 HAL_SPI_Init 前显式调用；HAL_SPI_MspInit
// 保留为兜底（.o 被引用后随成员入链，强符号生效）。
//
// 前置契约（任务层保证）：main 板级初始化已先调 lib/dma 的 MX_DMA_Init
// （官方时序「DMA 先于 SPI2」对纯阻塞路径不再是硬依赖，保留惯例）。
// 本文件不 include 任何其他 lib 头。

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

// HAL 回调兜底：正常路径已由 imu_spi2_board_init 显式完成（见文件头
// 弱符号陷阱说明）。幂等保护下重复执行收敛为空操作。
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
