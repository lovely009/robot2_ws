// BMI088 通信层（以官方 CtrBoard-H7_IMU_TempCtrl 例程为准移植）。
//
// 例程挂在 HAL SPI2（PB13=SCK, PC1=MOSI, PC2=MISO）+ DMA1_Stream0/1，
// 硬件 CS=PC0(ACC)/PC3(GYRO)，POWER_5V=PC15，EXTI=PE10/PE12。
// 本工程为 STM32duino + FreeRTOS，因此：
//   - GPIO 上电/片选由本层初始化，SPI2 引脚与 DMA 由本库 imu_spi2_transport.c
//     的 HAL_SPI_MspInit 配置；
//   - EXTI 使用 Arduino attachInterrupt()（sensor_task.cpp 内）；
//   - SysTick 已被 FreeRTOS 接管，us 忙等用 delayMicroseconds，ms 用 vTaskDelay。

#include "bmi088_middleware.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "stm32h7xx_hal.h"

#define POWER_5V_PORT GPIOC
#define POWER_5V_PIN GPIO_PIN_15
#define ACC_CS_PORT GPIOC
#define ACC_CS_PIN GPIO_PIN_0
#define GYRO_CS_PORT GPIOC
#define GYRO_CS_PIN GPIO_PIN_3

static SPI_HandleTypeDef hspi_bmi088;
static const uint32_t kBmi088SpiTransferTimeoutMs = 2U;

void BMI088_GPIO_init(void) {
  // PC15=POWER_5V：传感器供电使能（官方默认置高）
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = POWER_5V_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_5V_PORT, &gpio);

  // PC0=ACC_CS, PC3=GYRO_CS：硬件片选，默认拉到高电平（未选通）
  gpio.Pin = ACC_CS_PIN | GYRO_CS_PIN;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACC_CS_PORT, &gpio);
  HAL_GPIO_WritePin(GPIOC, POWER_5V_PIN | ACC_CS_PIN | GYRO_CS_PIN,
                    GPIO_PIN_SET);

  // PA0：官方例程固定输出的保留引脚，置低
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef pa0 = {0};
  pa0.Pin = GPIO_PIN_0;
  pa0.Mode = GPIO_MODE_OUTPUT_PP;
  pa0.Pull = GPIO_NOPULL;
  pa0.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &pa0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void BMI088_com_init(void) {
  // 前置契约：任务层（sensor_task_start）已先经 lib/dma 完成 DMA1 时钟与
  // 流 NVIC（官方时序：DMA 先于 SPI2）。SPI2 引脚/DMA 流链接在本库
  // imu_spi2_transport.c 的 HAL_SPI_MspInit 内完成。
  hspi_bmi088.Instance = SPI2;
  hspi_bmi088.Init.Mode = SPI_MODE_MASTER;
  hspi_bmi088.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_bmi088.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_bmi088.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi_bmi088.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_bmi088.Init.NSS = SPI_NSS_SOFT;
  hspi_bmi088.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi_bmi088.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_bmi088.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_bmi088.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_bmi088.Init.CRCPolynomial = 0x0;
  hspi_bmi088.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi_bmi088.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi_bmi088.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  if (HAL_SPI_Init(&hspi_bmi088) != HAL_OK) {
    Error_Handler();
  }
}

void BMI088_delay_ms(uint16_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));  // 让出 CPU，交给其他任务
}

void BMI088_delay_us(uint16_t us) {
  delayMicroseconds(us);  // 自旋延时，不依赖 SysTick
}

void BMI088_ACCEL_NS_L(void) {
  HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void) {
  HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void) {
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void) {
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t reg) {
  uint8_t tx = reg, rx = 0;
  if (BMI088_transport_error_latched()) {
    return 0U;
  }
  if (HAL_SPI_TransmitReceive(&hspi_bmi088, &tx, &rx, 1,
                              kBmi088SpiTransferTimeoutMs) != HAL_OK) {
    BMI088_transport_error_latch();
    return 0U;
  }
  return rx;
}

// 官方使能了 SPI2 中断；阻塞路径不产生事件，保留转发入口以防后续 DMA 化。
void SPI2_IRQHandler(void) {
  HAL_SPI_IRQHandler(&hspi_bmi088);
}
