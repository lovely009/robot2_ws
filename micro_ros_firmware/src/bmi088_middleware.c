// BMI088 通信层（以官方 CtrBoard-H7_IMU 例程为准移植）。
//
// 例程挂在 HAL SPI2（PB13=SCK, PC1=MOSI, PC2=MISO）,硬件 CS=PC0(ACC)/PC3(GYRO)。
// 本工程为 STM32duino + FreeRTOS，故：
//   - 采用与 ws2812.c 一致的直接 HAL 寄存器写法初始化 SPI2；
//   - SysTick 已被 FreeRTOS 接管，us 忙等改用 delayMicroseconds，ms 用 vTaskDelay。

#include "bmi088_middleware.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "stm32h7xx_hal.h"

#define ACC_CS_PORT GPIOC
#define ACC_CS_PIN GPIO_PIN_0
#define GYRO_CS_PORT GPIOC
#define GYRO_CS_PIN GPIO_PIN_3

static SPI_HandleTypeDef hspi_bmi088;

void BMI088_GPIO_init(void) {
  // 仅硬件片选（PC0=ACC_CS, PC3=GYRO_CS），默认拉到高电平（未选通）
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = ACC_CS_PIN | GYRO_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, ACC_CS_PIN | GYRO_CS_PIN, GPIO_PIN_SET);
}

void BMI088_com_init(void) {
  // SPI2 引脚：PB13=SCK, PC1=MOSI, PC2=MISO（AF5），与例程一致
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  HAL_GPIO_Init(GPIOC, &gpio);  // PC1=MOSI, PC2=MISO，其余字段与 SCK 一致

  // 关键字段与例程一致：主模式、CPOL/CPHA=11、MSB、软 NSS、预分频 32
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
  HAL_SPI_TransmitReceive(&hspi_bmi088, &tx, &rx, 1, 0xFFFF);
  return rx;
}
