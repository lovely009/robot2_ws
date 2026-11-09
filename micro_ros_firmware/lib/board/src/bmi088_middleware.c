// BMI088 通信层：SPI2 阻塞收发，板级资源由 imu_spi2_transport.c 初始化。
// GPIO 上电/片选由本层管理，EXTI 由 sensor_task.cpp 管理；毫秒延时使用
// FreeRTOS，微秒延时使用 delayMicroseconds。

#include "bmi088_middleware.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "stm32h7xx_hal.h"
#include "imu_spi2_transport.h"

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
  // 显式初始化 SPI2 板级资源；HAL_SPI_MspInit 仅作兜底。
  imu_spi2_board_init();
  hspi_bmi088.Instance = SPI2;
  hspi_bmi088.Init.Mode = SPI_MODE_MASTER;
  hspi_bmi088.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_bmi088.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_bmi088.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi_bmi088.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_bmi088.Init.NSS = SPI_NSS_SOFT;
  hspi_bmi088.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // PLL1Q 240MHz/32 = 7.5MHz SCK
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
    // 已锁存：短路本次操作剩余字节，避免每字节重跑 2ms HAL 超时。
    return 0U;
  }
  if (HAL_SPI_TransmitReceive(&hspi_bmi088, &tx, &rx, 1,
      kBmi088SpiTransferTimeoutMs) != HAL_OK) {
    BMI088_transport_error_latch();
    return 0U;
  }
  return rx;
}
