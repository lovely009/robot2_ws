#ifndef BMI088_MIDDLEWARE_H
#define BMI088_MIDDLEWARE_H

#include <stdint.h>

#define BMI088_USE_SPI

// 硬件抽象层（由本工程以例程为准实现）：SPI2 + 硬件 CS + FreeRTOS 安全延时
#ifdef __cplusplus
extern "C" {
#endif

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);
extern uint8_t BMI088_read_write_byte(uint8_t reg);
#endif

#ifdef __cplusplus
}
#endif

#endif  // BMI088_MIDDLEWARE_H
