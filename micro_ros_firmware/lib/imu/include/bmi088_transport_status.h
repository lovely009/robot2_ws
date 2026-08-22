#ifndef BMI088_TRANSPORT_STATUS_H
#define BMI088_TRANSPORT_STATUS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 每次 BMI088 初始化或采样前清除；首个 HAL 传输失败后保持锁存至下次尝试。
void BMI088_transport_error_clear(void);
bool BMI088_transport_error_latched(void);

// 仅由 middleware 在 HAL 传输失败时调用。
void BMI088_transport_error_latch(void);

#ifdef __cplusplus
}
#endif

#endif  // BMI088_TRANSPORT_STATUS_H
