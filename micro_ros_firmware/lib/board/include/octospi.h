// OCTOSPI2 控制器层：句柄/初始化/间接读写，只服务 storage 任务经 w25q64
// 命令层使用；地址单位字节。
#ifndef OCTOSPI_H
#define OCTOSPI_H

#include "stm32h7xx_hal.h"

// 控制器层状态（数值与 w25q64_status_t 对齐，w25q64.c 静态断言锁死；
// 不下沉 ARGUMENT/ID 等芯片层概念）。
typedef enum {
  OCTOSPI_OK = 0,
  OCTOSPI_ERROR_HAL = -2,
  OCTOSPI_ERROR_TIMEOUT = -3,
  OCTOSPI_ERROR_STATE = -5,
} octospi_status_t;

#ifdef __cplusplus
extern "C" {
#endif

octospi_status_t octospi2_init(void);
void octospi2_deinit(void);
OSPI_HandleTypeDef *octospi2_handle(void);

#ifdef __cplusplus
}
#endif

#endif  // OCTOSPI_H
