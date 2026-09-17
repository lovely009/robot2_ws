// OCTOSPI2 控制器层：句柄/初始化/间接读写，只服务 storage 任务经 w25q64
// 命令层使用；地址单位字节。
#ifndef OCTOSPI_H
#define OCTOSPI_H

#include "stm32h7xx_hal.h"

#include "w25q64.h"

#ifdef __cplusplus
extern "C" {
#endif

w25q64_status_t octospi2_init(void);
void octospi2_deinit(void);
OSPI_HandleTypeDef *octospi2_handle(void);

#ifdef __cplusplus
}
#endif

#endif  // OCTOSPI_H
