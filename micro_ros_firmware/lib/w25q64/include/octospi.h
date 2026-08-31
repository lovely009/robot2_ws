#ifndef OCTOSPI_H_
#define OCTOSPI_H_

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

#endif  // OCTOSPI_H_
