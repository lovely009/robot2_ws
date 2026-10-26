#ifndef IMU_SPI2_TRANSPORT_H
#define IMU_SPI2_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

// 显式初始化 SPI2 板级资源（时钟源、外设时钟、引脚和 NVIC）。
// 函数幂等，设备层须在 HAL_SPI_Init 前调用；HAL_SPI_MspInit 仅作兜底。
void imu_spi2_board_init(void);

#ifdef __cplusplus
}
#endif

#endif  // IMU_SPI2_TRANSPORT_H
