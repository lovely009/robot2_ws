#ifndef IMU_SPI2_TRANSPORT_H
#define IMU_SPI2_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

// SPI2 板级资源显式初始化（时钟源/外设时钟/引脚/NVIC），幂等可重入。
//
// 为什么不依赖 HAL_SPI_MspInit 回调：本文件曾在静态库 libimu.a 中仅以
// HAL_SPI_MspInit 强符号对外——SrcWrapper 的 stm32yyxx_hal_spi.c.o 作为
// 直接对象文件先于库成员提供 __weak 空体（08006fec，2 字节），链接器对
// 已解析符号不再回绑 archive 强定义，成员整体不提取——SPI2EN 永不置位，
// BMI088 首笔传输超时锁存（2026-09-08 实机定位：APB1LENR bit14=0）。
// 因此设备层必须显式调用本函数，回调仅作兜底。
void imu_spi2_board_init(void);

#ifdef __cplusplus
}
#endif

#endif  // IMU_SPI2_TRANSPORT_H
