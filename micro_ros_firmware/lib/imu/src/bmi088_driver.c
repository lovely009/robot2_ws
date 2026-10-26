// BMI088 驱动（官方 CtrBoard-H7_IMU 例程移植，SPI 模式）。
// 原样保留初始化/读写/换算逻辑；通信与延时由 bmi088_middleware.c 提供。

#include "bmi088_driver.h"
#include "bmi088_reg.h"
#include "bmi088_middleware.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)
#define BMI088_accel_write_single_reg(reg, data) \
  {                                              \
    BMI088_ACCEL_NS_L();                         \
    BMI088_write_single_reg((reg), (data));      \
    BMI088_ACCEL_NS_H();                         \
  }
// 注意：加速度计单寄存器读比陀螺多一个 dummy 字节（地址后先写 0x55 再读），
// 这是 BMI088 数据手册对 ACC SPI 读的硬性要求，不能改用下面的
// BMI088_read_single_reg（只有地址 + 数据两次传输，仅适用于 GYRO）。
#define BMI088_accel_read_single_reg(reg, data) \
  {                                             \
    BMI088_ACCEL_NS_L();                        \
    BMI088_read_write_byte((reg) | 0x80);       \
    BMI088_read_write_byte(0x55);               \
    (data) = BMI088_read_write_byte(0x55);      \
    BMI088_ACCEL_NS_H();                        \
  }
// 同上：ACC 多寄存器读同样需要在地址后补一个 dummy 字节。
#define BMI088_accel_read_muli_reg(reg, data, len) \
  {                                                \
    BMI088_ACCEL_NS_L();                           \
    BMI088_read_write_byte((reg) | 0x80);          \
    BMI088_read_muli_reg(reg, data, len);          \
    BMI088_ACCEL_NS_H();                           \
  }
#define BMI088_gyro_write_single_reg(reg, data) \
  {                                             \
    BMI088_GYRO_NS_L();                         \
    BMI088_write_single_reg((reg), (data));     \
    BMI088_GYRO_NS_H();                         \
  }
#define BMI088_gyro_read_single_reg(reg, data)  \
  {                                             \
    BMI088_GYRO_NS_L();                         \
    BMI088_read_single_reg((reg), &(data));     \
    BMI088_GYRO_NS_H();                         \
  }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
  {                                                 \
    BMI088_GYRO_NS_L();                             \
    BMI088_read_muli_reg((reg), (data), (len));     \
    BMI088_GYRO_NS_H();                             \
  }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
#endif

static const uint8_t g_acc_cfg[BMI088_WRITE_ACCEL_REG_NUM][3] = {
  {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
  {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
  {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_100_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
  {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
  {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
  {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

static const uint8_t g_gyro_cfg[BMI088_WRITE_GYRO_REG_NUM][3] = {
  {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
  {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_100_12_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
  {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
  {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
  {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
  {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

uint8_t BMI088_init(void) {
  uint8_t error = BMI088_NO_ERROR;
  BMI088_GPIO_init();
  BMI088_com_init();
  error |= bmi088_accel_init();
  error |= bmi088_gyro_init();
  return error;
}

uint8_t bmi088_accel_init(void) {
  uint8_t res = 0;
  uint8_t n = 0;

  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  if (res != BMI088_ACC_CHIP_ID_VALUE) {
    return BMI088_NO_SENSOR;
  }

  for (n = 0; n < BMI088_WRITE_ACCEL_REG_NUM; n++) {
    BMI088_accel_write_single_reg(g_acc_cfg[n][0], g_acc_cfg[n][1]);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(g_acc_cfg[n][0], res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != g_acc_cfg[n][1]) {
      return g_acc_cfg[n][2];
    }
  }
  return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void) {
  uint8_t n = 0, res = 0;

  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  if (res != BMI088_GYRO_CHIP_ID_VALUE) {
    return BMI088_NO_SENSOR;
  }

  for (n = 0; n < BMI088_WRITE_GYRO_REG_NUM; n++) {
    BMI088_gyro_write_single_reg(g_gyro_cfg[n][0], g_gyro_cfg[n][1]);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(g_gyro_cfg[n][0], res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != g_gyro_cfg[n][1]) {
      return g_gyro_cfg[n][2];
    }
  }

  return BMI088_NO_ERROR;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate) {
  uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
  int16_t raw_temp;

  BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
  raw_temp = (int16_t)((buf[1] << 8) | buf[0]);
  accel[0] = raw_temp * BMI088_ACCEL_SEN;
  raw_temp = (int16_t)((buf[3] << 8) | buf[2]);
  accel[1] = raw_temp * BMI088_ACCEL_SEN;
  raw_temp = (int16_t)((buf[5] << 8) | buf[4]);
  accel[2] = raw_temp * BMI088_ACCEL_SEN;

  BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
  if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
    raw_temp = (int16_t)((buf[3] << 8) | buf[2]);
    gyro[0] = raw_temp * BMI088_GYRO_SEN;
    raw_temp = (int16_t)((buf[5] << 8) | buf[4]);
    gyro[1] = raw_temp * BMI088_GYRO_SEN;
    raw_temp = (int16_t)((buf[7] << 8) | buf[6]);
    gyro[2] = raw_temp * BMI088_GYRO_SEN;
  }

  BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
  raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if (raw_temp > 1023) {
    raw_temp -= 2048;
  }
  *temperate = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)
static void BMI088_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_read_write_byte(reg);
  BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data) {
  BMI088_read_write_byte(reg | 0x80);
  *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
  BMI088_read_write_byte(reg | 0x80);
  while (len != 0) {
    *buf = BMI088_read_write_byte(0x55);
    buf++;
    len--;
  }
}
#endif
