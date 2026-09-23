// IMU 单位边界回归：BMI088 驱动给出 °/s，Mahony 与 ROS 输出必须使用 rad/s。

#include "imu.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

extern "C" uint8_t BMI088_init(void) {
  return 0U;
}

extern "C" void BMI088_read(float gyro[3], float accel[3], float *temperature) {
  gyro[0] = 0.0f;
  gyro[1] = 0.0f;
  gyro[2] = 1.0f;
  accel[0] = 0.0f;
  accel[1] = 0.0f;
  accel[2] = 0.0f;
  *temperature = 25.0f;
}

static bool near(float actual, float expected, float tolerance) {
  if (std::fabs(actual - expected) <= tolerance) {
    return true;
  }
  std::fprintf(stderr, "actual=%f expected=%f tolerance=%f\n", actual,
      expected, tolerance);
  return false;
}

int main() {
  constexpr float kOneDegreeRad = 3.14159265358979f / 180.0f;
  constexpr int kSamplesPerSecond = 100;

  if (!imu_try_init()) {
    std::fprintf(stderr, "imu_try_init failed\n");
    return 1;
  }
  for (int sample = 0; sample < kSamplesPerSecond; ++sample) {
    if (!imu_update()) {
      std::fprintf(stderr, "imu_update failed at sample %d\n", sample);
      return 1;
    }
  }

  imu_data_t data{};
  imu_get_data(&data);
  if (!near(data.gyro[2], kOneDegreeRad, 1.0e-6f)) {
    return 1;
  }
  if (!near(data.angle[0], kOneDegreeRad, 5.0e-4f)) {
    return 1;
  }
  return 0;
}
