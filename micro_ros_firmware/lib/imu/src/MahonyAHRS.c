// Madgwick 的实现版 Mahony AHRS（官方 CtrBoard-H7_ALL 例程移植）。
// sampleFreq 与 BMI088 加速度计 ODR=100Hz 对齐：ACC_INT(PE10) 每周期触发
// sensor_task 信号量，一次信号量对应一次采样，姿态积分 dt=1/100s。
// 注意：改 ODR 必须同步改这里的 sampleFreq，否则姿态积分尺度错误。

#include "MahonyAHRS.h"
#include <math.h>
#include <stdint.h>

#define sampleFreq 100.0f
#define twoKpDef (2.0f * 0.5f)
#define twoKiDef (2.0f * 0.0f)

static volatile float twoKp = twoKpDef;
static volatile float twoKi = twoKiDef;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float invSqrt(float x);

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax,
                         float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q[1] * q[3] - q[0] * q[2];
    halfvy = q[0] * q[1] + q[2] * q[3];
    halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * (1.0f / sampleFreq));
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

static float invSqrt(float x) {
  float halfx = 0.5f * x;
  union {
    float f;
    uint32_t u;
  } bit = {x};
  bit.u = 0x5f3759df - (bit.u >> 1);
  bit.f = bit.f * (1.5f - (halfx * bit.f * bit.f));
  return bit.f;
}
