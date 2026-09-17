// 加热器：TIM3 HardwareTimer 唯一所有者 + 纯 PID（官方 imu_temp_ctrl 系数）。
#include "heater.h"

// 输出单位为 TIM3 CCR 计数：ARR=10000 → PWM 500Hz，占空比 = duty/(ARR+1)。
static constexpr float kDesiredTempC = 40.0f;
static constexpr float kPidKp = 100.0f;  // 每 °C 误差的 CCR 增量
static constexpr float kPidKi = 50.0f;
static constexpr float kPidKd = 10.0f;
static constexpr float kMaxCcr = 500.0f; // CCR 上限（≤ ARR=10000，即 5%）

extern "C" float heater_pid_update(heater_pid_state_t *state,
                                   float measured_c) {
  const float err = kDesiredTempC - measured_c;
  float out = kPidKp * err +
              kPidKi * (err + state->err_l + state->err_ll) +
              kPidKd * (err - state->err_l);
  state->err_ll = state->err_l;
  state->err_l = err;
  if (out > kMaxCcr) {
    out = kMaxCcr;
  }
  if (out < 0.0f) {
    out = 0.0f;
  }
  return out;
}

#ifndef HOST_TEST
#include <Arduino.h>
#include <HardwareTimer.h>

namespace {
HardwareTimer &heater_timer() {
  static HardwareTimer tim3(TIM3);
  return tim3;
}
} // namespace

extern "C" void heater_init(void) {
  // 官方 TIM3：PSC=24-1、ARR=10000-1，APB1=120MHz → PWM 500Hz，初始占空比 0
  heater_timer().setPWM(4, PB1_ALT1, 500, 0);
}

extern "C" void heater_set_duty(uint16_t ccr) {
  heater_timer().setCaptureCompare(4, ccr);
}
#endif
