// 板温加热（TIM3_CH4/PB1）：PWM 所有权在 .cpp，PID 为不依赖 Arduino 的
// 纯函数（HOST_TEST 下可直编，见 test_heater_pid）。
#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float err_l;  // 上一次误差（°C）
  float err_ll; // 上两次误差（°C）
} heater_pid_state_t;

// 目标 40°C 增量式 PID：返回 TIM3 CCR，钳在 [0,500]
// （PWM 500Hz/ARR=10000，500 即占空比上限 5%）。
float heater_pid_update(heater_pid_state_t *state, float measured_c);

// TIM3_CH4/PB1 初始化（500Hz、初始占空比 0）。
void heater_init(void);

// 设置 CCR（≤ ARR=10000）。
void heater_set_duty(uint16_t ccr);

#ifdef __cplusplus
}
#endif

#endif
