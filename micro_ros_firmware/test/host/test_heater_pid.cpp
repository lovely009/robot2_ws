// heater PID 纯函数特征：钳位、误差递推（系数移植自官方 imu_temp_ctrl）。
#include "heater.h"

#include <cassert>
#include <cmath>

static bool near(float a, float b) {
  return std::fabs(a - b) < 1e-4f;
}

int main() {
  heater_pid_state_t s{};

  // 冷板（0°C）：首轮 P=100*40 + I=50*40 = 6000 → 钳到上限 500。
  assert(near(heater_pid_update(&s, 0.0f), 500.0f));

  // 高温（80°C，误差 -40）：输出钳到 0。
  heater_pid_state_t hot{};
  assert(near(heater_pid_update(&hot, 80.0f), 0.0f));

  // 恰在目标：稳态输出 0，状态误差归零。
  heater_pid_state_t at{};
  assert(near(heater_pid_update(&at, 40.0f), 0.0f));
  assert(near(at.err_l, 0.0f) && near(at.err_ll, 0.0f));

  // 误差递推：连续两次 39°C（e=1），第二次用上历史项
  // out = 100*1 + 50*(1+1+0) + 10*(1-1) = 200。
  heater_pid_state_t warm{};
  (void)heater_pid_update(&warm, 39.0f);
  assert(near(heater_pid_update(&warm, 39.0f), 200.0f));
  return 0;
}
