#include <cassert>
#include <cstdint>

#include "supervisor_heartbeat.h"

static void test_first_heartbeat_starts_a_progress_window() {
  supervisor_heartbeat_state_t state{};

  assert(!supervisor_heartbeat_evaluate(&state, 1U, 100U, 300U));
  assert(state.last_progress_ms == 100U);
}

static void test_control_startup_is_unseen_until_its_first_heartbeat() {
  supervisor_heartbeat_state_t state{};

  static_assert(SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS == 100U);
  assert(!supervisor_heartbeat_evaluate(
      &state, 0U, 0U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
  assert(!supervisor_heartbeat_evaluate(
      &state, 0U, 500U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
  assert(!supervisor_heartbeat_evaluate(
      &state, 1U, 500U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
  assert(!supervisor_heartbeat_evaluate(
      &state, 1U, 599U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
  assert(supervisor_heartbeat_evaluate(
      &state, 1U, 600U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
  assert(!supervisor_heartbeat_evaluate(
      &state, 2U, 601U, SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS));
}

static void test_elapsed_time_is_wrap_safe_at_the_timeout_boundary() {
  supervisor_heartbeat_state_t state{};
  assert(!supervisor_heartbeat_evaluate(&state, 7U, 0xfffffff0U, 64U));

  assert(!supervisor_heartbeat_evaluate(&state, 7U, 0x0000002fU, 64U));
  assert(supervisor_heartbeat_evaluate(&state, 7U, 0x00000030U, 64U));
}

static void test_heartbeat_progress_clears_a_previous_stall() {
  supervisor_heartbeat_state_t state{};
  assert(!supervisor_heartbeat_evaluate(&state, 3U, 100U, 300U));
  assert(supervisor_heartbeat_evaluate(&state, 3U, 400U, 300U));

  assert(!supervisor_heartbeat_evaluate(&state, 4U, 401U, 300U));
  assert(state.last_progress_ms == 401U);
}

static void test_micro_ros_normal_six_hundred_ms_ping_is_not_a_stall() {
  supervisor_heartbeat_state_t state{};
  assert(!supervisor_heartbeat_evaluate(&state, 1U, 0U, 1500U));
  assert(!supervisor_heartbeat_evaluate(&state, 1U, 600U, 1500U));
  assert(supervisor_heartbeat_evaluate(&state, 1U, 1500U, 1500U));
}

static void test_degraded_sensor_retry_interval_is_not_a_stall() {
  supervisor_heartbeat_state_t state{};
  assert(!supervisor_heartbeat_evaluate(&state, 12U, 0U, 2500U));
  assert(!supervisor_heartbeat_evaluate(&state, 12U, 1000U, 2500U));
  assert(supervisor_heartbeat_evaluate(&state, 12U, 2500U, 2500U));
}

int main() {
  test_first_heartbeat_starts_a_progress_window();
  test_control_startup_is_unseen_until_its_first_heartbeat();
  test_elapsed_time_is_wrap_safe_at_the_timeout_boundary();
  test_heartbeat_progress_clears_a_previous_stall();
  test_micro_ros_normal_six_hundred_ms_ping_is_not_a_stall();
  test_degraded_sensor_retry_interval_is_not_a_stall();
  return 0;
}
