#include "control_lease.h"
#include "control_mailbox.h"
#include "motor_control_internal.h"
#include "motor_control_profile.h"

#include <cassert>
#include <cstring>
#include <cstdio>
#include <limits>

static constexpr uint64_t kSession = UINT64_C(1800000000000);
static constexpr uint64_t kBase = kSession * CONTROL_LEASE_SESSION_STRIDE;

static control_lease_state_t fresh() {
  control_lease_state_t state{};
  control_lease_init(&state);
  assert(!control_lease_allows(&state, kBase + 1, 100));
  control_lease_set_session(&state, kSession);
  assert(control_lease_take_revocation(&state));
  return state;
}

static void test_acceptance_and_strict_renewal() {
  auto s = fresh();
  assert(control_lease_submit(&s, kSession, kBase + 1, 1, 300, true, 100));
  assert(control_lease_take_revocation(&s));
  assert(control_lease_allows(&s, kBase + 1, 299));
  assert(!control_lease_allows(&s, kBase + 1, 300));
  assert(!control_lease_allows(&s, kBase + 2, 150));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 350, true, 150));
  assert(s.sequence == 1 && s.valid_until_tick_ms == 300);
  assert(control_lease_submit(&s, kSession, kBase + 1, 2, 350, true, 150));
  assert(!control_lease_take_revocation(&s));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 400, true, 160));
  assert(s.sequence == 2 && s.valid_until_tick_ms == 350);
}

static void test_invalid_inputs_do_not_claim_generation() {
  auto s = fresh();
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 100, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 99, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 401, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 0, 200, true, 100));
  assert(!control_lease_submit(&s, kSession + 1, kBase + 1, 1, 200, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase, 1, 200, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase - 1, 1, 200, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase + 1000000, 1, 200, true, 100));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 0, false, 100));
  assert(s.lease_id == 0 && !s.active);
  control_lease_set_session(&s, UINT64_MAX);
  assert(!control_lease_submit(&s, UINT64_MAX, UINT64_MAX, 1, 200, true, 100));
}

static void test_expiry_cannot_be_renewed_or_replayed() {
  auto s = fresh();
  assert(control_lease_submit(&s, kSession, kBase + 1, 1, 200, true, 100));
  assert(control_lease_take_revocation(&s));
  control_lease_service(&s, 200);
  assert(!s.active && s.lease_id == kBase + 1);
  assert(control_lease_take_revocation(&s));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 2, 400, true, 200));
  assert(control_lease_submit(&s, kSession, kBase + 2, 1, 400, true, 200));
  assert(control_lease_take_revocation(&s));
  assert(!control_lease_allows(&s, kBase + 1, 201));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 3, 500, true, 250));
  assert(s.lease_id == kBase + 2 && s.valid_until_tick_ms == 400);
  // Expiry is checked inside receipt too, even if the next 1 ms service is late.
  assert(!control_lease_submit(&s, kSession, kBase + 2, 2, 600, true, 400));
  assert(!s.active && control_lease_take_revocation(&s));
}

static void test_explicit_revoke_and_session_fence() {
  auto s = fresh();
  assert(control_lease_submit(&s, kSession, kBase + 1, 1, 300, true, 100));
  control_lease_take_revocation(&s);
  assert(!control_lease_submit(&s, kSession, kBase + 1, 1, 0, false, 110));
  assert(control_lease_submit(&s, kSession, kBase + 1, 2, 0, false, 110));
  assert(!s.active && control_lease_take_revocation(&s));
  assert(!control_lease_submit(&s, kSession, kBase + 1, 3, 300, true, 120));
  assert(control_lease_submit(&s, kSession, kBase + 2, 1, 300, true, 120));
  control_lease_set_session(&s, kSession + 1);
  assert(!s.active && control_lease_take_revocation(&s));
  assert(!control_lease_submit(&s, kSession + 1, kBase + 2, 2, 300, true, 130));
  assert(control_lease_submit(&s, kSession + 1, kBase + 1000001, 1, 300, true, 130));
  assert(!control_lease_allows(&s, kBase + 2, 140));
}

static void test_tick_wrap_and_sequence_exhaustion() {
  auto s = fresh();
  const uint32_t now = UINT32_MAX - 50;
  assert(control_lease_submit(&s, kSession, kBase + 999999, UINT32_MAX, 49, true, now));
  assert(control_lease_allows(&s, kBase + 999999, 48));
  assert(!control_lease_submit(&s, kSession, kBase + 999999, 0, 200, true, 20));
  assert(!control_lease_submit(&s, kSession, kBase + 1000000, 1, 200, true, 20));
  control_lease_service(&s, 49);
  assert(!s.active && s.lease_id == kBase + 999999);
}

static void test_new_lease_requires_kernel_stop_and_expiry_cancels_trajectory() {
  motor_control_context_t kernel{};
  const motor_protocol_adapter_t *adapters[] = {&motor_protocol_cerb_adapter};
  motor_control_init(&kernel, adapters, 1);
  motor_control_set_agent_connected(&kernel, true, 100);
  motor_control_set_authorization(&kernel, true, 100);
  auto &e = kernel.entries[kernel.entry_count++];
  e.used = e.online = true;
  e.motor_id = e.node = 1;
  e.adapter = adapters[0];
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const auto *p = motor_control_profile_at(i);
    if (std::strcmp(p->model_name, "cerb_r48_101") == 0) e.profile = p;
  }
  assert(e.profile);
  e.lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
  e.motion_enabled = e.motion_authorized = e.policy_enable = true;
  e.last_rx_tick_ms = e.last_motion_rx_tick_ms = 100;
  e.identity_read_done = e.dev_read_done = e.boot_record_offered = true;
  motor_control_trajectory_t trajectory{};
  trajectory.trajectory_id = 9;
  trajectory.target_count = 1;
  trajectory.target_ids[0] = 1;
  trajectory.point_count = 2;
  trajectory.points[0] = {0, 1, 0.f, 0.f, 0.f};
  trajectory.points[1] = {2000, 1, 1.f, 0.f, 0.f};
  assert(motor_control_apply_trajectory(&kernel, &trajectory, 100));
  assert(kernel.trajectory_active);
  auto s = fresh();
  assert(control_lease_submit(&s, kSession, kBase + 1, 1, 200, true, 100));
  control_lease_take_revocation(&s); // This lease already owns the active test motion.
  control_lease_service(&s, 199);
  assert(!control_lease_take_revocation(&s));
  assert(kernel.trajectory_active);
  control_lease_service(&s, 200);
  assert(control_lease_take_revocation(&s));
  // The same kernel revoke used by the 1 ms motor task cancels buffered motion.
  motor_control_set_authorization(&kernel, false, 200);
  assert(kernel.agent_connected); // Agent remains alive throughout.
  assert(!kernel.trajectory_active && !e.policy_enable && !e.motion_enabled);
  assert(motor_control_take_tx_cancellations(&kernel) != 0);
  assert(control_lease_submit(&s, kSession, kBase + 2, 1, 400, true, 201));
  assert(control_lease_take_revocation(&s));
  motor_control_set_authorization(&kernel, false, 201);
  motor_control_set_authorization(&kernel, true, 201);
  assert(!e.policy_enable && !e.motion_enabled && !kernel.trajectory_active);
}

static void test_trajectory_segments_cannot_mix_leases() {
  control_trajectory_mailbox_t box{};
  control_trajectory_mailbox_init(&box);
  control_trajectory_t first{};
  first.lease_id = kBase + 1;
  first.trajectory_id = 1;
  first.segment_count = 2;
  first.target_count = first.point_count = 1;
  first.target_ids[0] = 1;
  first.points[0] = {0, 1, 0.f, 0.f, 0.f};
  assert(control_trajectory_mailbox_publish(&box, &first));
  auto second = first;
  second.lease_id = kBase + 2;
  second.segment_index = 1;
  second.points[0].time_from_start_ms = 100;
  assert(!control_trajectory_mailbox_publish(&box, &second));
  control_trajectory_t out{};
  assert(!control_trajectory_mailbox_take(&box, &out));
}

int main() {
  test_acceptance_and_strict_renewal();
  test_invalid_inputs_do_not_claim_generation();
  test_expiry_cannot_be_renewed_or_replayed();
  test_explicit_revoke_and_session_fence();
  test_tick_wrap_and_sequence_exhaustion();
  test_new_lease_requires_kernel_stop_and_expiry_cancels_trajectory();
  test_trajectory_segments_cannot_mix_leases();
  std::puts("control lease guards and buffered trajectory expiry passed");
}
