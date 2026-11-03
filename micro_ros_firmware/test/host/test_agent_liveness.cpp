#include "agent_liveness.h"
#include <cassert>
#include <cstdint>

int main() {
  agent_liveness_t state;
  agent_liveness_init(&state, 0);
  assert(agent_liveness_probe_due(&state, 0));
  assert(agent_liveness_probe_result(&state, 5, true));
  assert(!agent_liveness_probe_due(&state, 1004));
  assert(agent_liveness_probe_due(&state, 1005));
  // A valid 50 Hz command stream needs no extra Agent probes.
  for (uint32_t now = 20; now < 5000; now += 20) {
    agent_liveness_received(&state, now);
    assert(!agent_liveness_probe_due(&state, now));
  }
  assert(!agent_liveness_probe_due(&state, 5979));
  assert(agent_liveness_probe_due(&state, 5980));
  assert(agent_liveness_probe_result(&state, 5985, false));
  assert(!agent_liveness_probe_due(&state, 5986));
  assert(agent_liveness_probe_due(&state, 6985));
  assert(agent_liveness_probe_result(&state, 6990, false));
  assert(!agent_liveness_probe_result(&state, 7995, false));
  agent_liveness_received(&state, 7996);
  assert(state.failures == 0);
  assert(!agent_liveness_probe_due(&state, 7996));
  agent_liveness_init(&state, UINT32_MAX - 500);
  assert(agent_liveness_probe_result(&state, UINT32_MAX - 500, true));
  assert(!agent_liveness_probe_due(&state, 498));
  assert(agent_liveness_probe_due(&state, 499));
}
