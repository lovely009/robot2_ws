#include "agent_protocol.h"

bool agent_config_operation_supported(uint8_t operation) {
  return operation == 0U || operation == 2U || operation == 3U || operation == 4U ||
         operation == 6U || operation == 7U || operation == 8U;
}
#include <stddef.h>
#include <string.h>

bool agent_deadline_valid(uint32_t now_ms, uint32_t deadline_ms) {
  const uint32_t remaining = deadline_ms - now_ms;
  return remaining > 0U && remaining <= 30U;
}

bool agent_request_deadline(uint64_t issued_ns, uint16_t ttl_ms,
    uint16_t ttl_limit_ms, uint64_t now_epoch_ns, uint32_t now_tick_ms,
    uint32_t *deadline_ms) {
  if (deadline_ms == NULL || issued_ns == 0U || now_epoch_ns == 0U ||
      ttl_ms == 0U || ttl_ms > ttl_limit_ms) return false;
  uint64_t remaining_ns = (uint64_t)ttl_ms * 1000000ULL;
  if (issued_ns > now_epoch_ns) {
    if (issued_ns - now_epoch_ns > 20000000ULL) return false;
    // 容忍主机小幅领先，但不把未来偏差加到执行窗口。
  } else {
    const uint64_t age_ns = now_epoch_ns - issued_ns;
    if (age_ns >= remaining_ns) return false;
    remaining_ns -= age_ns;
  }
  const uint32_t remaining_ms = (uint32_t)(remaining_ns / 1000000ULL);
  if (remaining_ms == 0U) return false;
  *deadline_ms = now_tick_ms + remaining_ms;
  return true;
}

void agent_link_reset(agent_link_t *link) {
  if (link != NULL) memset(link, 0, sizeof(*link));
}

void agent_link_accept_command(agent_link_t *link, uint32_t sequence, uint32_t now_ms) {
  if (link == NULL) return;
  link->command_sequence = sequence;
  link->command_tick_ms = now_ms;
  link->have_command = true;
  link->lease_valid = true;
  link->soft_stopped = false;
  link->revoked = false;
}

bool agent_link_accept_policy(agent_link_t *link, uint32_t generation, uint32_t now_ms) {
  if (link == NULL || generation == 0U || link->stop_pending ||
      !link->connected || !link->clock_valid || !link->have_command ||
      (uint32_t)(now_ms - link->command_tick_ms) > 30U ||
      generation < link->highest_policy_generation ||
      (!link->authorized && generation <= link->barrier_generation)) return false;
  link->highest_policy_generation = generation;
  link->authorized = true;
  link->revoked = false;
  return true;
}

void agent_link_request_stop(agent_link_t *link) {
  if (link == NULL) return;
  link->barrier_generation = link->highest_policy_generation;
  link->stop_pending = true;
  link->authorized = false;
}

uint8_t agent_link_poll(agent_link_t *link, uint32_t now_ms,
    bool connected, bool clock_valid) {
  if (link == NULL) return 0U;
  link->connected = connected;
  link->clock_valid = clock_valid;
  const uint32_t age = now_ms - link->command_tick_ms;
  link->lease_valid = link->have_command && age <= 30U && connected && clock_valid;
  if (link->stop_pending || ((!connected || !clock_valid ||
      (link->have_command && age > 100U)) && !link->revoked)) {
    link->barrier_generation = link->highest_policy_generation;
    link->authorized = false;
    link->stop_pending = false;
    link->soft_stopped = true;
    link->revoked = true;
    ++link->stop_generation;
    return AGENT_LINK_REVOKE;
  }
  if (link->have_command && age > 30U && !link->soft_stopped) {
    link->soft_stopped = true;
    return AGENT_LINK_SOFT_STOP;
  }
  return 0U;
}
