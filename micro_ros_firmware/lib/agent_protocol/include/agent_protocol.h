#ifndef AGENT_PROTOCOL_H
#define AGENT_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AGENT_LINK_SOFT_STOP 1U
#define AGENT_LINK_REVOKE 2U

// Stable wire opcode numbers; parameter writes, zeroing and SAVE are retired.
bool agent_config_operation_supported(uint8_t operation);

typedef struct {
  uint32_t command_sequence;
  uint32_t command_tick_ms;
  uint32_t highest_policy_generation;
  uint32_t barrier_generation;
  uint32_t stop_generation;
  bool have_command;
  bool lease_valid;
  bool authorized;
  bool connected;
  bool clock_valid;
  bool stop_pending;
  bool soft_stopped;
  bool revoked;
} agent_link_t;

bool agent_deadline_valid(uint32_t now_ms, uint32_t deadline_ms);
bool agent_request_deadline(uint64_t issued_ns, uint16_t ttl_ms,
    uint16_t ttl_limit_ms, uint64_t now_epoch_ns, uint32_t now_tick_ms,
    uint32_t *deadline_ms);
void agent_link_reset(agent_link_t *link);
void agent_link_accept_command(agent_link_t *link, uint32_t sequence, uint32_t now_ms);
bool agent_link_accept_policy(agent_link_t *link, uint32_t generation, uint32_t now_ms);
void agent_link_request_stop(agent_link_t *link);
uint8_t agent_link_poll(agent_link_t *link, uint32_t now_ms,
    bool connected, bool clock_valid);

#ifdef __cplusplus
}
#endif
#endif
