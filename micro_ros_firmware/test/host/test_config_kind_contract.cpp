// Compile-time bridge contract for the distinct mailbox/protocol config enums.

#include "motor_config_contract.hpp"

static_assert(MOTOR_PROTOCOL_CONFIG_READ_IDENTITY == 7U,
    "READ_IDENTITY remains protocol-only and has no mailbox kind");

int main() {
  return 0;
}
