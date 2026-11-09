// Exhaustive value contract between the independently owned mailbox and protocol enums.

#ifndef MOTOR_CONFIG_CONTRACT_HPP
#define MOTOR_CONFIG_CONTRACT_HPP

#include "control_mailbox.h"
#include "motor_protocol_adapter.h"

#define MOTOR_CONFIG_KIND_SHARED_LIST(X) \
  X(SET_MODE, 1U)                     \
  X(SET_GAINS, 2U)                    \
  X(SET_POSITION_LIMITS, 3U)          \
  X(SET_ENABLE, 4U)                   \
  X(SET_VELOCITY_LIMIT, 5U)           \
  X(SET_TORQUE_LIMIT, 6U)             \
  X(REFRESH_PARAMS, 8U)               \
  X(SET_ZERO, 9U)                     \
  X(CLEAR_FAULT, 10U)                 \
  X(SET_POLICY, 11U)                  \
  X(SET_SAFETY_PROTECT, 12U)          \
  X(SET_DIRECTION, 13U)               \
  X(SAVE_DEVICE_FLASH, 14U)             \
  X(SET_T_MAX, 15U)                    \
  X(SET_NODE_ID, 16U)

#define MOTOR_CONFIG_KIND_ASSERT(name, value)                                \
  static_assert(static_cast<int>(CONTROL_MAILBOX_CONFIG_ ## name) == value,    \
    "mailbox config kind numeric contract changed");          \
  static_assert(static_cast<int>(MOTOR_PROTOCOL_CONFIG_ ## name) == value,     \
    "protocol config kind numeric contract changed");         \
  static_assert(static_cast<int>(CONTROL_MAILBOX_CONFIG_ ## name) ==           \
    static_cast<int>(MOTOR_PROTOCOL_CONFIG_ ## name),          \
    "mailbox/protocol config kind values must match");

MOTOR_CONFIG_KIND_SHARED_LIST(MOTOR_CONFIG_KIND_ASSERT);

#undef MOTOR_CONFIG_KIND_ASSERT

#endif // MOTOR_CONFIG_CONTRACT_HPP
