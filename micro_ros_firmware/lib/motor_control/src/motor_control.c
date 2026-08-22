#include "motor_control.h"

#include <math.h>
#include <string.h>

static uint32_t elapsed_ms(uint32_t now, uint32_t then) {
  return now - then;
}

static void latch_vaxor_stop(motor_control_context_t *ctx,
                              size_t motor_index) {
  motor_control_motor_runtime_t *runtime = &ctx->runtime[motor_index];
  if (ctx->motors[motor_index].protocol == MOTOR_CONTROL_VAXOR &&
      runtime->vaxor_enabled) {
    runtime->vaxor_stop_pending = true;
    runtime->vaxor_enabled = false;
  }
}

static void latch_all_motors(motor_control_context_t *ctx) {
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    latch_vaxor_stop(ctx, index);
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    runtime->state = MOTOR_CONTROL_FAULT_LATCHED;
    runtime->command_valid = false;
    runtime->reauthorization_required = true;
  }
}

static void mark_rp_bus_disable(motor_control_context_t *ctx,
                                uint8_t bus_index) {
  if (bus_index >= MOTOR_CONTROL_MAX_BUSES) {
    return;
  }
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].protocol == MOTOR_CONTROL_RP &&
        ctx->motors[index].bus_index == bus_index) {
      ctx->rp_bus_disabled[bus_index] = true;
      return;
    }
  }
}

static void mark_all_rp_buses_disabled(motor_control_context_t *ctx) {
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].protocol == MOTOR_CONTROL_RP) {
      mark_rp_bus_disable(ctx, ctx->motors[index].bus_index);
    }
  }
}

static void pack_rp_neutral(uint8_t out[8]) {
  const motor_protocol_joint_command_t neutral = {0.0, 0.0, 0.0, 0.0, 0.0};
  motor_protocol_pack_rp_slot(&neutral, out);
}

static void pack_rp_runtime(const motor_control_motor_config_t *motor,
                            const motor_control_motor_runtime_t *runtime,
                            uint8_t out[8]) {
  if (runtime->state == MOTOR_CONTROL_ACTIVE) {
    motor_protocol_pack_rp_slot(&runtime->command, out);
    return;
  }
  if (runtime->state == MOTOR_CONTROL_SOFT_STOP) {
    const motor_protocol_joint_command_t damping = {
        0.0, 0.0, 0.0, motor->default_kd, 0.0};
    motor_protocol_pack_rp_slot(&damping, out);
    return;
  }
  pack_rp_neutral(out);
}

static bool is_first_rp_motor_on_bus(const motor_control_context_t *ctx,
                                     size_t motor_index) {
  uint8_t bus_index = ctx->motors[motor_index].bus_index;
  for (size_t index = 0; index < motor_index; ++index) {
    if (ctx->motors[index].protocol == MOTOR_CONTROL_RP &&
        ctx->motors[index].bus_index == bus_index) {
      return false;
    }
  }
  return true;
}

static void fill_rp_control_action(motor_control_tx_action_t *action,
                                   uint8_t bus_index, bool enabled) {
  memset(action, 0, sizeof(*action));
  action->bus_index = bus_index;
  action->can_id = MOTOR_PROTOCOL_RP_ENABLE_ID;
  action->size = 8U;
  action->is_fd = false;
  action->is_extended = false;
  action->data[0] = enabled ? 1U : 0U;
}

static void fill_rp_command_action(const motor_control_context_t *ctx,
                                   uint8_t bus_index,
                                   motor_control_tx_action_t *action) {
  memset(action, 0, sizeof(*action));
  action->bus_index = bus_index;
  action->can_id = MOTOR_PROTOCOL_RP_COMMAND_ID;
  action->size = 64U;
  action->is_fd = true;
  action->is_extended = false;
  for (size_t slot = 0; slot < MOTOR_PROTOCOL_RP_BROADCAST_SLOTS; ++slot) {
    pack_rp_neutral(action->data + slot * 8U);
  }

  for (size_t peer = 0; peer < ctx->motor_count; ++peer) {
    const motor_control_motor_config_t *peer_motor = &ctx->motors[peer];
    if (peer_motor->protocol != MOTOR_CONTROL_RP ||
        peer_motor->bus_index != bus_index || peer_motor->id == 0U ||
        peer_motor->id > MOTOR_PROTOCOL_RP_BROADCAST_SLOTS) {
      continue;
    }
    pack_rp_runtime(peer_motor, &ctx->runtime[peer],
                    action->data + (peer_motor->id - 1U) * 8U);
  }
}

static bool rp_bus_has_active_command(const motor_control_context_t *ctx,
                                      uint8_t bus_index) {
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].protocol == MOTOR_CONTROL_RP &&
        ctx->motors[index].bus_index == bus_index &&
        ctx->runtime[index].state == MOTOR_CONTROL_ACTIVE) {
      return true;
    }
  }
  return false;
}

static size_t append_rp_actions(motor_control_context_t *ctx,
                                motor_control_tx_action_t *actions,
                                size_t action_capacity) {
  size_t action_count = 0U;
  if (actions == NULL) {
    return 0U;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    if (motor->protocol != MOTOR_CONTROL_RP ||
        !is_first_rp_motor_on_bus(ctx, index)) {
      continue;
    }
    bool bus_fault_active =
        motor->bus_index < MOTOR_CONTROL_MAX_BUSES &&
        ctx->bus_fault[motor->bus_index];
    bool bus_disabled = motor->bus_index < MOTOR_CONTROL_MAX_BUSES &&
                        ctx->rp_bus_disabled[motor->bus_index];
    if (!ctx->authorized || bus_fault_active) {
      bus_disabled = true;
      if (motor->bus_index < MOTOR_CONTROL_MAX_BUSES) {
        ctx->rp_bus_disabled[motor->bus_index] = true;
      }
    }

    if (bus_disabled) {
      bool recovery_ready =
          ctx->authorized && !bus_fault_active &&
          rp_bus_has_active_command(ctx, motor->bus_index);
      if (recovery_ready && action_capacity - action_count >= 2U) {
        fill_rp_control_action(&actions[action_count++], motor->bus_index,
                               true);
        fill_rp_command_action(ctx, motor->bus_index,
                               &actions[action_count++]);
        if (motor->bus_index < MOTOR_CONTROL_MAX_BUSES) {
          ctx->rp_bus_disabled[motor->bus_index] = false;
        }
      } else if (action_count < action_capacity) {
        fill_rp_control_action(&actions[action_count++], motor->bus_index,
                               false);
      }
      continue;
    }

    if (action_count < action_capacity) {
      fill_rp_command_action(ctx, motor->bus_index,
                             &actions[action_count++]);
    }
  }
  return action_count;
}

static void fill_vaxor_control_action(motor_control_tx_action_t *action,
                                       const motor_control_motor_config_t *motor,
                                       bool run, uint8_t mode) {
  const motor_protocol_wrist_control_t control = {run, false, mode};
  memset(action, 0, sizeof(*action));
  action->bus_index = motor->bus_index;
  action->can_id = motor_protocol_wrist_control_id(motor->id);
  action->size = 8U;
  action->is_fd = false;
  action->is_extended = false;
  motor_protocol_pack_wrist_control(&control, action->data);
}

static void fill_vaxor_command_action(
    motor_control_tx_action_t *action,
    const motor_control_motor_config_t *motor,
    const motor_protocol_joint_command_t *joint_command) {
  motor_protocol_joint_command_t motor_command = *joint_command;
  motor_command.position *= motor->gear_ratio;
  motor_command.velocity *= motor->gear_ratio;
  memset(action, 0, sizeof(*action));
  action->bus_index = motor->bus_index;
  action->can_id = motor_protocol_wrist_command_id(motor->id);
  action->size = 12U;
  action->is_fd = true;
  action->is_extended = false;
  motor_protocol_pack_wrist_command(&motor_command, action->data);
}

static size_t append_vaxor_actions(motor_control_context_t *ctx,
                                   motor_control_tx_action_t *actions,
                                   size_t action_capacity) {
  size_t action_count = 0U;
  if (actions == NULL) {
    return 0U;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    if (motor->protocol != MOTOR_CONTROL_VAXOR) {
      continue;
    }

    if (runtime->state == MOTOR_CONTROL_DISABLED ||
        runtime->state == MOTOR_CONTROL_FAULT_LATCHED) {
      if ((runtime->vaxor_stop_pending || runtime->vaxor_enabled) &&
          action_count < action_capacity) {
        fill_vaxor_control_action(&actions[action_count++], motor, false,
                                  runtime->active_mode);
        runtime->vaxor_enabled = false;
        runtime->vaxor_stop_pending = false;
      }
      continue;
    }

    uint8_t target_mode = runtime->state == MOTOR_CONTROL_SOFT_STOP
                              ? (uint8_t)MOTOR_PROTOCOL_WRIST_MODE_MIT
                              : runtime->requested_mode;
    bool mode_transition =
        !runtime->vaxor_enabled || runtime->active_mode != target_mode;
    bool stop_start_required = runtime->vaxor_stop_pending || mode_transition;
    size_t required_actions = stop_start_required ? 3U : 1U;
    if (required_actions > action_capacity - action_count) {
      continue;
    }

    if (stop_start_required) {
      fill_vaxor_control_action(&actions[action_count++], motor, false,
                                target_mode);
      runtime->vaxor_stop_pending = false;
      fill_vaxor_control_action(&actions[action_count++], motor, true,
                                target_mode);
      runtime->vaxor_enabled = true;
      runtime->active_mode = target_mode;
    }

    if (runtime->state == MOTOR_CONTROL_SOFT_STOP) {
      const motor_protocol_joint_command_t damping = {
          0.0, 0.0, 0.0, motor->default_kd, 0.0};
      fill_vaxor_command_action(&actions[action_count++], motor, &damping);
    } else {
      fill_vaxor_command_action(&actions[action_count++], motor,
                                 &runtime->command);
    }
  }
  return action_count;
}

void motor_control_init(motor_control_context_t *ctx,
                        const motor_control_motor_config_t *motors,
                        size_t motor_count) {
  if (ctx == NULL) {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->authorized = true;
  if (motors == NULL) {
    return;
  }
  for (size_t source = 0;
       source < motor_count && ctx->motor_count < MOTOR_CONTROL_MAX_MOTORS;
       ++source) {
    const char *joint_name = motors[source].joint_name;
    if (joint_name == NULL || joint_name[0] == '\0') {
      continue;
    }
    const char *terminator =
        memchr(joint_name, '\0', MOTOR_CONTROL_JOINT_NAME_CAPACITY);
    if (terminator == NULL) {
      continue;
    }
    size_t index = ctx->motor_count;
    size_t name_size = (size_t)(terminator - joint_name) + 1U;
    memcpy(ctx->joint_names[index], joint_name, name_size);
    ctx->motors[index] = motors[source];
    ctx->motors[index].joint_name = ctx->joint_names[index];
    ctx->runtime[index].state = MOTOR_CONTROL_DISABLED;
    ctx->runtime[index].active_mode = MOTOR_PROTOCOL_WRIST_MODE_MIT;
    ctx->runtime[index].requested_mode = MOTOR_PROTOCOL_WRIST_MODE_MIT;
    ++ctx->motor_count;
  }
}

bool motor_control_accept_command(motor_control_context_t *ctx,
                                  const motor_control_command_t *command,
                                  uint32_t now_ms) {
  if (ctx == NULL) {
    return false;
  }
  if (command == NULL || !isfinite(command->position) ||
      !isfinite(command->velocity) || !isfinite(command->effort)) {
    ++ctx->invalid_command_count;
    return false;
  }
  if (command->joint_name[0] == '\0' ||
      memchr(command->joint_name, '\0', sizeof(command->joint_name)) == NULL) {
    ++ctx->invalid_command_count;
    return false;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].joint_name != NULL &&
        strcmp(ctx->motors[index].joint_name, command->joint_name) == 0) {
      if (ctx->motors[index].protocol == MOTOR_CONTROL_VAXOR &&
          command->mode > MOTOR_PROTOCOL_WRIST_MODE_POSITION) {
        ++ctx->invalid_command_count;
        return false;
      }
      if (!ctx->authorized) {
        return false;
      }
      motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
      if (runtime->reauthorization_required) {
        return false;
      }
      runtime->command.position = command->position;
      runtime->command.velocity = command->velocity;
      runtime->command.kp = ctx->motors[index].default_kp;
      runtime->command.kd = ctx->motors[index].default_kd;
      runtime->command.feedforward = command->effort;
      runtime->last_command_ms = now_ms;
      runtime->command_valid = true;
      runtime->requested_mode = command->mode;
      ++ctx->valid_command_count;
      return true;
    }
  }
  ++ctx->invalid_command_count;
  return false;
}

size_t motor_control_step(motor_control_context_t *ctx, uint32_t now_ms,
                          motor_control_tx_action_t *actions,
                          size_t action_capacity) {
  if (ctx == NULL) {
    return 0U;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    if (runtime->fault_active || runtime->reauthorization_required ||
        runtime->state == MOTOR_CONTROL_FAULT_LATCHED) {
      runtime->state = MOTOR_CONTROL_FAULT_LATCHED;
      continue;
    }
    if (!runtime->command_valid) {
      runtime->state = MOTOR_CONTROL_DISABLED;
      continue;
    }
    uint32_t age = elapsed_ms(now_ms, runtime->last_command_ms);
    if (age >= 300U) {
      runtime->command_valid = false;
      latch_vaxor_stop(ctx, index);
      runtime->state = MOTOR_CONTROL_DISABLED;
    } else if (age >= 100U) {
      runtime->state = MOTOR_CONTROL_SOFT_STOP;
    } else {
      runtime->state = MOTOR_CONTROL_ACTIVE;
    }
  }
  size_t action_count = append_rp_actions(ctx, actions, action_capacity);
  if (action_count >= action_capacity || actions == NULL) {
    return action_count;
  }
  action_count += append_vaxor_actions(
      ctx, actions + action_count, action_capacity - action_count);
  return action_count;
}

void motor_control_set_agent_connected(motor_control_context_t *ctx,
                                       bool connected) {
  if (ctx == NULL) {
    return;
  }
  if (connected) {
    ctx->agent_connected = true;
    ctx->agent_ever_connected = true;
    return;
  }

  bool disconnected_after_connection =
      ctx->agent_ever_connected && ctx->agent_connected;
  ctx->agent_connected = false;
  if (disconnected_after_connection) {
    ctx->authorized = false;
    latch_all_motors(ctx);
    mark_all_rp_buses_disabled(ctx);
  }
}

void motor_control_set_bus_fault(motor_control_context_t *ctx,
                                 uint8_t bus_index, bool active) {
  if (ctx == NULL || bus_index >= MOTOR_CONTROL_MAX_BUSES) {
    return;
  }
  ctx->bus_fault[bus_index] = active;
  if (!active) {
    return;
  }
  mark_rp_bus_disable(ctx, bus_index);
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].bus_index != bus_index) {
      continue;
    }
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    latch_vaxor_stop(ctx, index);
    runtime->state = MOTOR_CONTROL_FAULT_LATCHED;
    runtime->command_valid = false;
    runtime->reauthorization_required = true;
  }
}

void motor_control_set_motor_fault(motor_control_context_t *ctx,
                                   size_t motor_index, bool active) {
  if (ctx == NULL || motor_index >= ctx->motor_count) {
    return;
  }
  motor_control_motor_runtime_t *runtime = &ctx->runtime[motor_index];
  runtime->fault_active = active;
  if (active) {
    latch_vaxor_stop(ctx, motor_index);
    runtime->state = MOTOR_CONTROL_FAULT_LATCHED;
    runtime->command_valid = false;
    runtime->reauthorization_required = true;
  }
}

bool motor_control_set_authorization(motor_control_context_t *ctx,
                                     bool authorized) {
  if (ctx == NULL) {
    return false;
  }
  if (!authorized) {
    ctx->authorized = false;
    latch_all_motors(ctx);
    mark_all_rp_buses_disabled(ctx);
    return true;
  }
  if (ctx->agent_ever_connected && !ctx->agent_connected) {
    return false;
  }
  for (size_t bus = 0; bus < MOTOR_CONTROL_MAX_BUSES; ++bus) {
    if (ctx->bus_fault[bus]) {
      return false;
    }
  }
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->runtime[index].fault_active) {
      return false;
    }
  }

  ctx->authorized = true;
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    if (runtime->reauthorization_required) {
      runtime->reauthorization_required = false;
      runtime->state = MOTOR_CONTROL_DISABLED;
    }
  }
  return true;
}
