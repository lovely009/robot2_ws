#include "motor_control.h"

#include <math.h>
#include <string.h>

static uint32_t elapsed_ms(uint32_t now, uint32_t then) {
  return now - then;
}

static void latch_motor_stop(motor_control_context_t *ctx,
                             size_t motor_index) {
  motor_control_motor_runtime_t *runtime = &ctx->runtime[motor_index];
  if (ctx->motors[motor_index].protocol == MOTOR_CONTROL_VAXOR &&
      runtime->vaxor_enabled) {
    runtime->vaxor_stop_pending = true;
    runtime->vaxor_enabled = false;
  } else if (ctx->motors[motor_index].protocol == MOTOR_CONTROL_CREB &&
             runtime->creb_enabled) {
    runtime->creb_disable_pending = true;
    runtime->creb_enabled = false;
  }
}

static void latch_all_motors(motor_control_context_t *ctx) {
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    latch_motor_stop(ctx, index);
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    runtime->state = MOTOR_CONTROL_FAULT_LATCHED;
    runtime->command_valid = false;
    runtime->reauthorization_required = true;
  }
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
      size_t required_actions = runtime->vaxor_mode_change_pending ? 2U : 1U;
      if ((runtime->vaxor_stop_pending || runtime->vaxor_enabled ||
           runtime->vaxor_mode_change_pending) &&
          required_actions <= action_capacity - action_count) {
        fill_vaxor_control_action(&actions[action_count++], motor, false,
                                  runtime->active_mode);
        runtime->vaxor_enabled = false;
        runtime->vaxor_stop_pending = false;
        if (runtime->vaxor_mode_change_pending) {
          fill_vaxor_control_action(&actions[action_count++], motor, false,
                                    runtime->requested_mode);
          runtime->active_mode = runtime->requested_mode;
          runtime->vaxor_mode_change_pending = false;
        }
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

static void fill_creb_action(motor_control_tx_action_t *action,
                             const motor_control_motor_config_t *motor,
                             const motor_protocol_joint_command_t *command,
                             bool enabled) {
  const motor_protocol_creb_limits_t limits = {
      motor->mit_position_limit_rad,
      motor->mit_velocity_limit_rad_s,
      motor->mit_torque_limit_nm,
  };
  memset(action, 0, sizeof(*action));
  action->bus_index = motor->bus_index;
  (void)motor_protocol_creb_command_id(motor->id, &action->can_id);
  action->size = 12U;
  action->is_fd = true;
  action->is_extended = false;
  (void)motor_protocol_pack_creb_mit_frame(command, &limits, enabled,
                                           action->data);
}

static size_t append_creb_actions(motor_control_context_t *ctx,
                                  uint32_t now_ms,
                                  motor_control_tx_action_t *actions,
                                  size_t action_capacity) {
  size_t action_count = 0U;
  if (actions == NULL) {
    return 0U;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    if (motor->protocol != MOTOR_CONTROL_CREB) {
      continue;
    }

    if (runtime->state == MOTOR_CONTROL_DISABLED ||
        runtime->state == MOTOR_CONTROL_FAULT_LATCHED) {
      if ((runtime->creb_disable_pending || runtime->creb_enabled) &&
          action_count < action_capacity) {
        const motor_protocol_joint_command_t neutral = {0};
        fill_creb_action(&actions[action_count++], motor, &neutral, false);
        runtime->creb_enabled = false;
        runtime->creb_disable_pending = false;
      }
      continue;
    }

    if (!runtime->creb_enabled) {
      if (action_count >= action_capacity) {
        continue;
      }
      const motor_protocol_joint_command_t neutral = {0};
      fill_creb_action(&actions[action_count++], motor, &neutral, true);
      runtime->creb_enabled = true;
      runtime->creb_disable_pending = false;
      runtime->creb_enable_started_ms = now_ms;
      continue;
    }

    if (elapsed_ms(now_ms, runtime->creb_enable_started_ms) < 100U ||
        action_count >= action_capacity) {
      continue;
    }

    if (runtime->state == MOTOR_CONTROL_SOFT_STOP) {
      const motor_protocol_joint_command_t damping = {
          0.0, 0.0, 0.0, motor->default_kd, 0.0};
      fill_creb_action(&actions[action_count++], motor, &damping, true);
    } else {
      fill_creb_action(&actions[action_count++], motor, &runtime->command,
                       true);
    }
  }
  return action_count;
}

static size_t append_creb_sync_actions(const motor_control_context_t *ctx,
                                       motor_control_tx_action_t *actions,
                                       size_t action_capacity) {
  bool bus_has_creb[MOTOR_CONTROL_MAX_BUSES] = {false};
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].protocol == MOTOR_CONTROL_CREB) {
      bus_has_creb[ctx->motors[index].bus_index] = true;
    }
  }

  size_t action_count = 0U;
  for (uint8_t bus = 0U; bus < MOTOR_CONTROL_MAX_BUSES; ++bus) {
    if (!bus_has_creb[bus] || action_count >= action_capacity) {
      continue;
    }
    motor_control_tx_action_t *action = &actions[action_count++];
    memset(action, 0, sizeof(*action));
    action->bus_index = bus;
    action->can_id = 0x080U;
    action->size = 0U;
    action->is_fd = false;
    action->is_extended = false;
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
    if (motors[source].bus_index >= MOTOR_CONTROL_MAX_BUSES) {
      continue;
    }
    if (motors[source].protocol == MOTOR_CONTROL_CREB &&
        (!motor_protocol_creb_node_valid(motors[source].id) ||
         !isfinite(motors[source].gear_ratio) ||
         !(motors[source].gear_ratio > 0.0) ||
         !isfinite(motors[source].torque_constant_nm_per_amp) ||
         !(motors[source].torque_constant_nm_per_amp > 0.0) ||
         !isfinite(motors[source].mit_position_limit_rad) ||
         !(motors[source].mit_position_limit_rad > 0.0) ||
         !isfinite(motors[source].mit_velocity_limit_rad_s) ||
         !(motors[source].mit_velocity_limit_rad_s > 0.0) ||
         !isfinite(motors[source].mit_torque_limit_nm) ||
         !(motors[source].mit_torque_limit_nm > 0.0))) {
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
    ctx->runtime[index].kp = motors[source].default_kp;
    ctx->runtime[index].kd = motors[source].default_kd;
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
      if (!ctx->authorized) {
        return false;
      }
      motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
      if (runtime->reauthorization_required) {
        return false;
      }
      runtime->command.position = command->position;
      runtime->command.velocity = command->velocity;
      runtime->command.kp = runtime->kp;
      runtime->command.kd = runtime->kd;
      runtime->command.feedforward = command->effort;
      runtime->last_command_ms = now_ms;
      runtime->command_valid = true;
      ++ctx->valid_command_count;
      return true;
    }
  }
  ++ctx->invalid_command_count;
  return false;
}

bool motor_control_accept_feedback(motor_control_context_t *ctx,
                                   const motor_control_rx_frame_t *frame) {
  if (ctx == NULL || frame == NULL || frame->size > sizeof(frame->data)) {
    if (ctx != NULL) {
      ++ctx->invalid_feedback_count;
    }
    return false;
  }

  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    if (motor->bus_index != frame->bus_index) {
      continue;
    }

    motor_protocol_joint_state_t decoded = {0};
    bool matched = false;
    if ((frame->can_id == motor_protocol_wrist_reply_id(motor->id) &&
         motor->protocol == MOTOR_CONTROL_VAXOR && frame->size == 12U) ||
        (frame->can_id == motor_protocol_wrist_status_id(motor->id) &&
         motor->protocol == MOTOR_CONTROL_VAXOR && frame->size == 20U)) {
      matched = motor_protocol_unpack_wrist_feedback(frame->data, frame->size,
                                                    &decoded);
      decoded.position /= motor->gear_ratio;
      decoded.velocity /= motor->gear_ratio;
      decoded.effort *= motor->torque_constant_nm_per_amp *
                        motor->gear_ratio * motor->gear_efficiency;
    } else if (motor->protocol == MOTOR_CONTROL_CREB && frame->is_fd &&
               !frame->is_extended && frame->size == 12U) {
      uint32_t feedback_id = 0U;
      if (motor_protocol_creb_feedback_id(motor->id, &feedback_id) &&
          frame->can_id == feedback_id) {
        matched = motor_protocol_unpack_creb_feedback(
            frame->data, frame->size, &decoded);
        decoded.velocity /= motor->gear_ratio;
        decoded.effort *=
            motor->torque_constant_nm_per_amp * motor->gear_ratio;
      }
    }

    if (!matched) {
      continue;
    }

    motor_control_joint_state_t *state = &ctx->joint_states[index];
    memset(state, 0, sizeof(*state));
    memcpy(state->joint_name, motor->joint_name,
           strlen(motor->joint_name) + 1U);
    state->position = decoded.position;
    state->velocity = decoded.velocity;
    state->effort = decoded.effort;
    ctx->joint_state_valid[index] = true;

    if (motor->protocol == MOTOR_CONTROL_VAXOR && frame->size == 20U) {
      motor_protocol_wrist_status_t decoded_status = {0};
      if (motor_protocol_unpack_wrist_status(frame->data, frame->size,
                                             &decoded_status)) {
        motor_control_motor_status_t *status = &ctx->motor_statuses[index];
        uint32_t next_sequence = status->sequence + 1U;
        memset(status, 0, sizeof(*status));
        memcpy(status->joint_name, motor->joint_name,
               strlen(motor->joint_name) + 1U);
        status->sequence = next_sequence;
        status->voltage_v = decoded_status.voltage_v;
        status->fault = decoded_status.fault;
        status->motor_state = decoded_status.motor_state;
        status->calibration_state = decoded_status.calibration_state;
        status->mode = decoded_status.mode;
        status->motor_type = decoded_status.motor_type;
        ctx->motor_status_valid[index] = true;
      }
    }
    ++ctx->valid_feedback_count;
    return true;
  }

  ++ctx->invalid_feedback_count;
  return false;
}

bool motor_control_get_joint_state(const motor_control_context_t *ctx,
                                   size_t index,
                                   motor_control_joint_state_t *out) {
  if (ctx == NULL || out == NULL || index >= ctx->motor_count ||
      !ctx->joint_state_valid[index]) {
    return false;
  }
  *out = ctx->joint_states[index];
  return true;
}

bool motor_control_get_motor_status(const motor_control_context_t *ctx,
                                    size_t index,
                                    motor_control_motor_status_t *out) {
  if (ctx == NULL || out == NULL || index >= ctx->motor_count ||
      !ctx->motor_status_valid[index]) {
    return false;
  }
  *out = ctx->motor_statuses[index];
  return true;
}

bool motor_control_set_mode(motor_control_context_t *ctx,
                            const char *joint_name, uint8_t mode) {
  if (ctx == NULL || joint_name == NULL || joint_name[0] == '\0' ||
      memchr(joint_name, '\0', MOTOR_CONTROL_JOINT_NAME_CAPACITY) == NULL) {
    return false;
  }
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    if (strcmp(motor->joint_name, joint_name) != 0) {
      continue;
    }
    if ((motor->protocol == MOTOR_CONTROL_VAXOR &&
         mode > MOTOR_PROTOCOL_WRIST_MODE_POSITION) ||
        (motor->protocol == MOTOR_CONTROL_CREB &&
         mode != MOTOR_PROTOCOL_WRIST_MODE_MIT)) {
      return false;
    }
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    if (runtime->requested_mode == mode) {
      return true;
    }
    latch_motor_stop(ctx, index);
    runtime->requested_mode = mode;
    runtime->vaxor_mode_change_pending = true;
    runtime->command_valid = false;
    runtime->state = MOTOR_CONTROL_DISABLED;
    return true;
  }
  return false;
}

bool motor_control_set_gains(motor_control_context_t *ctx,
                             const char *joint_name, double kp, double kd) {
  if (ctx == NULL || joint_name == NULL || joint_name[0] == '\0' ||
      memchr(joint_name, '\0', MOTOR_CONTROL_JOINT_NAME_CAPACITY) == NULL ||
      !isfinite(kp) || !isfinite(kd) || kp < 0.0 || kd < 0.0) {
    return false;
  }
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    const motor_control_motor_config_t *motor = &ctx->motors[index];
    if (strcmp(motor->joint_name, joint_name) != 0) {
      continue;
    }
    if ((motor->protocol == MOTOR_CONTROL_VAXOR &&
         (kp > 0.65535 || kd > 655.35)) ||
        (motor->protocol == MOTOR_CONTROL_CREB &&
         (kp > 500.0 || kd > 5.0))) {
      return false;
    }
    ctx->runtime[index].kp = kp;
    ctx->runtime[index].kd = kd;
    return true;
  }
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
      latch_motor_stop(ctx, index);
      runtime->state = MOTOR_CONTROL_DISABLED;
    } else if (age >= 100U) {
      runtime->state = MOTOR_CONTROL_SOFT_STOP;
    } else {
      runtime->state = MOTOR_CONTROL_ACTIVE;
    }
  }

  size_t action_count = 0U;
  if (actions == NULL) {
    return 0U;
  }

  action_count += append_vaxor_actions(ctx, actions, action_capacity);
  action_count += append_creb_actions(
      ctx, now_ms, actions + action_count, action_capacity - action_count);
  action_count += append_creb_sync_actions(
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
  for (size_t index = 0; index < ctx->motor_count; ++index) {
    if (ctx->motors[index].bus_index != bus_index) {
      continue;
    }
    motor_control_motor_runtime_t *runtime = &ctx->runtime[index];
    latch_motor_stop(ctx, index);
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
    latch_motor_stop(ctx, motor_index);
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
