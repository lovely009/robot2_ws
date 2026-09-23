// Device SAVE requests must settle before storage can persist the parameter view.

#include "save_transaction.h"

#include <string.h>

#define kMotorIdNone UINT16_MAX

static bool ports_ready(const save_transaction_ports_t *ports) {
  return ports != NULL && ports->next_request_id != NULL &&
         ports->submit_device_save != NULL &&
         ports->request_storage_save != NULL &&
         ports->take_storage_result != NULL;
}

static void report_error(const save_transaction_ports_t *ports,
    save_transaction_error_t reason, uint16_t motor_id,
    uint16_t device_error_code) {
  if (ports != NULL && ports->report_error != NULL) {
    ports->report_error(ports->context, reason, motor_id, device_error_code);
  }
}

static bool track_request(save_transaction_t *transaction,
    uint32_t request_id) {
  if (request_id == 0U || transaction->pending_count >=
      SAVE_TRANSACTION_MAX_TARGETS) {
    return false;
  }
  for (size_t i = 0U; i < SAVE_TRANSACTION_MAX_TARGETS; ++i) {
    if (transaction->outstanding_request_ids[i] == request_id) {
      return false;
    }
  }
  for (size_t i = 0U; i < SAVE_TRANSACTION_MAX_TARGETS; ++i) {
    if (transaction->outstanding_request_ids[i] == 0U) {
      transaction->outstanding_request_ids[i] = request_id;
      ++transaction->pending_count;
      return true;
    }
  }
  return false;
}

static bool take_request(save_transaction_t *transaction,
    uint32_t request_id) {
  if (request_id == 0U) {
    return false;
  }
  for (size_t i = 0U; i < SAVE_TRANSACTION_MAX_TARGETS; ++i) {
    if (transaction->outstanding_request_ids[i] == request_id) {
      transaction->outstanding_request_ids[i] = 0U;
      if (transaction->pending_count > 0U) {
        --transaction->pending_count;
      }
      return true;
    }
  }
  return false;
}

static void fail_device_phase(save_transaction_t *transaction) {
  transaction->device_failed = true;
  transaction->cursor = transaction->target_count;
}

static void finish(save_transaction_t *transaction,
    save_transaction_effects_t *effects) {
  const bool request_pending = transaction->request_pending;
  transaction->phase = SAVE_TRANSACTION_PHASE_IDLE;
  transaction->device_failed = false;
  transaction->cursor = 0U;
  transaction->target_count = 0U;
  transaction->pending_count = 0U;
  transaction->phase_started_ms = 0U;
  transaction->storage_request_id = 0U;
  memset(transaction->targets, 0, sizeof(transaction->targets));
  memset(transaction->outstanding_request_ids, 0,
      sizeof(transaction->outstanding_request_ids));
  transaction->request_pending = request_pending;
  effects->status_changed = true;
  effects->completed = true;
}

static void start_storage_phase(save_transaction_t *transaction,
    uint32_t now_ms,
    const save_transaction_ports_t *ports,
    save_transaction_effects_t *effects) {
  transaction->phase = SAVE_TRANSACTION_PHASE_STORAGE;
  transaction->phase_started_ms = now_ms;
  transaction->storage_request_id = ports->next_request_id(ports->context);
  if (transaction->storage_request_id == 0U ||
      !ports->request_storage_save(ports->context,
      transaction->storage_request_id)) {
    report_error(ports, SAVE_TRANSACTION_ERROR_STORAGE, kMotorIdNone, 0U);
    finish(transaction, effects);
  }
}

void save_transaction_init(save_transaction_t *transaction) {
  if (transaction != NULL) {
    memset(transaction, 0, sizeof(*transaction));
  }
}

void save_transaction_request(save_transaction_t *transaction) {
  if (transaction != NULL) {
    transaction->request_pending = true;
  }
}

void save_transaction_disconnect(save_transaction_t *transaction) {
  save_transaction_init(transaction);
}

bool save_transaction_in_progress(const save_transaction_t *transaction) {
  return transaction != NULL &&
         (transaction->request_pending ||
         transaction->phase != SAVE_TRANSACTION_PHASE_IDLE);
}

bool save_transaction_on_device_result(
    save_transaction_t *transaction, uint32_t request_id, bool success,
    uint16_t motor_id, uint16_t device_error_code,
    const save_transaction_ports_t *ports) {
  if (transaction == NULL ||
      transaction->phase != SAVE_TRANSACTION_PHASE_DEVICE ||
      !take_request(transaction, request_id)) {
    return false;
  }
  if (!success) {
    fail_device_phase(transaction);
    report_error(ports, SAVE_TRANSACTION_ERROR_DEVICE, motor_id,
        device_error_code);
  }
  return true;
}

save_transaction_effects_t save_transaction_poll(
    save_transaction_t *transaction, uint32_t now_ms,
    const save_transaction_target_t *targets, size_t target_count,
    const save_transaction_ports_t *ports) {
  save_transaction_effects_t effects = {false, false};
  if (transaction == NULL || !ports_ready(ports)) {
    return effects;
  }

  if (transaction->phase == SAVE_TRANSACTION_PHASE_IDLE &&
      transaction->request_pending) {
    transaction->request_pending = false;
    transaction->phase = SAVE_TRANSACTION_PHASE_DEVICE;
    transaction->phase_started_ms = now_ms;
    transaction->device_failed = false;
    transaction->cursor = 0U;
    transaction->target_count = 0U;
    transaction->pending_count = 0U;
    transaction->storage_request_id = 0U;
    memset(transaction->targets, 0, sizeof(transaction->targets));
    memset(transaction->outstanding_request_ids, 0,
        sizeof(transaction->outstanding_request_ids));
    effects.status_changed = true;

    if (target_count > SAVE_TRANSACTION_MAX_TARGETS ||
        (target_count > 0U && targets == NULL)) {
      report_error(ports, SAVE_TRANSACTION_ERROR_SUBMIT, kMotorIdNone, 0U);
      finish(transaction, &effects);
      return effects;
    }
    for (size_t i = 0U; i < target_count; ++i) {
      if (targets[i].dirty_slots != 0U) {
        transaction->targets[transaction->target_count++] = targets[i];
      }
    }
  }

  if (transaction->phase == SAVE_TRANSACTION_PHASE_DEVICE) {
    if ((uint32_t)(now_ms - transaction->phase_started_ms) >=
        SAVE_TRANSACTION_TIMEOUT_MS) {
      report_error(ports, SAVE_TRANSACTION_ERROR_TIMEOUT, kMotorIdNone, 0U);
      finish(transaction, &effects);
      return effects;
    }

    uint32_t submitted = 0U;
    while (transaction->cursor < transaction->target_count &&
        submitted < SAVE_TRANSACTION_SUBMITS_PER_POLL) {
      const save_transaction_target_t target =
          transaction->targets[transaction->cursor++];
      const uint32_t request_id = ports->next_request_id(ports->context);
      if (!track_request(transaction, request_id)) {
        fail_device_phase(transaction);
        report_error(ports, SAVE_TRANSACTION_ERROR_SUBMIT, target.motor_id,
            0U);
        break;
      }
      if (!ports->submit_device_save(ports->context, request_id,
          target.motor_id, target.dirty_slots)) {
        (void)take_request(transaction, request_id);
        fail_device_phase(transaction);
        report_error(ports, SAVE_TRANSACTION_ERROR_SUBMIT, target.motor_id,
            0U);
        break;
      }
      ++submitted;
    }

    if (transaction->cursor >= transaction->target_count &&
        transaction->pending_count == 0U) {
      if (transaction->device_failed) {
        finish(transaction, &effects);
        return effects;
      }
      start_storage_phase(transaction, now_ms, ports, &effects);
    }
    return effects;
  }

  if (transaction->phase != SAVE_TRANSACTION_PHASE_STORAGE) {
    return effects;
  }

  uint32_t result_request_id = 0U;
  const save_transaction_storage_result_t result =
      ports->take_storage_result(ports->context, &result_request_id);
  if (result != SAVE_TRANSACTION_STORAGE_IDLE &&
      result_request_id != transaction->storage_request_id) {
    return effects;
  }
  if (result == SAVE_TRANSACTION_STORAGE_IDLE) {
    if ((uint32_t)(now_ms - transaction->phase_started_ms) <
        SAVE_TRANSACTION_TIMEOUT_MS) {
      return effects;
    }
    report_error(ports, SAVE_TRANSACTION_ERROR_TIMEOUT, kMotorIdNone, 0U);
  } else if (result != SAVE_TRANSACTION_STORAGE_OK) {
    report_error(ports, SAVE_TRANSACTION_ERROR_STORAGE, kMotorIdNone, 0U);
  }
  finish(transaction, &effects);
  return effects;
}
