// Host-testable coordinator for the device-flash then W25 SAVE transaction.

#ifndef SAVE_TRANSACTION_H
#define SAVE_TRANSACTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAVE_TRANSACTION_MAX_TARGETS 64U
#define SAVE_TRANSACTION_SUBMITS_PER_POLL 4U
#define SAVE_TRANSACTION_TIMEOUT_MS 5000U

typedef struct {
  uint16_t motor_id;
  uint8_t dirty_slots;
} save_transaction_target_t;

typedef enum {
  SAVE_TRANSACTION_STORAGE_IDLE = 0,
  SAVE_TRANSACTION_STORAGE_OK = 1,
  SAVE_TRANSACTION_STORAGE_FAIL = 2,
} save_transaction_storage_result_t;

typedef enum {
  SAVE_TRANSACTION_ERROR_SUBMIT = 0,
  SAVE_TRANSACTION_ERROR_DEVICE = 1,
  SAVE_TRANSACTION_ERROR_TIMEOUT = 2,
  SAVE_TRANSACTION_ERROR_STORAGE = 3,
} save_transaction_error_t;

typedef enum {
  SAVE_TRANSACTION_PHASE_IDLE = 0,
  SAVE_TRANSACTION_PHASE_DEVICE = 1,
  SAVE_TRANSACTION_PHASE_STORAGE = 2,
} save_transaction_phase_t;

typedef struct {
  void *context;
  uint32_t (*next_request_id)(void *context);
  bool (*submit_device_save)(void *context, uint32_t request_id,
      uint16_t motor_id, uint8_t dirty_slots);
  bool (*request_storage_save)(void *context, uint32_t request_id);
  save_transaction_storage_result_t (*take_storage_result)(
    void *context, uint32_t *request_id);
  void (*report_error)(void *context, save_transaction_error_t reason,
      uint16_t motor_id, uint16_t device_error_code);
} save_transaction_ports_t;

typedef struct {
  bool status_changed;
  bool completed;
} save_transaction_effects_t;

typedef struct {
  bool request_pending;
  bool device_failed;
  save_transaction_phase_t phase;
  uint16_t cursor;
  uint16_t target_count;
  uint16_t pending_count;
  uint32_t phase_started_ms;
  uint32_t storage_request_id;
  save_transaction_target_t targets[SAVE_TRANSACTION_MAX_TARGETS];
  uint32_t outstanding_request_ids[SAVE_TRANSACTION_MAX_TARGETS];
} save_transaction_t;

void save_transaction_init(save_transaction_t *transaction);
void save_transaction_request(save_transaction_t *transaction);
void save_transaction_disconnect(save_transaction_t *transaction);
bool save_transaction_in_progress(const save_transaction_t *transaction);

// Returns true only when request_id belongs to this transaction. A failed
// device result stops new submissions and drains already outstanding requests.
bool save_transaction_on_device_result(
  save_transaction_t *transaction, uint32_t request_id, bool success,
  uint16_t motor_id, uint16_t device_error_code,
  const save_transaction_ports_t *ports);

// Starts queued work from the current target snapshot, sends at most four
// device requests, and starts W25 only after all matching device results pass.
save_transaction_effects_t save_transaction_poll(
  save_transaction_t *transaction, uint32_t now_ms,
  const save_transaction_target_t *targets, size_t target_count,
  const save_transaction_ports_t *ports);

#ifdef __cplusplus
}
#endif

#endif // SAVE_TRANSACTION_H
