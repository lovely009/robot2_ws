// save_transaction host tests. All device, storage, ID, and error effects are fake ports.

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "save_transaction.h"

struct device_attempt_t {
  uint32_t request_id;
  uint16_t motor_id;
  uint8_t dirty_slots;
  bool accepted;
};

struct reported_error_t {
  save_transaction_error_t reason;
  uint16_t motor_id;
  uint16_t device_error_code;
};

struct fake_ports_t {
  uint32_t next_id;
  device_attempt_t device_attempts[64];
  size_t device_attempt_count;
  uint16_t reject_motor_id;
  uint32_t storage_request_ids[8];
  size_t storage_request_count;
  bool accept_storage_request;
  save_transaction_storage_result_t storage_result;
  uint32_t storage_result_id;
  reported_error_t errors[16];
  size_t error_count;
};

static uint32_t next_request_id(void *context) {
  auto *fake = static_cast<fake_ports_t *>(context);
  ++fake->next_id;
  if (fake->next_id == 0U) {
    ++fake->next_id;
  }
  return fake->next_id;
}

static bool submit_device_save(void *context, uint32_t request_id,
    uint16_t motor_id, uint8_t dirty_slots) {
  auto *fake = static_cast<fake_ports_t *>(context);
  assert(fake->device_attempt_count <
      sizeof(fake->device_attempts) / sizeof(fake->device_attempts[0]));
  const bool accepted = motor_id != fake->reject_motor_id;
  fake->device_attempts[fake->device_attempt_count++] = {
    request_id, motor_id, dirty_slots, accepted};
  if (!accepted) {
    fake->reject_motor_id = UINT16_MAX;
  }
  return accepted;
}

static bool request_storage_save(void *context, uint32_t request_id) {
  auto *fake = static_cast<fake_ports_t *>(context);
  assert(fake->storage_request_count <
      sizeof(fake->storage_request_ids) / sizeof(fake->storage_request_ids[0]));
  fake->storage_request_ids[fake->storage_request_count++] = request_id;
  return fake->accept_storage_request;
}

static save_transaction_storage_result_t take_storage_result(
    void *context, uint32_t *request_id) {
  auto *fake = static_cast<fake_ports_t *>(context);
  const save_transaction_storage_result_t result = fake->storage_result;
  if (request_id != nullptr) {
    *request_id = fake->storage_result_id;
  }
  fake->storage_result = SAVE_TRANSACTION_STORAGE_IDLE;
  fake->storage_result_id = 0U;
  return result;
}

static void report_error(void *context, save_transaction_error_t reason,
    uint16_t motor_id, uint16_t device_error_code) {
  auto *fake = static_cast<fake_ports_t *>(context);
  assert(fake->error_count < sizeof(fake->errors) / sizeof(fake->errors[0]));
  fake->errors[fake->error_count++] = {reason, motor_id, device_error_code};
}

static save_transaction_ports_t make_ports(fake_ports_t *fake) {
  save_transaction_ports_t ports = {};
  ports.context = fake;
  ports.next_request_id = next_request_id;
  ports.submit_device_save = submit_device_save;
  ports.request_storage_save = request_storage_save;
  ports.take_storage_result = take_storage_result;
  ports.report_error = report_error;
  return ports;
}

static void test_dirty_filter_and_four_device_submissions_per_poll() {
  save_transaction_t transaction = {};
  save_transaction_init(&transaction);
  fake_ports_t fake = {};
  fake.next_id = 100U;
  fake.reject_motor_id = UINT16_MAX;
  fake.accept_storage_request = true;
  const save_transaction_ports_t ports = make_ports(&fake);
  const save_transaction_target_t targets[] = {
    {1U, 0U}, {2U, 1U}, {3U, 2U}, {4U, 4U},
    {5U, 8U}, {6U, 16U}, {7U, 32U},
  };

  save_transaction_request(&transaction);
  save_transaction_effects_t effects = save_transaction_poll(
      &transaction, 10U, targets, 7U, &ports);
  assert(effects.status_changed);
  assert(fake.device_attempt_count == SAVE_TRANSACTION_SUBMITS_PER_POLL);
  for (size_t i = 0U; i < SAVE_TRANSACTION_SUBMITS_PER_POLL; ++i) {
    assert(fake.device_attempts[i].motor_id == (uint16_t)(i + 2U));
    assert(fake.device_attempts[i].dirty_slots == (uint8_t)(1U << i));
    assert(fake.device_attempts[i].accepted);
  }

  (void)save_transaction_poll(&transaction, 20U, targets, 7U, &ports);
  assert(fake.device_attempt_count == 6U);
  assert(fake.storage_request_count == 0U);
  assert(save_transaction_in_progress(&transaction));
}

static void test_failed_device_result_drains_outstanding_without_storage() {
  save_transaction_t transaction = {};
  save_transaction_init(&transaction);
  fake_ports_t fake = {};
  fake.next_id = 10U;
  fake.reject_motor_id = UINT16_MAX;
  fake.accept_storage_request = true;
  const save_transaction_ports_t ports = make_ports(&fake);
  const save_transaction_target_t targets[] = {
    {3U, 1U}, {4U, 2U}, {5U, 4U},
  };

  save_transaction_request(&transaction);
  (void)save_transaction_poll(&transaction, 0U, targets, 3U, &ports);
  assert(fake.device_attempt_count == 3U);
  assert(!save_transaction_on_device_result(
      &transaction, 0xDEADU, true, 3U, 0U, &ports));
  assert(save_transaction_on_device_result(
      &transaction, fake.device_attempts[0].request_id, false, 3U, 4U,
      &ports));
  assert(!save_transaction_on_device_result(
      &transaction, fake.device_attempts[0].request_id, true, 3U, 0U,
      &ports));

  save_transaction_effects_t effects =
      save_transaction_poll(&transaction, 1U, targets, 3U, &ports);
  assert(!effects.completed);
  assert(fake.storage_request_count == 0U);
  assert(save_transaction_in_progress(&transaction));

  assert(save_transaction_on_device_result(
      &transaction, fake.device_attempts[1].request_id, true, 4U, 0U,
      &ports));
  assert(save_transaction_on_device_result(
      &transaction, fake.device_attempts[2].request_id, true, 5U, 0U,
      &ports));
  effects = save_transaction_poll(&transaction, 2U, targets, 3U, &ports);
  assert(effects.completed && effects.status_changed);
  assert(!save_transaction_in_progress(&transaction));
  assert(fake.storage_request_count == 0U);
  assert(fake.error_count == 1U);
  assert(fake.errors[0].reason == SAVE_TRANSACTION_ERROR_DEVICE);
  assert(fake.errors[0].motor_id == 3U);
  assert(fake.errors[0].device_error_code == 4U);
}

static void test_storage_waits_for_all_device_success_and_matches_id() {
  save_transaction_t transaction = {};
  save_transaction_init(&transaction);
  fake_ports_t fake = {};
  fake.next_id = 200U;
  fake.reject_motor_id = UINT16_MAX;
  fake.accept_storage_request = true;
  const save_transaction_ports_t ports = make_ports(&fake);
  const save_transaction_target_t targets[] = {{9U, 1U}, {10U, 2U}};

  save_transaction_request(&transaction);
  (void)save_transaction_poll(&transaction, 100U, targets, 2U, &ports);
  assert(fake.device_attempt_count == 2U);
  assert(save_transaction_on_device_result(
      &transaction, fake.device_attempts[0].request_id, true, 9U, 0U,
      &ports));
  (void)save_transaction_poll(&transaction, 110U, targets, 2U, &ports);
  assert(fake.storage_request_count == 0U);

  assert(save_transaction_on_device_result(
      &transaction, fake.device_attempts[1].request_id, true, 10U, 0U,
      &ports));
  save_transaction_effects_t effects =
      save_transaction_poll(&transaction, 120U, targets, 2U, &ports);
  assert(!effects.completed);
  assert(fake.storage_request_count == 1U);
  assert(save_transaction_in_progress(&transaction));

  fake.storage_result = SAVE_TRANSACTION_STORAGE_OK;
  fake.storage_result_id = fake.storage_request_ids[0] + 1U;
  effects = save_transaction_poll(&transaction, 130U, targets, 2U, &ports);
  assert(!effects.completed);
  assert(save_transaction_in_progress(&transaction));

  fake.storage_result = SAVE_TRANSACTION_STORAGE_OK;
  fake.storage_result_id = fake.storage_request_ids[0];
  effects = save_transaction_poll(&transaction, 140U, targets, 2U, &ports);
  assert(effects.completed && effects.status_changed);
  assert(!save_transaction_in_progress(&transaction));
  assert(fake.error_count == 0U);
}

static void test_device_and_storage_timeouts_are_wrap_safe() {
  const save_transaction_target_t dirty_target[] = {{1U, 1U}};
  save_transaction_t device_transaction = {};
  save_transaction_init(&device_transaction);
  fake_ports_t device_fake = {};
  device_fake.next_id = 1U;
  device_fake.reject_motor_id = UINT16_MAX;
  device_fake.accept_storage_request = true;
  const save_transaction_ports_t device_ports = make_ports(&device_fake);
  const uint32_t device_start = UINT32_MAX - 1000U;
  save_transaction_request(&device_transaction);
  (void)save_transaction_poll(&device_transaction, device_start,
      dirty_target, 1U, &device_ports);
  save_transaction_effects_t effects = save_transaction_poll(
      &device_transaction, device_start + SAVE_TRANSACTION_TIMEOUT_MS - 1U,
      dirty_target, 1U, &device_ports);
  assert(!effects.completed && save_transaction_in_progress(&device_transaction));
  effects = save_transaction_poll(
      &device_transaction, device_start + SAVE_TRANSACTION_TIMEOUT_MS,
      dirty_target, 1U, &device_ports);
  assert(effects.completed && !save_transaction_in_progress(&device_transaction));
  assert(device_fake.error_count == 1U);
  assert(device_fake.errors[0].reason == SAVE_TRANSACTION_ERROR_TIMEOUT);

  save_transaction_t storage_transaction = {};
  save_transaction_init(&storage_transaction);
  fake_ports_t storage_fake = {};
  storage_fake.next_id = 50U;
  storage_fake.reject_motor_id = UINT16_MAX;
  storage_fake.accept_storage_request = true;
  const save_transaction_ports_t storage_ports = make_ports(&storage_fake);
  save_transaction_request(&storage_transaction);
  (void)save_transaction_poll(&storage_transaction, 100U, nullptr, 0U,
      &storage_ports);
  effects = save_transaction_poll(
      &storage_transaction, 100U + SAVE_TRANSACTION_TIMEOUT_MS - 1U,
      nullptr, 0U, &storage_ports);
  assert(!effects.completed && save_transaction_in_progress(&storage_transaction));
  effects = save_transaction_poll(
      &storage_transaction, 100U + SAVE_TRANSACTION_TIMEOUT_MS,
      nullptr, 0U, &storage_ports);
  assert(effects.completed && !save_transaction_in_progress(&storage_transaction));
  assert(storage_fake.error_count == 1U);
  assert(storage_fake.errors[0].reason == SAVE_TRANSACTION_ERROR_TIMEOUT);
}

static void test_duplicate_requests_coalesce_and_disconnect_clears_state() {
  save_transaction_t transaction = {};
  save_transaction_init(&transaction);
  fake_ports_t fake = {};
  fake.next_id = 80U;
  fake.reject_motor_id = UINT16_MAX;
  fake.accept_storage_request = true;
  const save_transaction_ports_t ports = make_ports(&fake);

  save_transaction_request(&transaction);
  (void)save_transaction_poll(&transaction, 0U, nullptr, 0U, &ports);
  assert(fake.storage_request_count == 1U);
  const uint32_t first_storage_id = fake.storage_request_ids[0];
  save_transaction_request(&transaction);
  save_transaction_request(&transaction);
  fake.storage_result = SAVE_TRANSACTION_STORAGE_OK;
  fake.storage_result_id = first_storage_id;
  save_transaction_effects_t effects =
      save_transaction_poll(&transaction, 1U, nullptr, 0U, &ports);
  assert(effects.completed);
  assert(save_transaction_in_progress(&transaction));
  (void)save_transaction_poll(&transaction, 2U, nullptr, 0U, &ports);
  assert(fake.storage_request_count == 2U);

  save_transaction_disconnect(&transaction);
  assert(!save_transaction_in_progress(&transaction));
  assert(!save_transaction_on_device_result(
      &transaction, 0x123U, true, 1U, 0U, &ports));
}

int main() {
  test_dirty_filter_and_four_device_submissions_per_poll();
  test_failed_device_result_drains_outstanding_without_storage();
  test_storage_waits_for_all_device_success_and_matches_id();
  test_device_and_storage_timeouts_are_wrap_safe();
  test_duplicate_requests_coalesce_and_disconnect_clears_state();
  std::printf("test_save_transaction: all passed\n");
  return 0;
}
