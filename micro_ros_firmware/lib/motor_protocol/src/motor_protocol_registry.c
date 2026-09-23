// One protocol-owned adapter list shared by runtime binding and frame helpers.

#include "motor_protocol_adapter.h"

static const motor_protocol_adapter_t *const kAdapters[] = {
  &motor_protocol_vaxor_adapter,
  &motor_protocol_cerb_adapter,
};

const motor_protocol_adapter_t *const *motor_protocol_adapter_registry(
    size_t *out_count) {
  if (out_count == NULL) {
    return NULL;
  }
  *out_count = sizeof(kAdapters) / sizeof(kAdapters[0]);
  return kAdapters;
}
