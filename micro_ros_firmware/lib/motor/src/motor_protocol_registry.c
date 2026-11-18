// 运行时绑定与帧辅助逻辑共用的协议适配器注册表。

#include "motor_protocol_adapter.h"

bool motor_protocol_adapter_allows_motion(
    const motor_protocol_adapter_t *adapter, uint8_t raw_present, uint32_t t_max_raw) {
  return adapter != &motor_protocol_cerb_adapter ||
      ((raw_present & (1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX)) && t_max_raw > 0U);
}

bool motor_protocol_adapter_allows_driver_config(
    const motor_protocol_adapter_t *adapter) {
  return adapter == &motor_protocol_cerb_adapter;
}

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
