#include <cassert>
#include "storage_projection.h"

int main() {
  param_store_blob_t blob{};
  assert(param_store_blob_reset(&blob, sizeof(blob)));
  assert(param_store_blob_set(&blob, 7, PARAM_FIELD_KP, 30.0f));
  assert(param_store_blob_set(&blob, 7, PARAM_FIELD_KD, 1.0f));
  assert(param_store_blob_set(&blob, 7, PARAM_FIELD_POS_MIN, -1.0f));
  assert(param_store_blob_set(&blob, 7, PARAM_FIELD_POS_MAX, 1.0f));
  assert(param_store_blob_set_device_raw(&blob, 7, PARAM_DEVICE_SLOT_DIRECTION, 1U));
  param_store_blob_finalize(&blob, 9);
  assert(param_store_blob_validate(&blob));
  motor_control_stored_params_t out[2]{};
  assert(storage_projection_blob_to_boot(&blob, out, 2) == 1);
  assert(out[0].motor_id == 7 && out[0].has_kp_kd && out[0].kp == 30.0f);
  assert(out[0].has_position_limits && out[0].pos_min_rad == -1.0f);
  assert(out[0].device_raw_present & (1U << PARAM_DEVICE_SLOT_DIRECTION));
  assert(out[0].device_raw[PARAM_DEVICE_SLOT_DIRECTION] == 1U);
  assert(storage_projection_blob_to_boot(nullptr, out, 2) == 0);
  assert(storage_projection_blob_to_boot(&blob, out, 0) == 0);
}
