#include "imu_snapshot.h"

#include <string.h>

void imu_snapshot_init(imu_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  memset(snapshot, 0, sizeof(*snapshot));
}

imu_data_t *imu_snapshot_begin_write(imu_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return NULL;
  }
  return &snapshot->buffers[snapshot->active_index ^ 1U];
}

void imu_snapshot_publish(imu_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  snapshot->active_index ^= 1U;
  ++snapshot->sequence;
  snapshot->valid = true;
}

bool imu_snapshot_copy(const imu_snapshot_t *snapshot, imu_data_t *out,
                       uint32_t *sequence) {
  if (snapshot == NULL || out == NULL || !snapshot->valid) {
    return false;
  }
  *out = snapshot->buffers[snapshot->active_index];
  if (sequence != NULL) {
    *sequence = snapshot->sequence;
  }
  return true;
}
