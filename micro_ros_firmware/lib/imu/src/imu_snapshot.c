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

void imu_snapshot_publish_timed(imu_snapshot_t *snapshot, uint32_t tick_ms,
    uint64_t epoch_ns, bool clock_valid, bool temperature_new) {
  if (snapshot == NULL) return;
  const uint8_t next = snapshot->active_index ^ 1U;
  sensor_sample_metadata_t metadata = {snapshot->sequence + 1U, tick_ms,
      clock_valid ? epoch_ns : 0U, clock_valid};
  snapshot->metadata[next] = metadata;
  if (temperature_new) {
    const uint32_t sequence = snapshot->temperature_metadata.sequence + 1U;
    snapshot->temperature_metadata = metadata;
    snapshot->temperature_metadata.sequence = sequence;
    snapshot->temperature = snapshot->buffers[next].temp;
    snapshot->temperature_valid = true;
  }
  imu_snapshot_publish(snapshot);
}

bool imu_snapshot_copy_sample(const imu_snapshot_t *snapshot, imu_data_t *out,
    sensor_sample_metadata_t *metadata) {
  if (!imu_snapshot_copy(snapshot, out, NULL)) return false;
  if (metadata != NULL) *metadata = snapshot->metadata[snapshot->active_index];
  return true;
}

bool imu_snapshot_copy_temperature(const imu_snapshot_t *snapshot, float *out,
    sensor_sample_metadata_t *metadata) {
  if (snapshot == NULL || out == NULL || !snapshot->temperature_valid) return false;
  *out = snapshot->temperature;
  if (metadata != NULL) *metadata = snapshot->temperature_metadata;
  return true;
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
