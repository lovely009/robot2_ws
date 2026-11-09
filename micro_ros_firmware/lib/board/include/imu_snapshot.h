#ifndef IMU_SNAPSHOT_H
#define IMU_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t sequence;
  uint32_t tick_ms;
  uint64_t epoch_ns;
  bool clock_valid;
} sensor_sample_metadata_t;

// 双缓冲 IMU 快照的纯数据操作。调用方负责在 publish/copy 时提供同步边界。
typedef struct {
  imu_data_t buffers[2];
  uint8_t active_index;
  uint32_t sequence;
  bool valid;
  sensor_sample_metadata_t metadata[2];
  sensor_sample_metadata_t temperature_metadata;
  float temperature;
  bool temperature_valid;
} imu_snapshot_t;

void imu_snapshot_init(imu_snapshot_t *snapshot);
imu_data_t *imu_snapshot_begin_write(imu_snapshot_t *snapshot);
void imu_snapshot_publish(imu_snapshot_t *snapshot);
void imu_snapshot_publish_timed(imu_snapshot_t *snapshot, uint32_t tick_ms,
    uint64_t epoch_ns, bool clock_valid, bool temperature_new);
bool imu_snapshot_copy_sample(const imu_snapshot_t *snapshot, imu_data_t *out,
    sensor_sample_metadata_t *metadata);
bool imu_snapshot_copy_temperature(const imu_snapshot_t *snapshot, float *out,
    sensor_sample_metadata_t *metadata);
bool imu_snapshot_copy(const imu_snapshot_t *snapshot, imu_data_t *out,
    uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif  // IMU_SNAPSHOT_H
