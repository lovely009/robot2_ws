// imu_snapshot host 单测（cassert 风格）：双缓冲快照的发布/拷贝契约——
// 首次发布前 copy 失败；写 inactive 缓冲不干扰读者（仍读到旧完整样本）；
// publish 后序号递增，读者原子切换到新样本。

#include "imu_snapshot.h"

#include <cassert>
#include <cstdint>

static imu_data_t make_sample(float sentinel) {
  imu_data_t sample{};
  for (int index = 0; index < 3; ++index) {
    sample.gyro[index] = sentinel + static_cast<float>(index + 1);
    sample.accel[index] = sentinel + static_cast<float>(index + 11);
    sample.angle[index] = sentinel + static_cast<float>(index + 21);
  }
  sample.temp = sentinel + 31.0f;
  for (int index = 0; index < 4; ++index) {
    sample.quat[index] = sentinel + static_cast<float>(index + 41);
  }
  return sample;
}

static void assert_sample_equals(const imu_data_t &actual,
    const imu_data_t &expected) {
  for (int index = 0; index < 3; ++index) {
    assert(actual.gyro[index] == expected.gyro[index]);
    assert(actual.accel[index] == expected.accel[index]);
    assert(actual.angle[index] == expected.angle[index]);
  }
  assert(actual.temp == expected.temp);
  for (int index = 0; index < 4; ++index) {
    assert(actual.quat[index] == expected.quat[index]);
  }
}

static void test_consumer_gets_complete_published_buffer() {
  imu_snapshot_t snapshot{};
  const imu_data_t sample_a = make_sample(100.0f);
  const imu_data_t sample_b = make_sample(200.0f);
  imu_data_t copied{};
  uint32_t sequence = 99U;

  imu_snapshot_init(&snapshot);
  assert(!imu_snapshot_copy(&snapshot, &copied, &sequence));

  *imu_snapshot_begin_write(&snapshot) = sample_a;
  imu_snapshot_publish(&snapshot);
  assert(imu_snapshot_copy(&snapshot, &copied, &sequence));
  assert(sequence == 1U);
  assert_sample_equals(copied, sample_a);

  imu_data_t *inactive = imu_snapshot_begin_write(&snapshot);
  inactive->gyro[0] = sample_b.gyro[0];
  assert(imu_snapshot_copy(&snapshot, &copied, &sequence));
  assert(sequence == 1U);
  assert_sample_equals(copied, sample_a);

  *inactive = sample_b;
  imu_snapshot_publish(&snapshot);
  assert(imu_snapshot_copy(&snapshot, &copied, &sequence));
  assert(sequence == 2U);
  assert_sample_equals(copied, sample_b);
}

int main() {
  test_consumer_gets_complete_published_buffer();
  return 0;
}
