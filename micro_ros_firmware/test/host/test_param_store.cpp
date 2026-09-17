// param_store 纯库：W25 blob（header+present 位图+定长记录+CRC32）与
// ROS 参数名模型（m<id>.<field> 的生成/解析/值域校验）。
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include "param_name.h"
#include "param_store_blob.h"

namespace {

void test_param_name_build_parse_roundtrip() {
  char buf[PARAM_NAME_MAX_LEN];
  for (uint16_t id = 1U; id <= 127U; ++id) {
    for (int f = 0; f < PARAM_FIELD_COUNT; ++f) {
      auto field = static_cast<param_field_t>(f);
      assert(param_name_build_motor(buf, sizeof(buf), id, field));
      uint16_t got_id = 0;
      param_field_t got_field = PARAM_FIELD_KP;
      assert(param_name_parse(buf, &got_id, &got_field) == PARAM_NAME_MOTOR);
      assert(got_id == id && got_field == field);
    }
  }
  // 具体名字锁死（参数契约）。
  param_name_build_motor(buf, sizeof(buf), 7, PARAM_FIELD_TORQUE_LIMIT);
  assert(strcmp(buf, "m7.torque_limit") == 0);
  param_name_build_motor(buf, sizeof(buf), 127, PARAM_FIELD_ENABLE);
  assert(strcmp(buf, "m127.enable") == 0);

  uint16_t id = 0;
  param_field_t field = PARAM_FIELD_KP;
  assert(param_name_parse("system.save", &id, &field) == PARAM_NAME_SYSTEM);
  assert(param_name_parse("m7.kp", &id, &field) == PARAM_NAME_MOTOR);
  assert(id == 7 && field == PARAM_FIELD_KP);
  assert(param_name_parse("garbage", &id, &field) == PARAM_NAME_UNKNOWN);
  assert(param_name_parse("m0.kp", &id, &field) == PARAM_NAME_UNKNOWN);
  assert(param_name_parse("m128.kp", &id, &field) == PARAM_NAME_UNKNOWN);
  assert(param_name_parse("m7.bogus", &id, &field) == PARAM_NAME_UNKNOWN);
  assert(param_name_parse("m.kp", &id, &field) == PARAM_NAME_UNKNOWN);
}

void test_param_name_validation() {
  // kp/kd 非负；位置 min<max；速度/力矩为正有限值。
  float clamped = 0.0f;
  assert(param_value_ok(PARAM_FIELD_KP, 0.0f, 500.0f, &clamped) &&
         clamped == 0.0f);
  assert(param_value_ok(PARAM_FIELD_KP, 600.0f, 500.0f, &clamped) &&
         clamped == 500.0f); // 超上限钳到上限（受理）
  assert(!param_value_ok(PARAM_FIELD_KP, -1.0f, 500.0f, &clamped));
  assert(!param_value_ok(PARAM_FIELD_KP, NAN, 500.0f, &clamped));
  assert(!param_value_ok(PARAM_FIELD_KP, INFINITY, 500.0f, &clamped));
  assert(param_value_ok(PARAM_FIELD_VEL_LIMIT, 3.0f, 3.14f, &clamped) &&
         clamped == 3.0f);
  assert(!param_value_ok(PARAM_FIELD_TORQUE_LIMIT, 0.0f, 45.0f, &clamped));
  // 成对校验：min<max。
  assert(param_position_limits_ok(-1.0f, 1.0f));
  assert(!param_position_limits_ok(1.0f, 1.0f));
  assert(!param_position_limits_ok(2.0f, 1.0f));
}

static const float kEps = 1e-6f;

void test_blob_roundtrip_and_crc() {
  static uint8_t storage[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob = param_store_blob_reset(storage, sizeof(storage));
  assert(blob != nullptr);
  assert(param_store_blob_validate(blob));
  assert(blob->record_count == PARAM_STORE_RECORD_COUNT);

  // 空 blob：字段读不到；不与 0 值混淆（present 位）。
  float v = 123.0f;
  assert(!param_store_blob_get(blob, 7, PARAM_FIELD_KP, &v));
  param_store_blob_set(blob, 7, PARAM_FIELD_KP, 30.0f);
  param_store_blob_set(blob, 7, PARAM_FIELD_POS_MIN, -1.0f);
  param_store_blob_set(blob, 9, PARAM_FIELD_VEL_LIMIT, 2.5f);
  assert(param_store_blob_get(blob, 7, PARAM_FIELD_KP, &v) &&
         std::abs(v - 30.0f) < kEps);
  assert(param_store_blob_get(blob, 7, PARAM_FIELD_POS_MIN, &v) &&
         std::abs(v + 1.0f) < kEps);
  assert(!param_store_blob_get(blob, 7, PARAM_FIELD_KD, &v));
  assert(!param_store_blob_get(blob, 8, PARAM_FIELD_KP, &v));
  assert(param_store_blob_get(blob, 9, PARAM_FIELD_VEL_LIMIT, &v) &&
         std::abs(v - 2.5f) < kEps);

  // finalize 后 CRC 有效；改一个字节即失效。
  param_store_blob_finalize(blob, 1);
  assert(param_store_blob_validate(blob));
  assert(blob->seq == 1U);
  blob->records[0].kp = 31.0f;
  assert(!param_store_blob_validate(blob));
  blob->records[0].kp = 30.0f;
  assert(param_store_blob_validate(blob));
}

void test_blob_rejects_bad_layout() {
  static uint8_t storage[PARAM_STORE_BLOB_SIZE];
  assert(param_store_blob_reset(storage, sizeof(storage)));
  param_store_blob_t *blob = reinterpret_cast<param_store_blob_t *>(storage);
  // 短缓冲拒绝。
  assert(param_store_blob_reset(storage, PARAM_STORE_BLOB_SIZE - 1) == nullptr);
  // magic/version/record_count 不符即无效。
  param_store_blob_finalize(blob, 0);
  uint32_t saved_magic = blob->magic;
  blob->magic = 0xDEADBEEFU;
  assert(!param_store_blob_validate(blob));
  blob->magic = saved_magic;
  assert(param_store_blob_validate(blob));
  blob->version = 999U;
  assert(!param_store_blob_validate(blob));
}

void test_blob_record_capacity() {
  static uint8_t storage[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob = param_store_blob_reset(storage, sizeof(storage));
  // 64 个不同 motor_id 都能放（稀疏槽）；第 65 个拒绝。
  for (uint16_t id = 1U; id <= 64U; ++id) {
    assert(param_store_blob_set(blob, id, PARAM_FIELD_KP, 1.0f));
  }
  assert(!param_store_blob_set(blob, 65U, PARAM_FIELD_KP, 1.0f));
  // 同一电机的 6 个持久化字段都在同一条记录（enable 不持久化）。
  float v = 0.0f;
  for (int f = PARAM_FIELD_KP; f < PARAM_FIELD_COUNT; ++f) {
    assert(param_store_blob_set(blob, 1, static_cast<param_field_t>(f), 1.0f));
  }
  for (int f = PARAM_FIELD_KP; f < PARAM_FIELD_COUNT; ++f) {
    assert(param_store_blob_get(blob, 1, static_cast<param_field_t>(f), &v));
  }
  assert(!param_store_blob_set(blob, 1, PARAM_FIELD_ENABLE, true ? 1.0f : 0.0f));
}

} // namespace

int main() {
  test_param_name_build_parse_roundtrip();
  test_param_name_validation();
  test_blob_roundtrip_and_crc();
  test_blob_rejects_bad_layout();
  test_blob_record_capacity();
  printf("test_param_store: all passed\n");
  return 0;
}
