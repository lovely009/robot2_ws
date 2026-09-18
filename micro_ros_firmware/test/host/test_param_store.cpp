// param_store 纯库：W25 blob（header+present 位图+定长记录+CRC32）与
// ROS 参数名模型（m<id>.<field> 的生成/解析/值域校验）。
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include "param_name.h"
#include "param_store_blob.h"
#include "param_units.h"

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

// v3 布局尺寸契约：改动尺寸必须先动这里的断言，防止静默漂移。
void test_blob_v3_layout_size() {
  static_assert(sizeof(param_store_record_t) == 120U, "v3 记录必须 120B");
  static_assert(PARAM_STORE_RECORD_SIZE == 120U, "v3 记录尺寸常量");
  static_assert(PARAM_STORE_BLOB_SIZE == 7712U, "v3 blob = 32 + 64*120");
  static_assert(PARAM_STORE_VERSION == 3U, "v3 版本号");
  // 槽区：每槽 2 个 4KB 扇区，blob 必须装得下。
  assert(PARAM_STORE_BLOB_SIZE <= 2U * 4096U);
  // 字段偏移契约（迁移与页读回都依赖它）。
  param_store_record_t rec{};
  assert(reinterpret_cast<uint8_t *>(&rec.present) == reinterpret_cast<uint8_t *>(&rec));
  assert(reinterpret_cast<uint8_t *>(&rec.device_raw) - reinterpret_cast<uint8_t *>(&rec) == 32);
  assert(reinterpret_cast<uint8_t *>(&rec.model_name) - reinterpret_cast<uint8_t *>(&rec) == 64);
  assert(reinterpret_cast<uint8_t *>(&rec.identity_hash) - reinterpret_cast<uint8_t *>(&rec) == 80);
  assert(reinterpret_cast<uint8_t *>(&rec.mit_pos_limit_rad) - reinterpret_cast<uint8_t *>(&rec) == 84);
  assert(reinterpret_cast<uint8_t *>(&rec.profile_version) - reinterpret_cast<uint8_t *>(&rec) == 96);
  assert(sizeof(rec.device_raw) == 32U);
  assert(sizeof(rec.reserved) == 23U);
}

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

// 设备端原值（v2 新增）：与 SI 字段同记录、位图独立、互不干扰。
void test_blob_device_raw_roundtrip() {
  static uint8_t storage[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob = param_store_blob_reset(storage, sizeof(storage));
  // 先写原值、后写字段：必须落在同一条记录（空槽判定看两位图）。
  assert(param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_TORQUE_LIMIT,
                                         13300U));
  assert(param_store_blob_set(blob, 7, PARAM_FIELD_TORQUE_LIMIT, 115.5f));
  assert(param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_VEL_LIMIT,
                                         3030U));
  uint32_t raw = 0U;
  float v = 0.0f;
  assert(param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_TORQUE_LIMIT,
                                         &raw) && raw == 13300U);
  assert(param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_VEL_LIMIT,
                                         &raw) && raw == 3030U);
  assert(!param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_POS_MIN,
                                          &raw));
  assert(param_store_blob_device_raw_present(blob, 7) ==
         (uint8_t)((1U << PARAM_DEVICE_SLOT_TORQUE_LIMIT) |
                   (1U << PARAM_DEVICE_SLOT_VEL_LIMIT)));
  assert(param_store_blob_get(blob, 7, PARAM_FIELD_TORQUE_LIMIT, &v) &&
         std::fabs(v - 115.5f) < 1e-4f);
  // 只占原值位的记录也必须能被查到（present 无位）。
  assert(param_store_blob_device_raw_present(blob, 7) != 0U);
  // 未记录原值的电机：位图为 0。
  assert(param_store_blob_set(blob, 9, PARAM_FIELD_KP, 30.0f));
  assert(param_store_blob_device_raw_present(blob, 9) == 0U);
  // 槽位越界拒绝。
  assert(!param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_COUNT, 1U));
}

// v1 → v2 迁移：升级不丢既有的增益/限值记录；设备端原值位图为 0（不出来源
// 不明的原值，首次上电按"无可比基准"处理）。
void test_blob_migrate_v1() {
  static uint8_t legacy[PARAM_STORE_LEGACY_BLOB_SIZE];
  std::memset(legacy, 0, sizeof(legacy));
  auto *hdr = reinterpret_cast<uint32_t *>(legacy);
  hdr[0] = PARAM_STORE_MAGIC;                    // magic
  hdr[1] = PARAM_STORE_VERSION_LEGACY | (PARAM_STORE_HEADER_SIZE << 16); // ver/size
  hdr[2] = 5U;                                   // seq
  hdr[3] = PARAM_STORE_RECORD_COUNT;             // record_count
  // 记录 0：motor 3，kp/kd/位置限位。
  const size_t rec0 = PARAM_STORE_HEADER_SIZE;
  auto *p32 = reinterpret_cast<uint32_t *>(legacy + rec0);
  p32[0] = (1U << PARAM_FIELD_KP) | (1U << PARAM_FIELD_KD) |
           (1U << PARAM_FIELD_POS_MIN) | (1U << PARAM_FIELD_POS_MAX);
  auto *p16 = reinterpret_cast<uint16_t *>(legacy + rec0);
  p16[2] = 3U; // motor_id
  auto *pf = reinterpret_cast<float *>(legacy + rec0);
  pf[2] = 12.5f; // kp
  pf[3] = 1.0f;  // kd
  pf[4] = -3.14f;
  pf[5] = 3.14f;
  // v1 CRC：头（crc 字段清零）+ 全部 32B 记录。
  auto crc32 = [](uint32_t crc, const uint8_t *d, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
      crc ^= d[i];
      for (int b = 0; b < 8; ++b) {
        crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
      }
    }
    return ~crc;
  };
  uint8_t header_copy[PARAM_STORE_HEADER_SIZE];
  std::memcpy(header_copy, legacy, PARAM_STORE_HEADER_SIZE);
  std::memset(header_copy + 16, 0, sizeof(uint32_t));
  uint32_t crc = crc32(0U, header_copy, PARAM_STORE_HEADER_SIZE);
  crc = crc32(crc, legacy + PARAM_STORE_HEADER_SIZE,
              PARAM_STORE_RECORD_COUNT * PARAM_STORE_LEGACY_RECORD_SIZE);
  hdr[4] = crc; // crc32 字段（偏移 16）

  static uint8_t out[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob = param_store_blob_migrate_v1(
      legacy, sizeof(legacy), out, sizeof(out));
  assert(blob != nullptr);
  assert(blob->version == PARAM_STORE_VERSION && blob->seq == 5U);
  assert(param_store_blob_validate(blob));
  float v = 0.0f;
  assert(param_store_blob_get(blob, 3, PARAM_FIELD_KP, &v) && v == 12.5f);
  assert(param_store_blob_get(blob, 3, PARAM_FIELD_KD, &v) && v == 1.0f);
  assert(param_store_blob_get(blob, 3, PARAM_FIELD_POS_MIN, &v) && v == -3.14f);
  assert(param_store_blob_get(blob, 3, PARAM_FIELD_POS_MAX, &v) && v == 3.14f);
  assert(!param_store_blob_get(blob, 3, PARAM_FIELD_VEL_LIMIT, &v));
  assert(param_store_blob_device_raw_present(blob, 3) == 0U);

  // CRC 损坏 → 拒绝迁移（当作无有效 blob）。
  hdr[4] ^= 0xFFU;
  assert(param_store_blob_migrate_v1(legacy, sizeof(legacy), out, sizeof(out)) ==
         nullptr);
}

// 参数服务器对外工程单位 ↔ 内核 SI（param_units.h）。
void test_param_units() {
  // 角度往返：±180° ↔ ±π，180 的整数度必须精确（页面显示的就是这些值）。
  assert(std::fabs(param_units_rad_to_deg(3.141592653589793f) - 180.0f) < 1e-3f);
  assert(std::fabs(param_units_rad_to_deg(-0.5235987756f) + 30.0f) < 1e-4f);
  assert(std::fabs(param_units_deg_to_rad(180.0f) - 3.14159265f) < 1e-5f);
  assert(std::fabs(param_units_deg_to_rad(-90.0f) + 1.5707963f) < 1e-6f);
  // 速度同系数（deg/s ↔ rad/s）。
  assert(std::fabs(param_units_rad_to_deg(1.0f) - 57.29578f) < 1e-3f);
  // 力矩：系数由固件按型号给出（这里是规格书峰值对 45 N·m ÷ 10.8 A）；
  // 设备里 13.3 A → 55.4 N·m → 回读仍是 13.3 A（往返一致，与系数取值无关）。
  const float r58 = 45.0f / 10.8f;
  assert(std::fabs(param_units_nm_to_amp(13.3f * r58, r58) - 13.3f) < 1e-3f);
  assert(std::fabs(param_units_amp_to_nm(13.3f, r58) - 55.4167f) < 0.05f);
  assert(std::fabs(param_units_amp_to_nm(param_units_nm_to_amp(45.0f, r58), r58) - 45.0f) < 1e-3f);
  // 系数缺失/非法：不换算（返回 NaN），调用方保持 SI 语义或拒绝下发。
  assert(std::isnan(param_units_nm_to_amp(45.0f, 0.0f)));
  assert(std::isnan(param_units_nm_to_amp(45.0f, NAN)));
  assert(std::isnan(param_units_amp_to_nm(5.0f, -1.0f)));
  assert(std::isnan(param_units_nm_to_amp(NAN, r58)));
  assert(std::isnan(param_units_amp_to_nm(NAN, r58)));
}

} // namespace

int main() {
  test_param_name_build_parse_roundtrip();
  test_param_name_validation();
  test_blob_v3_layout_size();
  test_blob_roundtrip_and_crc();
  test_blob_rejects_bad_layout();
  test_blob_record_capacity();
  test_blob_device_raw_roundtrip();
  test_param_units();
  test_blob_migrate_v1();
  printf("test_param_store: all passed\n");
  return 0;
}
