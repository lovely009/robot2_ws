// storage_projection 特征测试：内核参数视图 ↔ blob ↔ boot 记录的投影语义。
// 这些规则原藏在 storage_task.cpp 任务文件里，抽出纯 C 后在 host 钉死。
#include "storage_projection.h"

#include <cassert>
#include <cmath>
#include <cstring>

namespace {

alignas(8) uint8_t g_blob_storage[PARAM_STORE_BLOB_SIZE];

param_store_blob_t *reset_blob() {
  auto *blob = param_store_blob_reset(g_blob_storage, sizeof(g_blob_storage));
  assert(blob != nullptr);
  return blob;
}

const param_store_record_t *find_record(const param_store_blob_t *blob,
                                        uint16_t motor_id) {
  for (uint32_t i = 0; i < blob->record_count; ++i) {
    const auto &r = blob->records[i];
    if (r.motor_id == motor_id &&
        (r.present != 0U || r.device_raw_present != 0U)) {
      return &r;
    }
  }
  return nullptr;
}

motor_control_param_view_t cerb_view(uint16_t id) {
  motor_control_param_view_t v{};
  v.motor_id = id;
  v.online = true;
  v.bound = true;
  v.device_side_limits = true;
  v.kp = NAN;
  v.kd = NAN;
  v.pos_min_rad = -1.0f;
  v.pos_max_rad = 1.0f;
  v.velocity_limit_rad_s = 2.0f;
  v.torque_limit_nm = 3.0f;
  return v;
}

} // namespace

// CERB：只收录设备核验过的参数对象（dev_raw_present 有位），原值同时落盘。
static void test_cerb_only_verified_objects_recorded() {
  auto *blob = reset_blob();
  motor_control_param_view_t views[2] = {cerb_view(1), cerb_view(2)};
  // id1：位置对 + 速度位有设备核验；扭矩位缺失
  views[0].kp = 4.0f;
  views[0].kd = 5.0f;
  views[0].dev_raw_present =
      (1U << PARAM_DEVICE_SLOT_POS_MIN) | (1U << PARAM_DEVICE_SLOT_POS_MAX) |
      (1U << PARAM_DEVICE_SLOT_VEL_LIMIT);
  views[0].dev_raw[PARAM_DEVICE_SLOT_POS_MIN] = 100;
  views[0].dev_raw[PARAM_DEVICE_SLOT_POS_MAX] = 200;
  views[0].dev_raw[PARAM_DEVICE_SLOT_VEL_LIMIT] = 300;

  storage_projection_views_to_blob(blob, views, 2);

  const param_store_record_t *r1 = find_record(blob, 1);
  assert(r1 != nullptr);
  // kp/kd 有限值照常收录
  assert(r1->present & (1U << PARAM_FIELD_KP));
  assert(r1->present & (1U << PARAM_FIELD_KD));
  // 位置对 + 速度收录、扭矩（无设备位）不收录
  assert(r1->present & (1U << PARAM_FIELD_POS_MIN));
  assert(r1->present & (1U << PARAM_FIELD_POS_MAX));
  assert(r1->present & (1U << PARAM_FIELD_VEL_LIMIT));
  assert((r1->present & (1U << PARAM_FIELD_TORQUE_LIMIT)) == 0U);
  assert(r1->device_raw[PARAM_DEVICE_SLOT_POS_MIN] == 100);
  assert(r1->device_raw[PARAM_DEVICE_SLOT_POS_MAX] == 200);
  assert(r1->device_raw[PARAM_DEVICE_SLOT_VEL_LIMIT] == 300);
  // id2 一个设备位都没有：限值一个不收录（但也不产生空记录）
  assert(find_record(blob, 2) == nullptr);
}

// 位置限位不成对：即便 SI 有限也不收录，原值不落盘。
static void test_cerb_unpaired_position_excluded() {
  auto *blob = reset_blob();
  motor_control_param_view_t v = cerb_view(3);
  v.dev_raw_present = (1U << PARAM_DEVICE_SLOT_POS_MIN); // 只有下限
  v.dev_raw[PARAM_DEVICE_SLOT_POS_MIN] = 7;
  motor_control_param_view_t views[1] = {v};
  storage_projection_views_to_blob(blob, views, 1);
  const param_store_record_t *r = find_record(blob, 3);
  assert(r == nullptr);
}

// 未回读过设备时退回 record_raw 原值（保住上电比对基准）。
static void test_cerb_falls_back_to_record_raw() {
  auto *blob = reset_blob();
  motor_control_param_view_t v = cerb_view(4);
  v.dev_raw_present = 0U; // 本次上电没回读过
  v.record_raw_present =
      (1U << PARAM_DEVICE_SLOT_POS_MIN) | (1U << PARAM_DEVICE_SLOT_POS_MAX);
  v.record_raw[PARAM_DEVICE_SLOT_POS_MIN] = 11;
  v.record_raw[PARAM_DEVICE_SLOT_POS_MAX] = 22;
  motor_control_param_view_t views[1] = {v};
  storage_projection_views_to_blob(blob, views, 1);
  const param_store_record_t *r = find_record(blob, 4);
  assert(r != nullptr);
  assert(r->device_raw[PARAM_DEVICE_SLOT_POS_MIN] == 11);
  assert(r->device_raw[PARAM_DEVICE_SLOT_POS_MAX] == 22);
  assert(r->device_raw_present == v.record_raw_present);
}

// VAXOR（本地软限位）：四个有限限值照收；NaN 字段不收录。
static void test_vaxor_soft_limits_finite_only() {
  auto *blob = reset_blob();
  motor_control_param_view_t v{};
  v.motor_id = 7;
  v.device_side_limits = false;
  v.kp = 1.5f;
  v.kd = NAN; // blob 侧 kp/kd 独立收录（成对要求在 boot 投影）
  v.pos_min_rad = -0.5f;
  v.pos_max_rad = 0.5f;
  v.velocity_limit_rad_s = 9.0f;
  v.torque_limit_nm = NAN;
  motor_control_param_view_t views[1] = {v};
  storage_projection_views_to_blob(blob, views, 1);
  const param_store_record_t *r = find_record(blob, 7);
  assert(r != nullptr);
  assert(r->present & (1U << PARAM_FIELD_KP));
  assert((r->present & (1U << PARAM_FIELD_KD)) == 0U);
  assert(r->present & (1U << PARAM_FIELD_POS_MIN));
  assert(r->present & (1U << PARAM_FIELD_POS_MAX));
  assert(r->present & (1U << PARAM_FIELD_VEL_LIMIT));
  assert((r->present & (1U << PARAM_FIELD_TORQUE_LIMIT)) == 0U);
  assert(r->device_raw_present == 0U);
}

// blob → boot：位图决定 has_* 与设备原值；无记录的槽不产生条目。
static void test_blob_to_boot_projection() {
  auto *blob = reset_blob();
  motor_control_param_view_t v{};
  v.motor_id = 5;
  v.device_side_limits = false;
  v.kp = 1.0f;
  v.kd = 2.0f;
  v.pos_min_rad = -3.0f;
  v.pos_max_rad = 3.0f;
  v.velocity_limit_rad_s = NAN;
  v.torque_limit_nm = NAN;
  motor_control_param_view_t views[1] = {v};
  storage_projection_views_to_blob(blob, views, 1);

  motor_control_stored_params_t out[4] = {};
  size_t n = storage_projection_blob_to_boot(blob, out, 4);
  assert(n == 1U);
  assert(out[0].motor_id == 5);
  assert(out[0].has_kp_kd);
  assert(out[0].kp == 1.0f && out[0].kd == 2.0f);
  assert(out[0].has_position_limits);
  assert(out[0].pos_min_rad == -3.0f && out[0].pos_max_rad == 3.0f);
  assert(!out[0].has_velocity_limit && !out[0].has_torque_limit);

  // 只有 kp 没有 kd：blob 里有位，boot 投影不置 has_kp_kd（成对要求）。
  {
    auto *b2 = reset_blob();
    motor_control_param_view_t w{};
    w.motor_id = 6;
    w.device_side_limits = false;
    w.kp = 9.0f;
    w.kd = NAN;
    motor_control_param_view_t vs[1] = {w};
    storage_projection_views_to_blob(b2, vs, 1);
    motor_control_stored_params_t o2[2] = {};
    assert(storage_projection_blob_to_boot(b2, o2, 2) == 1U);
    assert(!o2[0].has_kp_kd);
  }

  // capacity 上界
  motor_control_stored_params_t small[0] = {};
  assert(storage_projection_blob_to_boot(blob, small, 0) == 0U);
}

// motor_id=0 的视图被跳过（0 非法）。
static void test_zero_motor_id_skipped() {
  auto *blob = reset_blob();
  motor_control_param_view_t v = cerb_view(0);
  v.dev_raw_present = 0xFFU;
  motor_control_param_view_t views[1] = {v};
  storage_projection_views_to_blob(blob, views, 1);
  assert(find_record(blob, 0) == nullptr);
}

int main() {
  test_cerb_only_verified_objects_recorded();
  test_cerb_unpaired_position_excluded();
  test_cerb_falls_back_to_record_raw();
  test_vaxor_soft_limits_finite_only();
  test_blob_to_boot_projection();
  test_zero_motor_id_skipped();
  return 0;
}
