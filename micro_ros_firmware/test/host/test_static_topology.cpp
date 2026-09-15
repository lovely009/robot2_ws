// 编译期拓扑表 host test：真机台架（1×R58 + 3×R48）id→型号映射、表外
// NULL、表内型号名必须能在编译期型号目录命中（锚点一致性，防手滑改名）。
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "motor_control_profile.h"
#include "motor_static_topology.h"

static void test_bench_mapping() {
  assert(strcmp(motor_static_topology_model_for(1), "cerb_r58_101") == 0);
  for (uint16_t id = 2; id <= 4; ++id) {
    assert(strcmp(motor_static_topology_model_for(id), "cerb_r48_101") == 0);
  }
  assert(motor_static_topology_model_for(0) == NULL);
  assert(motor_static_topology_model_for(5) == NULL);
  assert(motor_static_topology_model_for(9) == NULL);
}

static void test_models_exist_in_catalog() {
  const uint16_t ids[] = {1, 2, 3, 4};
  for (uint16_t id : ids) {
    const char *model = motor_static_topology_model_for(id);
    assert(model != NULL);
    bool found = false;
    for (size_t i = 0; i < motor_control_profile_count(); ++i) {
      if (strcmp(motor_control_profile_at(i)->model_name, model) == 0) {
        found = true;
        break;
      }
    }
    assert(found);
  }
}

int main() {
  test_bench_mapping();
  test_models_exist_in_catalog();
  printf("test_static_topology: all passed\n");
  return 0;
}
