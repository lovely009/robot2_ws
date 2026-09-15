#include "motor_static_topology.h"

#include <stddef.h>

typedef struct {
  uint16_t motor_id;
  const char *model_name; // 与 motor_control_profile.c 的 model_name 锚点一致
} motor_static_entry_t;

// 真机台架（2026-09-15）：1 号 R58，2-4 号 R48，全部 FDCAN1。
static const motor_static_entry_t kStaticTopology[] = {
    {1U, "cerb_r58_101"},
    {2U, "cerb_r48_101"},
    {3U, "cerb_r48_101"},
    {4U, "cerb_r48_101"},
};

const char *motor_static_topology_model_for(uint16_t motor_id) {
  for (size_t i = 0; i < sizeof(kStaticTopology) / sizeof(kStaticTopology[0]);
       ++i) {
    if (kStaticTopology[i].motor_id == motor_id) {
      return kStaticTopology[i].model_name;
    }
  }
  return NULL;
}
