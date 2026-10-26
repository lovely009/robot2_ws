// motor_tx_router host test: queue policy to physical-frame submission.

#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "motor_tx_router.h"

struct submit_capture_t {
  can_tx_frame_t frames[64];
  size_t count;
  uint32_t reject_p0_ids[4];
  size_t reject_p0_count;
};

static can_tx_frame_t make_frame(uint32_t can_id, uint8_t marker) {
  can_tx_frame_t frame = {};
  frame.can_id = can_id;
  frame.is_fd = true;
  frame.size = 8U;
  frame.data[0] = marker;
  return frame;
}

static can_tx_frame_t make_cerb_mit_frame(uint8_t node) {
  can_tx_frame_t frame = {};
  // NodeID>15 没有无歧义的单轴 ID：模拟内核产出的聚合成员帧。
  if (node > MOTOR_PROTOCOL_CERB_SINGLE_NODE_MAX) {
    frame.can_id = MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID;
    frame.node = node;
    frame.aggregate_only = true;
  } else {
    frame.can_id = MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE | node;
  }
  frame.is_fd = true;
  frame.is_brs = true;
  frame.size = 12U;
  frame.data[0] = 0x0CU;
  for (uint8_t i = 1U; i < 9U; ++i) {
    frame.data[i] = (uint8_t)(node + i);
  }
  return frame;
}

static bool capture_submit(void *context, uint8_t bus_index,
    const can_tx_frame_t *frame) {
  auto *capture = static_cast<submit_capture_t *>(context);
  assert(bus_index == 0U);
  assert(frame != nullptr);
  assert(capture->count < sizeof(capture->frames) / sizeof(capture->frames[0]));
  capture->frames[capture->count++] = *frame;
  for (size_t i = 0U; i < capture->reject_p0_count; ++i) {
    if (capture->reject_p0_ids[i] == frame->can_id) {
      capture->reject_p0_ids[i] = 0U;
      return false;
    }
  }
  return true;
}

static void test_cerb_commands_batch_and_singleton_stays_single_axis() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  const can_tx_frame_t first = make_cerb_mit_frame(1U);
  const can_tx_frame_t second = make_cerb_mit_frame(2U);
  assert(can_tx_scheduler_submit_p1(&scheduler, 1U, &first));
  assert(can_tx_scheduler_submit_p1(&scheduler, 2U, &second));
  motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames == 2U);
  assert(stats.physical_attempts == 1U);
  assert(capture.count == 1U);
  assert(capture.frames[0].can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(capture.frames[0].size == 64U);
  assert(capture.frames[0].data[56] == 1U && capture.frames[0].data[57] == 2U);

  const can_tx_frame_t singleton = make_cerb_mit_frame(3U);
  assert(can_tx_scheduler_submit_p1(&scheduler, 3U, &singleton));
  stats = motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames == 1U && stats.physical_attempts == 1U);
  assert(capture.count == 2U);
  assert(capture.frames[1].can_id == singleton.can_id);
  assert(capture.frames[1].size == singleton.size);
  assert(memcmp(capture.frames[1].data, singleton.data, singleton.size) == 0);
}

// 高节点只能经 0x210 上线：两条高节点 P1 必须聚合成一条 64B 多轴帧，
// 不允许退回带歧义的单轴 ID。
static void test_cerb_high_nodes_aggregate_into_multi_axis() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  const can_tx_frame_t first = make_cerb_mit_frame(16U);
  const can_tx_frame_t second = make_cerb_mit_frame(127U);
  assert(can_tx_scheduler_submit_p1(&scheduler, 0U, &first));
  assert(can_tx_scheduler_submit_p1(&scheduler, 1U, &second));
  const motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames == 2U);
  assert(stats.physical_attempts == 1U);
  assert(stats.aggregate_dropped == 0U);
  assert(capture.count == 1U);
  assert(capture.frames[0].can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(capture.frames[0].size == 64U);
  assert(capture.frames[0].data[56] == 16U);
  assert(capture.frames[0].data[57] == 127U);
}

// 单个高节点也必须走 0x210（不能因为没有同批伙伴而退回 0x110|node）。
static void test_cerb_lone_high_node_still_uses_multi_axis() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  const can_tx_frame_t lone = make_cerb_mit_frame(20U);
  assert(can_tx_scheduler_submit_p1(&scheduler, 0U, &lone));
  const motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames == 1U);
  assert(stats.physical_attempts == 1U);
  assert(stats.aggregate_dropped == 0U);
  assert(capture.count == 1U);
  assert(capture.frames[0].can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(capture.frames[0].can_id !=
      (MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE | 20U));
  assert(capture.frames[0].size == 64U);
  assert(capture.frames[0].data[56] == 20U);
}

static void test_p2_reservation_and_physical_budget() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  for (uint16_t slot = 0U; slot < CAN_TX_P1_SLOTS; ++slot) {
    const can_tx_frame_t frame = make_cerb_mit_frame((uint8_t)(slot + 1U));
    assert(can_tx_scheduler_submit_p1(&scheduler, slot, &frame));
  }
  for (uint8_t i = 0U; i < CAN_TX_P2_PUMP_QUOTA; ++i) {
    const can_tx_frame_t frame = make_frame(0x080U + i, i);
    assert(can_tx_scheduler_submit_p2(&scheduler, &frame));
  }

  const motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames ==
      MOTOR_TX_ROUTER_LOGICAL_CAPACITY - CAN_TX_P2_PUMP_QUOTA);
  assert(stats.p2_logical_frames == CAN_TX_P2_PUMP_QUOTA);
  assert(stats.physical_attempts <= MOTOR_TX_ROUTER_PHYSICAL_BUDGET);
  assert(capture.count == stats.physical_attempts);
  assert(capture.frames[capture.count - 2U].can_id == 0x080U);
  assert(capture.frames[capture.count - 1U].can_id == 0x081U);
}

static void test_p0_rejection_requeues_ahead_of_other_traffic() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};
  capture.reject_p0_ids[0] = 0x500U;
  capture.reject_p0_count = 1U;

  const can_tx_frame_t p0 = make_frame(0x500U, 0xA0U);
  const can_tx_frame_t p1 = make_cerb_mit_frame(1U);
  const can_tx_frame_t p2 = make_frame(0x080U, 0xB0U);
  assert(can_tx_scheduler_submit_p0(&scheduler, &p0));
  assert(can_tx_scheduler_submit_p1(&scheduler, 1U, &p1));
  assert(can_tx_scheduler_submit_p2(&scheduler, &p2));

  motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p0_requeued == 1U);
  assert(stats.hardware_rejections == 1U);
  assert(scheduler.p0.count == 1U);
  assert(capture.frames[0].can_id == p0.can_id);

  const size_t before = capture.count;
  stats = motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p0_logical_frames == 1U);
  assert(capture.count > before);
  assert(capture.frames[before].can_id == p0.can_id);
}

static void test_p0_rejection_stops_this_pump_and_preserves_lower_classes() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};
  capture.reject_p0_ids[0] = 0x500U;
  capture.reject_p0_count = 1U;

  const can_tx_frame_t first = make_frame(0x500U, 0xA0U);
  const can_tx_frame_t second = make_frame(0x501U, 0xA1U);
  const can_tx_frame_t p1 = make_cerb_mit_frame(1U);
  const can_tx_frame_t p2 = make_frame(0x080U, 0xB0U);
  assert(can_tx_scheduler_submit_p0(&scheduler, &first));
  assert(can_tx_scheduler_submit_p0(&scheduler, &second));
  assert(can_tx_scheduler_submit_p1(&scheduler, 1U, &p1));
  assert(can_tx_scheduler_submit_p2(&scheduler, &p2));

  const motor_tx_router_stats_t rejected =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(rejected.physical_attempts == 1U);
  assert(rejected.hardware_rejections == 1U);
  assert(rejected.p0_requeued == 2U);
  assert(rejected.p1_logical_frames == 0U);
  assert(rejected.p2_logical_frames == 0U);
  assert(capture.count == 1U);
  assert(scheduler.p0.count == 2U);
  assert(scheduler.p1.dirty[1U]);
  assert(scheduler.p2.count == 1U);

  const motor_tx_router_stats_t retried =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(retried.p0_logical_frames == 2U);
  assert(capture.frames[1].can_id == first.can_id);
  assert(capture.frames[2].can_id == second.can_id);
  assert(scheduler.p1.dirty[1U]);
  assert(scheduler.p2.count == 1U);

  const motor_tx_router_stats_t lower_priority =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(lower_priority.p1_logical_frames == 1U);
  assert(lower_priority.p2_logical_frames == 1U);
}

static void test_multiple_rejected_p0_frames_keep_fifo_order() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};
  capture.reject_p0_ids[0] = 0x500U;
  capture.reject_p0_ids[1] = 0x501U;
  capture.reject_p0_count = 2U;

  const can_tx_frame_t first = make_frame(0x500U, 0U);
  const can_tx_frame_t second = make_frame(0x501U, 1U);
  assert(can_tx_scheduler_submit_p0(&scheduler, &first));
  assert(can_tx_scheduler_submit_p0(&scheduler, &second));
  motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p0_requeued == 2U);

  const size_t before = capture.count;
  stats = motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p0_logical_frames == 2U);
  assert(capture.frames[before].can_id == first.can_id);
  assert(capture.frames[before + 1U].can_id == second.can_id);
}

static void test_p1_batch_does_not_cross_an_unbatchable_frame() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  const can_tx_frame_t first = make_cerb_mit_frame(1U);
  const can_tx_frame_t unrelated = make_frame(0x055U, 0x55U);
  const can_tx_frame_t last = make_cerb_mit_frame(2U);
  assert(can_tx_scheduler_submit_p1(&scheduler, 0U, &first));
  assert(can_tx_scheduler_submit_p1(&scheduler, 1U, &unrelated));
  assert(can_tx_scheduler_submit_p1(&scheduler, 2U, &last));

  const motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames == 3U);
  assert(stats.physical_attempts == 3U);
  assert(capture.frames[0].can_id == first.can_id);
  assert(capture.frames[1].can_id == unrelated.can_id);
  assert(capture.frames[2].can_id == last.can_id);
}

static void test_unbatchable_p1_never_exceeds_hardware_fifo_budget() {
  can_tx_scheduler_t scheduler = {};
  can_tx_scheduler_init(&scheduler);
  motor_tx_router_t router = {};
  motor_tx_router_init(&router);
  submit_capture_t capture = {};

  for (uint16_t slot = 0U; slot < CAN_TX_P1_SLOTS; ++slot) {
    const can_tx_frame_t frame = make_frame(0x500U + slot, (uint8_t)slot);
    assert(can_tx_scheduler_submit_p1(&scheduler, slot, &frame));
  }
  for (uint8_t i = 0U; i < CAN_TX_P2_PUMP_QUOTA; ++i) {
    const can_tx_frame_t frame = make_frame(0x080U + i, i);
    assert(can_tx_scheduler_submit_p2(&scheduler, &frame));
  }

  const motor_tx_router_stats_t stats =
      motor_tx_router_pump(&router, &scheduler, 0U, capture_submit, &capture);
  assert(stats.p1_logical_frames ==
      MOTOR_TX_ROUTER_PHYSICAL_BUDGET - CAN_TX_P2_PUMP_QUOTA);
  assert(stats.p2_logical_frames == CAN_TX_P2_PUMP_QUOTA);
  assert(stats.physical_attempts == MOTOR_TX_ROUTER_PHYSICAL_BUDGET);
}

int main() {
  test_cerb_commands_batch_and_singleton_stays_single_axis();
  test_cerb_high_nodes_aggregate_into_multi_axis();
  test_cerb_lone_high_node_still_uses_multi_axis();
  test_p2_reservation_and_physical_budget();
  test_p0_rejection_requeues_ahead_of_other_traffic();
  test_p0_rejection_stops_this_pump_and_preserves_lower_classes();
  test_multiple_rejected_p0_frames_keep_fifo_order();
  test_p1_batch_does_not_cross_an_unbatchable_frame();
  test_unbatchable_p1_never_exceeds_hardware_fifo_budget();
  std::printf("test_motor_tx_router: all passed\n");
  return 0;
}
