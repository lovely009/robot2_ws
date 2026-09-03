#include "can_snapshot.h"

#include <cassert>

int main() {
  can_rx_frame_t input = {};
  input.bus_index = 0U;
  input.id = 0x123U;
  input.dlc = 2U;
  input.data[0] = 0xAAU;
  input.data[1] = 0x55U;
  can_snapshot_store(0U, &input);

  can_rx_frame_t frame = {};
  assert(can_snapshot_read(0U, &frame) == 1U);
  assert(frame.id == 0x123U);
  assert(frame.dlc == 2U);
  assert(frame.data[0] == 0xAAU);
  assert(frame.data[1] == 0x55U);

  assert(can_snapshot_read(1U, &frame) == 0U);
  return 0;
}