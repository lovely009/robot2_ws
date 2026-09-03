#include "can_snapshot.h"

static can_rx_frame_t g_snapshots[CAN_RX_BUS_COUNT];
static uint32_t g_sequences[CAN_RX_BUS_COUNT];

void can_snapshot_store(uint8_t bus_index, const can_rx_frame_t *frame) {
  if (bus_index >= CAN_RX_BUS_COUNT || frame == 0) {
    return;
  }
  g_snapshots[bus_index] = *frame;
  ++g_sequences[bus_index];
}

uint32_t can_snapshot_read(uint8_t bus_index, can_rx_frame_t *out) {
  if (bus_index >= CAN_RX_BUS_COUNT) {
    return 0U;
  }
  if (out != 0) {
    *out = g_snapshots[bus_index];
  }
  return g_sequences[bus_index];
}