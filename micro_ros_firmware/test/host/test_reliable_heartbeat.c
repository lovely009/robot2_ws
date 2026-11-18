#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/client.h>

// Exercise the firmware's linked XRCE library, including the session retry path.
extern bool uxr_update_output_stream_heartbeat_timestamp(uxrOutputReliableStream *, int64_t);
static int64_t now_ms;
int64_t __wrap_uxr_millis(void) { return now_ms; }

static unsigned heartbeats, retransmissions;
static uint16_t next_received;
static bool dropped_tail, ack_pending, nack_pending;
static uint8_t ack[13] = {0x81, 0, 0, 0, 10, 1, 5, 0, 0, 0, 0, 0, 0x80};

static bool send_packet(void *context, const uint8_t *data, size_t size) {
  (void)context;
  assert(size >= 8);
  if (data[4] == 11) {  // HEARTBEAT exposes a missing final fragment to the peer.
    ++heartbeats;
    ack_pending = nack_pending = true;
  } else if (data[1] == 0x80) {
    const uint16_t sequence = data[2] | data[3] << 8;
    if (data[4] == 13 && (data[5] & 2) && !dropped_tail) {
      dropped_tail = true;
      return false;  // Transport budget expires on the final fragment.
    }
    if (sequence == next_received) {
      if (dropped_tail) ++retransmissions;
      ++next_received;
    }
    ack_pending = true;
    nack_pending = false;
  }
  return true;
}

static bool receive_ack(void *context, uint8_t **data, size_t *size, int timeout) {
  (void)context;
  if (ack_pending) {
    ack[8] = next_received & 0xff;
    ack[9] = next_received >> 8;
    ack[11] = nack_pending ? 1 : 0;
    *data = ack;
    *size = sizeof ack;
    ack_pending = false;
    return true;
  }
  now_ms += timeout > 0 ? timeout : 1;
  return false;
}
static uint8_t no_error(void) { return 0; }

static void timer_does_not_slide(void) {
  uint8_t storage[512 * 16];
  uxrSession session;
  uxrCommunication comm = {0};
  uxr_init_session(&session, &comm, 1);
  uxr_create_output_reliable_stream(&session, storage, sizeof storage, 16);
  uxrOutputReliableStream *stream = &session.streams.output_reliable[0];
  stream->last_sent = 0;
  stream->last_acknown = UINT16_MAX;
  unsigned sent = 0;
  for (int64_t t = 0; t <= 300; ++t)
    if (uxr_update_output_stream_heartbeat_timestamp(stream, t)) ++sent;
  assert(sent == 3);
  stream->last_acknown = stream->last_sent;
  assert(!uxr_update_output_stream_heartbeat_timestamp(stream, 301));
  assert(stream->next_heartbeat_timestamp == INT64_MAX);
  stream->last_sent = 1;
  assert(!uxr_update_output_stream_heartbeat_timestamp(stream, 1000));
  assert(!uxr_update_output_stream_heartbeat_timestamp(stream, 1099));
  assert(uxr_update_output_stream_heartbeat_timestamp(stream, 1100));
}

static void lost_tail_recovers_with_fast_polling(void) {
  uint8_t storage[512 * 16], payload[1360];
  memset(payload, 0x5a, sizeof payload);
  uxrCommunication comm = {.send_msg = send_packet, .recv_msg = receive_ack,
      .comm_error = no_error, .mtu = 512};
  uxrSession session;
  uxr_init_session(&session, &comm, 1);
  const uxrStreamId stream = uxr_create_output_reliable_stream(
      &session, storage, sizeof storage, 16);
  ucdrBuffer buffer;
  assert(uxr_prepare_output_stream(&session, stream,
      uxr_object_id(1, UXR_DATAWRITER_ID), &buffer, sizeof payload));
  assert(ucdr_serialize_array_uint8_t(&buffer, payload, sizeof payload));
  assert(!uxr_run_session_until_confirm_delivery(&session, 1));
  assert(dropped_tail);
  while (now_ms < 300 && !retransmissions) uxr_run_session_time(&session, 1);
  assert(heartbeats > 0 && retransmissions == 1);
  assert(uxr_run_session_until_confirm_delivery(&session, 1));
}

int main(void) {
  timer_does_not_slide();
  lost_tail_recovers_with_fast_polling();
  puts("reliable heartbeat: fast polling preserves retries and recovers a lost tail");
}
