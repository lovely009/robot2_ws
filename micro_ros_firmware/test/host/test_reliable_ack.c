#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/client.h>

extern void uxr_process_acknack(uxrOutputReliableStream *, uint16_t, uxrSeqNum);

int main(void) {
  uint8_t storage[512 * 16], before[sizeof storage];
  uxrCommunication comm = {0};
  uxrSession session;
  uxr_init_session(&session, &comm, 1);
  const uxrStreamId id = uxr_create_output_reliable_stream(
      &session, storage, sizeof storage, 16);
  uxrOutputReliableStream *stream = &session.streams.output_reliable[0];
  ucdrBuffer buffer;
  // Reserve a real fragmented sample, with no send/ACK yet.
  assert(uxr_prepare_output_stream(&session, id,
      uxr_object_id(1, UXR_DATAWRITER_ID), &buffer, 1360));
  uint8_t payload[1360];
  memset(payload, 0x5a, sizeof payload);
  assert(ucdr_serialize_array_uint8_t(&buffer, payload, sizeof payload));
  memcpy(before, storage, sizeof storage);
  stream->last_acknown = 10;
  stream->last_sent = 12;
  stream->next_heartbeat_tries = 1;
  stream->next_heartbeat_timestamp = 100;
  // Delayed ACK would wrap the library's cleanup loop to 65535 entries.
  uxr_process_acknack(stream, 1, 10);
  assert(stream->last_acknown == 10);
  assert(memcmp(before, storage, sizeof storage) == 0);
  assert(stream->next_heartbeat_timestamp == 100 && !stream->send_lost);
  // A peer cannot acknowledge packets the sender has not sent.
  uxr_process_acknack(stream, 0, 14);
  assert(stream->last_acknown == 10);
  assert(memcmp(before, storage, sizeof storage) == 0);
  // Duplicate ACK/NACK remains useful for requesting retransmission.
  uxr_process_acknack(stream, 1, 11);
  assert(stream->last_acknown == 10 && stream->send_lost);
  // Normal acknowledgement advances, and sequence wrap is valid.
  uxr_process_acknack(stream, 0, 13);
  assert(stream->last_acknown == 12 && !stream->send_lost);
  stream->last_acknown = UINT16_MAX - 1;
  stream->last_sent = 1;
  uxr_process_acknack(stream, 0, 2);
  assert(stream->last_acknown == 1);
  puts("reliable ACK: stale/future receipts preserve queued fragments; duplicates and wrap pass");
}
