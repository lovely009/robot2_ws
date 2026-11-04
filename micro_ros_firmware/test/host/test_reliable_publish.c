#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

static unsigned fragments, completed, assembled;
static uint16_t last_sequence;
static uint8_t assembled_data[8192];
static uint8_t payload[3200];
static bool acknowledge;

static bool send_packet(void *context, const uint8_t *data, size_t size) {
  (void)context;
  if (data[1] == 0x80) last_sequence = data[2] | data[3] << 8;
  for (size_t offset = 4; offset + 4 <= size;) {
    const unsigned length = data[offset + 2] | data[offset + 3] << 8;
    assert(offset + 4 + length <= size);
    if (data[offset] == 13) {  // XRCE FRAGMENT
      ++fragments;
      assert(assembled + length <= sizeof assembled_data);
      memcpy(assembled_data + assembled, data + offset + 4, length);
      assembled += length;
      if (data[offset + 1] & 2) {
        const unsigned declared = assembled_data[2] | assembled_data[3] << 8;
        assert(assembled == ((declared + 4 + 3) & ~3U));
        assert(memcmp(assembled_data + 8, payload, declared - 4) == 0);
        ++completed;
        assembled = 0;
      }
    }
    offset = (offset + 4 + length + 3) & ~(size_t)3;
  }
  return true;
}

static bool receive_ack(void *context, uint8_t **data, size_t *size, int timeout) {
  (void)context; (void)timeout;
  if (!acknowledge) return false;
  static uint8_t ack[] = {0x81, 0, 0, 0, 10, 1, 5, 0, 0, 0, 0, 0, 0x80};
  const uint16_t next = last_sequence + 1;
  ack[8] = next & 0xff; ack[9] = next >> 8;
  *data = ack; *size = sizeof ack;
  acknowledge = false;
  return true;
}

static uint8_t no_error(void) { return 0; }

static bool fail_confirmation(uxrSession *session, void *args) {
  (void)args;
  uxr_flash_output_streams(session);
  return false;
}

// 与 RMW 的普通预留 -> 连续分片后备路径一致，链接真实 XRCE 库和固件策略。
static bool publish(uxrSession *session, uxrStreamId stream, size_t length) {
  ucdrBuffer buffer;
  const uxrObjectId writer = uxr_object_id(1, UXR_DATAWRITER_ID);
  uint16_t request = uxr_prepare_output_stream(session, stream, writer, &buffer, length);
  if (!request)
    request = uxr_prepare_output_stream_fragmented(session, stream, writer,
        &buffer, length, fail_confirmation, NULL);
  if (!request) return false;
  const bool serialized = ucdr_serialize_array_uint8_t(&buffer, payload, length);
  uxr_flash_output_streams(session);
  return serialized;
}

static void confirm(uxrSession *session) {
  acknowledge = true;
  assert(uxr_run_session_until_confirm_delivery(session, 10));
}

int main(void) {
  uint8_t storage[512 * 16];
  uxrCommunication comm = {.send_msg=send_packet, .recv_msg=receive_ack,
      .comm_error=no_error, .mtu=512};
  uxrSession session;
  uxr_init_session(&session, &comm, 0x11223344);
  const uxrStreamId stream = uxr_create_output_reliable_stream(
      &session, storage, sizeof storage, 16);
  memset(payload, 0x5a, sizeof payload);
  for (unsigned cycle = 0; cycle < 4; ++cycle) {
    for (unsigned i = 0; i < 14; ++i) assert(publish(&session, stream, 400));
    const unsigned before = fragments;
    assert(!publish(&session, stream, 1360));
    assert(fragments == before && assembled == 0);
    confirm(&session);
    // 当前最大合法 Status=2525 B、Info=2303 B（含 CDR 封装），再测带余量的 3200 B。
    const size_t sizes[] = {1360, 2525, 2303, sizeof payload};
    for (unsigned i = 0; i < sizeof sizes / sizeof sizes[0]; ++i) {
      const unsigned previous = completed;
      assert(publish(&session, stream, sizes[i]));
      assert(completed == previous + 1 && assembled == 0);
      confirm(&session);
    }
  }
  puts("reliable publish: backpressure leaves no fragments; recovery and max messages pass");
}
