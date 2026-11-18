#include "reliable_publish.hpp"
#include <uxr/client/client.h>
#include <cassert>
#include <cstdio>
#include <cstring>

static int64_t now_ms;
extern "C" int64_t __wrap_uxr_millis(void) { return now_ms; }
static uint16_t last_sent;
static bool acknowledge;
static bool send_packet(void *, const uint8_t *data, size_t) {
  if (data[1] == 0x80) last_sent = data[2] | data[3] << 8;
  return true;
}
static bool receive_ack(void *, uint8_t **data, size_t *size, int timeout) {
  if (acknowledge) {
    static uint8_t ack[] = {0x81, 0, 0, 0, 10, 1, 5, 0, 0, 0, 0, 0, 0x80};
    const uint16_t next = last_sent + 1;
    ack[8] = next & 0xff; ack[9] = next >> 8;
    *data = ack; *size = sizeof ack; acknowledge = false; return true;
  }
  now_ms += timeout > 0 ? timeout : 1;
  return false;
}
static uint8_t no_error() { return 0; }

// 与已锁定 RMW 的 prepare -> serialize && confirm 流程一致。
static bool publish(uxrSession *session, uxrStreamId stream) {
  ucdrBuffer buffer;
  uint8_t payload[1360]{};
  if (!uxr_prepare_output_stream(session, stream,
      uxr_object_id(1, UXR_DATAWRITER_ID), &buffer, sizeof payload)) return false;
  const bool serialized = ucdr_serialize_array_uint8_t(&buffer, payload, sizeof payload);
  return serialized & uxr_run_session_until_confirm_delivery(session, 1);
}

int main() {
  uint8_t storage[512 * 16];
  uxrCommunication comm{};
  comm.send_msg = send_packet; comm.recv_msg = receive_ack; comm.comm_error = no_error;
  comm.mtu = 512;
  uxrSession session;
  uxr_init_session(&session, &comm, 1);
  const auto stream = uxr_create_output_reliable_stream(&session, storage, sizeof storage, 16);
  assert(!publish(&session, stream));  // 无 ACK 时，连接等原调用仍报告未确认。
  {
    ReliableTelemetryQueueScope scope;
    assert(publish(&session, stream));  // 遥测已入队，不重复保留旧设备页。
    assert(session.streams.output_reliable[0].last_acknown == UINT16_MAX);
    unsigned queued = 0;
    while (publish(&session, stream)) ++queued;
    assert(queued > 0);  // 历史耗尽仍必须失败，不能吞掉背压。
  }
  assert(!uxr_run_session_until_confirm_delivery(&session, 1));
  acknowledge = true;
  assert(uxr_run_session_until_confirm_delivery(&session, 1));
  assert(session.streams.output_reliable[0].last_acknown == last_sent);
  {
    ReliableTelemetryQueueScope scope;
    assert(publish(&session, stream));  // 确认回收后可以再次入队。
  }
  assert(!uxr_run_session_until_confirm_delivery(&session, 1));
  std::puts("reliable telemetry: enqueue advances pages; backpressure and ACK ownership remain intact");
}
