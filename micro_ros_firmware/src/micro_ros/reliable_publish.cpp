#include <uxr/client/core/session/write_access.h>
#include <climits>
#include "reliable_publish.hpp"

static bool g_telemetry_queue_scope = false;
ReliableTelemetryQueueScope::ReliableTelemetryQueueScope()
    : previous_(g_telemetry_queue_scope) { g_telemetry_queue_scope = true; }
ReliableTelemetryQueueScope::~ReliableTelemetryQueueScope() {
  g_telemetry_queue_scope = previous_;
}

extern "C" bool __real_uxr_update_output_stream_heartbeat_timestamp(
    uxrOutputReliableStream *, int64_t);
extern "C" void __real_uxr_process_acknack(
    uxrOutputReliableStream *, uint16_t, uxrSeqNum);
extern "C" bool __real_uxr_run_session_until_confirm_delivery(uxrSession *, int);

extern "C" bool __wrap_uxr_run_session_until_confirm_delivery(
    uxrSession *session, int timeout_ms) {
  const bool confirmed = __real_uxr_run_session_until_confirm_delivery(session, timeout_ms);
  // RMW 仅在 prepare 成功后调用此函数，返回值仍与 cdr_serialize 的结果相与。
  // 遥测交给可靠历史后即可推进本地页；不把 ACK 的延迟当作入队失败。
  return confirmed || g_telemetry_queue_scope;
}

// 可通过 SWD 只读核对异常 ACK，不向串口插入诊断文本。
extern "C" {
volatile uint32_t g_xrce_rejected_acknacks = 0;
}

// 旧 ACK 的负进度会在库内按 uint16 回绕，误清整条可靠历史。
// 接受区间为 [last_acknown, last_sent]，保留重复 NACK 和序号回绕。
extern "C" void __wrap_uxr_process_acknack(
    uxrOutputReliableStream *stream, uint16_t bitmap, uxrSeqNum first_unacked) {
  const uint16_t acknowledged = static_cast<uint16_t>(first_unacked - 1U);
  const uint16_t advance = static_cast<uint16_t>(acknowledged - stream->last_acknown);
  const uint16_t outstanding = static_cast<uint16_t>(stream->last_sent - stream->last_acknown);
  if (advance > outstanding) {
    ++g_xrce_rejected_acknacks;
    return;
  }
  __real_uxr_process_acknack(stream, bitmap, first_unacked);
}

// XRCE 2.4.2 每次轮询都会将 deadline 推后；1ms spin 下丢失末片后永不重传。
// 保留库自身的序号/ACK/心跳状态机，只阻止未到期的轮询滑动已有 deadline。
extern "C" bool __wrap_uxr_update_output_stream_heartbeat_timestamp(
    uxrOutputReliableStream *stream, int64_t now) {
  const int64_t deadline = stream->next_heartbeat_timestamp;
  const bool armed = stream->next_heartbeat_tries != 0;
  const bool due = __real_uxr_update_output_stream_heartbeat_timestamp(stream, now);
  if (!due && armed && deadline != INT64_MAX && now < deadline &&
      stream->next_heartbeat_timestamp != INT64_MAX) {
    stream->next_heartbeat_timestamp = deadline;
  }
  return due;
}

// Status/Info 均能完整放入可靠历史；队列不足时返回背压，避免连续分片超时污染后续样本。
extern "C" uint16_t __wrap_uxr_prepare_output_stream_fragmented(
    uxrSession *, uxrStreamId, uxrObjectId, ucdrBuffer *, size_t,
    uxrOnBuffersFull, void *) {
  return UXR_INVALID_REQUEST_ID;
}
