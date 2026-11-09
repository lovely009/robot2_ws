#include <uxr/client/core/session/write_access.h>

// Status/Info 均能完整放入可靠历史；队列不足时返回背压，避免连续分片超时污染后续样本。
extern "C" uint16_t __wrap_uxr_prepare_output_stream_fragmented(
    uxrSession *, uxrStreamId, uxrObjectId, ucdrBuffer *, size_t,
    uxrOnBuffersFull, void *) {
  return UXR_INVALID_REQUEST_ID;
}
