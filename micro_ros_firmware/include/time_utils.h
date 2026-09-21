// 时间戳工具：统一填充 ROS 2 header.stamp。
// 与 agent 建链后可经 rmw_uros_sync_session() 做 NTP 同步，同步后
// rmw_uros_epoch_nanos() 返回与主机对齐的 epoch 时间（1970-01-01 起）；各业务桥应
// 优先用它，未同步（断连/首次建链前）回退 millis()，避免发出 0 时间戳。

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <stdint.h>

#include <builtin_interfaces/msg/time.h>

#ifdef __cplusplus
extern "C" {
#endif

// 用「epoch 优先、millis 回退」填充 header.stamp。
// 同步成功时写入 epoch 秒/纳秒；否则写入 millis() 的秒/纳秒。
void fill_header_stamp(builtin_interfaces__msg__Time *stamp);

#ifdef __cplusplus
}
#endif

#endif /* TIME_UTILS_H */
