// W25Q64 参数存储任务（Flash 的唯一所有者，ARD 安全边界）：
//  - start：同步初始化 W25（无芯片/ID 不符则静默降级为无持久化，不阻断启动），
//    读双 sector A/B 选 CRC 有效且 seq 最新的 blob；
//  - boot 参数：start 后经 storage_boot_params() 供 motor_rt 在 20ms 边界套用；
//  - save：运行期仅由显式请求触发（system.save 脉冲）；erase 阻塞最长约 1s，
//    只能在本任务执行，严禁从 micro-ROS/motor_rt 上下文直接调 W25。
#ifndef STORAGE_TASK_H_
#define STORAGE_TASK_H_

#include <stdbool.h>
#include <stddef.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  STORAGE_SAVE_IDLE = 0,
  STORAGE_SAVE_OK = 1,
  STORAGE_SAVE_FAIL = 2,
} storage_save_status_t;

// 初始化 W25 + 加载 blob + 创建存储任务。无 W25 时返回 true（降级模式）；
// 只有任务创建失败等内部错误才返回 false。
bool storage_task_start(void);

// 请求一次保存（任务通知；保存进行中的重复请求合并丢弃）。
void storage_task_request_save(void);

// 取走上次保存结果（无新结果返回 IDLE）。
storage_save_status_t storage_take_result(void);

// boot 加载的存储参数（storage_task_start 后有效，运行期不变；save 的结果
// 下次启动才生效）。无有效 blob 时 count=0。
const motor_control_stored_params_t *storage_boot_params(size_t *count);

#ifdef __cplusplus
}
#endif

#endif // STORAGE_TASK_H_
