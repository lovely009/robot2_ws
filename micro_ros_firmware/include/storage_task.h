// W25Q64 参数存储任务（Flash 的唯一所有者，ARD 安全边界）：
//  - start：同步初始化 W25（无芯片/ID 不符则静默降级为无持久化，不阻断启动），
//    读双 sector A/B 选 CRC 有效且 seq 最新的 blob；
//  - boot 参数：start 后经 storage_boot_params() 供 motor_rt 在 5ms/200Hz 边界套用；
//  - save **只有一个触发源**：param_bridge 收到 system_command 的 COMMAND_SAVE 后经
//    storage_task_request_save(request_id) 请求。erase 阻塞最长约 1s，只能在本任务执行——
//    严禁从 micro-ROS/motor_rt 上下文直接调 W25。
//  - 同一 blob 头部保存系统参数；电机记录布局和双槽 CRC 规则保持不变。
#ifndef STORAGE_TASK_H
#define STORAGE_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// 参数快照提供者（由 motor_rt 注册）：在本任务上下文、内核临界区内拍视图。
// 未注册时 save 直接回 STORAGE_SAVE_FAIL（等价无 Flash 降级）。
typedef size_t (*storage_param_snapshot_fn_t)(
  motor_control_param_view_t *out, size_t capacity);
void storage_task_set_param_snapshot(storage_param_snapshot_fn_t fn);

typedef enum {
  STORAGE_SAVE_IDLE = 0,
  STORAGE_SAVE_OK = 1,
  STORAGE_SAVE_FAIL = 2,
} storage_save_status_t;

// 初始化 W25 + 加载 blob + 创建存储任务。无 W25 时返回 true（降级模式）；
// 只有任务创建失败等内部错误才返回 false。
bool storage_task_start(void);

// 请求一次保存。request_id 随任务通知传递，用于把结果与当前 micro-ROS 会话的
// 保存事务关联；保存进行中的重复请求只保留最新通知。
void storage_task_request_save(uint32_t request_id);

// 取走上次保存结果（无新结果返回 IDLE）。request_id 可为 NULL；结果与令牌在
// 同一短临界区内取走，避免任务间看到不匹配的一半状态。
storage_save_status_t storage_take_result(uint32_t *request_id);

// boot 加载的存储参数（storage_task_start 后有效，运行期不变；save 的结果
// 下次启动才生效）。无有效 blob 时 count=0。
const motor_control_stored_params_t *storage_boot_params(size_t *count);

#ifdef __cplusplus
}
#endif

#endif // STORAGE_TASK_H
