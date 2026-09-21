// 内核参数视图 ↔ W25 blob 的纯投影（无 Arduino/FreeRTOS/HAL 依赖，host 可测）。
// storage 任务负责 Flash 时序与双 sector，"哪些值进 blob/从 blob 出来长什么样"
// 全部收敛在本模块，是持久化字段语义的唯一归属。
#ifndef STORAGE_PROJECTION_H
#define STORAGE_PROJECTION_H

#include <stddef.h>

#include "motor_control.h"
#include "param_store_blob.h"

#ifdef __cplusplus
extern "C" {
#endif

// 内核参数视图 → blob 记录（enable 不持久化）。两类型号口径不同：
//  - CERB（设备侧限位）：只收录设备核验过的参数对象（dev_raw_present 有位，
//    共 6 槽 = 位置限位×2 / 最大转速 / 峰值电流 + 只读的 0x2650:00 保护开关与
//    0x2541 方向，后两槽同样逐位判存在性）；
//    未回读过设备时退回 record_raw 保住比对基准；位置限位成对才收录；
//  - VAXOR（本地软限位）：有限值即用户设定值，照实收录。
void storage_projection_views_to_blob(
    param_store_blob_t *blob, const motor_control_param_view_t *views,
    size_t count);

// blob → boot 记录（写入 out，最多 capacity 条，返回实际条数）。
// 字段存在性来自 present/device_raw_present 位图。
size_t storage_projection_blob_to_boot(
    const param_store_blob_t *blob, motor_control_stored_params_t *out,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
