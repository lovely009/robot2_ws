// 与电机协议和运行时策略无关的多点轨迹 mailbox。
//
// 与 control_command_mailbox 的关系：命令快照是「当前设定点」，轨迹是
// 「未来一段时间的设定点时间线」。二者各有独立通道，由内核按电机归属
// 仲裁（同一电机同一时刻只有一个是权威），此层不做仲裁。
//
// 并发契约：与 control_command_mailbox 逐字相同——本层无内部锁、无原子
// 字段，publish/take/clear 的每一次调用都必须位于调用方提供的临界区内，
// 且写侧与读侧必须共用同一互斥域。仅支持任务上下文调用。
//
// 语义：分段先在 assembly 中按序拼接，只有完整轨迹才进入双缓冲
// latest-wins。未取走时再次提交完整轨迹只覆盖「后台」并计入 overwrite_count；
// take 取走已发布轨迹并清除 pending；clear 同时撤销未取走轨迹和未完成组装。
//
// 空轨迹（point_count==0 && target_count==0）是**合法输入**，语义为「取消
// 当前轨迹」——它必须能被受理，因此不能当作非法载荷拒绝。

#ifndef CONTROL_TRAJECTORY_MAILBOX_H
#define CONTROL_TRAJECTORY_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_TRAJECTORY_MAILBOX_BUFFER_COUNT 2U
#define CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS 64U
#define CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS 256U
#define CONTROL_TRAJECTORY_MAILBOX_MAX_SEGMENTS 256U

// 路点样本：时间 + 一台电机的关节侧 SI 目标。结构对齐 4 字节（20 B/点）。
typedef struct {
  uint32_t time_from_start_ms; // 相对轨迹起点的偏移，同一电机内严格递增
  uint16_t motor_id;           // 必须属于所属轨迹的 target_ids
  float position;              // rad，关节侧（执行量：非均匀三次 Hermite）
  float velocity;              // rad/s，关节侧（诊断/自检；执行侧解析求导）
  float effort;                // N·m，关节侧（前馈，段内保持左端点值）
} control_trajectory_point_t;

typedef struct {
  uint32_t sequence;         // 诊断/日志关联，不参与裁决
  uint32_t trajectory_id;    // 上层分配，内核回带在状态与拒绝事件里
  uint32_t received_tick_ms; // 本地接收 tick（唯一执行时基，ARD §12.2）
  uint16_t segment_index;    // 0-based；完整轨迹提交后归一为 0
  uint16_t segment_count;    // 总分段数；完整轨迹提交后归一为 1
  uint16_t point_count;      // 当前载荷/完整轨迹的有效路点数
  uint16_t target_count;     // 每个分段都必须重复完整受控集合
  uint16_t target_ids[CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS];
  control_trajectory_point_t points[CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS];
} control_trajectory_t;

typedef struct {
  control_trajectory_t buffers[CONTROL_TRAJECTORY_MAILBOX_BUFFER_COUNT];
  control_trajectory_t assembly; // 固件侧分段拼接，未完成时不对外发布
  uint8_t published_buffer;
  bool pending;
  bool assembly_active;           // 有一组未完成的分段组装
  uint16_t assembly_next_segment; // 下一个应收到的 segment_index（乱序即拒）
  uint32_t overwrite_count;
  uint32_t invalid_trajectory_count;
} control_trajectory_mailbox_t;

void control_trajectory_mailbox_init(control_trajectory_mailbox_t *mailbox);
bool control_trajectory_mailbox_publish(control_trajectory_mailbox_t *mailbox,
                                       const control_trajectory_t *trajectory);
bool control_trajectory_mailbox_take(control_trajectory_mailbox_t *mailbox,
                                     control_trajectory_t *trajectory);
void control_trajectory_mailbox_clear(control_trajectory_mailbox_t *mailbox);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_TRAJECTORY_MAILBOX_H
