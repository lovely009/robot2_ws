#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_dynamics/arm_model.hpp"

namespace robot_dynamics {

// motor_id → 关节名。字符串解析（joint_map 参数）在节点侧，算法层只收成品。
using MotorJointMap = std::unordered_map<std::uint16_t, std::string>;

// 轨迹路点的最小视图：把 can_bridge_msgs::MotorTrajectoryPoint 压成这个结构，
// 算法层因此不依赖 ROS 消息类型。
struct TrajectoryPoint {
  std::uint32_t time_from_start_ms{0};
  std::uint16_t motor_id{0};
  double position_rad{0.0};
};

struct TrajectoryFeedforward {
  std::vector<double> effort_delta_nm;            // 与 points 逐位对齐 [Nm]
  std::vector<std::uint16_t> unmapped_motor_ids;  // joint_map 里没有的 motor_id
  std::vector<std::string> ignored_joint_names;   // joint_map 有但 URDF 不可控的关节名
};

// 逐路点算 τ_g 增量（不含 effort_gain；增益乘在调用侧，便于 A/B 对照）。
//
// 算法：
//   1. 按 time_from_start_ms 归组：同一时刻的多个点 = 同一构型。
//   2. 每个时刻组装 q：target_ids 里每台电机——该时刻有路点就用它的 position；
//      没有就在它自己的相邻路点之间线性插值；只有单侧路点（早于首点/晚于末点）
//      时保持该侧值。缺拍**不跳过**该电机：重力矩多关节耦合，少算一个关节会让
//      别的关节的前馈也错。本分段没有任何路点的电机不合成位置（保留 ArmModel
//      内部构型，不臆造）。
//   3. 每个时刻调一次 gravity_torque_at，把该时刻各路点的 τ_g 加进对应 delta。
//
// 不跨分段边界：调用单位是一次 MotorTrajectory（= 一个分段），相邻分段的时刻
// 不互相插值——分段端点因此可能取到保持值而不是跨段插值，这是有意的取舍。
//
// 未知 motor_id / 模型外关节名只记入诊断（非空即表示那几台没有前馈），
// 不是整帧失败：调用方照常把帧发出去。
TrajectoryFeedforward trajectory_gravity_feedforward(
    const ArmModel & model, const MotorJointMap & joint_map,
    const std::vector<std::uint16_t> & target_ids,
    const std::vector<TrajectoryPoint> & points);

}  // namespace robot_dynamics
