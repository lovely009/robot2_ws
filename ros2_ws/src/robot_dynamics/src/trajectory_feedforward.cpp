#include "robot_dynamics/trajectory_feedforward.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <utility>

namespace robot_dynamics {
namespace {

// 某台电机的路点序列（按时刻升序）。
using Samples = std::vector<std::pair<std::uint32_t, double>>;

// 序列外的时刻保持最近端点值（单侧无邻居可插值）；序列内线性插值。
double position_at(const Samples & samples, std::uint32_t time_ms) {
  if (time_ms <= samples.front().first) {
    return samples.front().second;
  }
  if (time_ms >= samples.back().first) {
    return samples.back().second;
  }
  const auto upper = std::upper_bound(
      samples.begin(), samples.end(), time_ms,
      [](std::uint32_t time, const std::pair<std::uint32_t, double> & sample) {
        return time < sample.first;
      });
  const auto lower = upper - 1;
  const double span = static_cast<double>(upper->first - lower->first);
  if (span <= 0.0) {  // 同刻重复路点（契约外）：不除零，退回左端值
    return lower->second;
  }
  return lower->second + (upper->second - lower->second) *
                             (static_cast<double>(time_ms - lower->first) / span);
}

}  // namespace

TrajectoryFeedforward trajectory_gravity_feedforward(
    const ArmModel & model, const MotorJointMap & joint_map,
    const std::vector<std::uint16_t> & target_ids,
    const std::vector<TrajectoryPoint> & points) {
  TrajectoryFeedforward result;
  result.effort_delta_nm.assign(points.size(), 0.0);

  // 只把落得回模型可控关节的电机纳入构型；其余进诊断。
  struct Series {
    std::string joint_name;
    std::size_t torque_index{0};  // 在 model.joint_names() 里的位次
    Samples samples;
  };
  const std::vector<std::string> & joint_names = model.joint_names();
  std::vector<Series> series;
  std::map<std::uint16_t, std::size_t> series_of_motor;
  for (const std::uint16_t motor_id : target_ids) {
    if (series_of_motor.count(motor_id) != 0) {
      continue;  // target_ids 重复项：契约禁止，按首次出现处理
    }
    const auto declared = joint_map.find(motor_id);
    if (declared == joint_map.end()) {
      result.unmapped_motor_ids.push_back(motor_id);
      continue;
    }
    const auto slot =
        std::find(joint_names.begin(), joint_names.end(), declared->second);
    if (slot == joint_names.end()) {
      result.ignored_joint_names.push_back(declared->second);
      continue;
    }
    series_of_motor.emplace(motor_id, series.size());
    series.push_back({declared->second,
                      static_cast<std::size_t>(slot - joint_names.begin()), {}});
  }

  // 归组：路点 → 所属电机序列（顺带收样本）；时刻 → (路点下标, 电机序列下标)。
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, std::size_t>>>
      points_at_time;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto motor = series_of_motor.find(points[i].motor_id);
    if (motor == series_of_motor.end()) {
      continue;  // 未知电机：本点没有前馈
    }
    series[motor->second].samples.emplace_back(points[i].time_from_start_ms,
                                               points[i].position_rad);
    points_at_time[points[i].time_from_start_ms].emplace_back(i,
                                                              motor->second);
  }
  for (Series & s : series) {
    // 契约已按时刻升序；乱序帧不靠运气，插值前排序。
    std::sort(s.samples.begin(), s.samples.end());
  }

  std::vector<std::string> eval_names;
  std::vector<double> eval_positions;
  for (const auto & [time_ms, indices] : points_at_time) {
    eval_names.clear();
    eval_positions.clear();
    for (const Series & s : series) {
      if (s.samples.empty()) {
        continue;  // 本分段无该电机路点：保留 ArmModel 内部构型
      }
      eval_names.push_back(s.joint_name);
      eval_positions.push_back(position_at(s.samples, time_ms));
    }
    const std::vector<double> torque =
        model.gravity_torque_at(eval_names, eval_positions).torque_nm;
    for (const auto & [point_index, series_index] : indices) {
      result.effort_delta_nm[point_index] += torque[series[series_index].torque_index];
    }
  }
  return result;
}

}  // namespace robot_dynamics
