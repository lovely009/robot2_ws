cmake_minimum_required(VERSION 3.5)
project(can_bridge_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  # ID 化有界电机消息（ARD-0007 §15-20）
  "msg/MotorCommandTarget.msg"
  "msg/MotorCommandArray.msg"
  "msg/MotorMotionStateTarget.msg"
  "msg/MotorMotionStateArray.msg"
  "msg/MotorStatusTarget.msg"
  "msg/MotorStatusArray.msg"
  "msg/MotorTopologyEntry.msg"
  "msg/MotorTopology.msg"
  "msg/MotorEvent.msg"
  # 下行多点轨迹（MotorCommandArray 的时序补充通道，非替代）
  "msg/MotorTrajectoryPoint.msg"
  "msg/MotorTrajectory.msg"
  # 下行逐电机策略（动态量 enable/kp/kd；稀疏语义，非完整快照）
  "msg/MotorPolicyTarget.msg"
  "msg/MotorPolicyArray.msg"
  # 上行：轨迹执行状态与固件级诊断
  "msg/MotorTrajectoryState.msg"
  "msg/MotorDiagnostics.msg"
  # 上行：参数核对双行快照（固件记录 vs 驱动板回读，低频 reliable）
  "msg/DeviceParamEntry.msg"
  "msg/DeviceParamReport.msg"
  # 下行：参数写（稀疏）与系统命令——取代已退役的 rclc 参数服务器
  "msg/ParamSetTarget.msg"
  "msg/ParamSetArray.msg"
  "msg/SystemCommand.msg"
  # 上行：系统状态（错误电平 + 四个完成序号，5 Hz best-effort）
  "msg/SystemStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_libraries()

ament_package()
