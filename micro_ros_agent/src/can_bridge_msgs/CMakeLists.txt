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
  DEPENDENCIES std_msgs
)

ament_export_libraries()

ament_package()
