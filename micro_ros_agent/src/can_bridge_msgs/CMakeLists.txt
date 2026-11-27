cmake_minimum_required(VERSION 3.5)
project(can_bridge_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  # 原生领域消息与聚合状态（Schema 6，37 台固定上限 + std_msgs/Header）
  "msg/MotorCommandTarget.msg"
  "msg/MotorCommandArray.msg"
  "msg/MotorMotionStateTarget.msg"
  "msg/MotorMotionStateArray.msg"
  "msg/MotorControlState.msg"
  "msg/MotorStatusTarget.msg"
  "msg/MotorTopologyEntry.msg"
  "msg/DeviceParamEntry.msg"
  "msg/MotorPolicyTarget.msg"
  "msg/ParamSetTarget.msg"
  "msg/MotorDiagnostics.msg"
  "msg/MotorEvent.msg"
  "msg/SystemCommand.msg"
  "msg/SystemStatus.msg"
  "msg/ConfigRequest.msg"
  "msg/ConfigResult.msg"
  "msg/MotorSystemState.msg"
  "msg/MotorDeviceInfo.msg"
  # 原生服务接口
  "srv/ExecuteConfig.srv"
  "srv/SubmitPolicy.srv"
  "srv/GetDeviceInfo.srv"
  DEPENDENCIES std_msgs
)

ament_export_libraries()

ament_package()
