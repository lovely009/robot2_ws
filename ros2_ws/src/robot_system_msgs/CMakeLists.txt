cmake_minimum_required(VERSION 3.8)
project(robot_system_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_bridge_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/CalibrationState.msg"
  "msg/LeasedCommand.msg"
  "msg/LeasedPolicy.msg"
  "msg/LeasedTrajectory.msg"
  "srv/AcquireControl.srv"
  "srv/RenewControl.srv"
  "srv/ReleaseControl.srv"
  "srv/SetArmed.srv"
  "srv/AuthorizeMotion.srv"
  DEPENDENCIES std_msgs can_bridge_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contract test/test_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime std_msgs can_bridge_msgs)
ament_package()
