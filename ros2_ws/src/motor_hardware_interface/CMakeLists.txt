cmake_minimum_required(VERSION 3.8)
project(motor_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_transmission REQUIRED)
find_package(robot_description REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/micro_ros_system.cpp
  src/joint_follow_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  can_bridge_msgs hardware_interface controller_interface realtime_tools sensor_msgs pluginlib rclcpp rclcpp_lifecycle robot_transmission std_msgs)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "motor.yaml" EXCLUDE)
install(FILES "${robot_description_DIR}/../config/motor.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES hardware_plugins.xml controller_plugins.xml DESTINATION share/${PROJECT_NAME})

# 固件 ↔ ROS 标准话题适配桥（节点入口；转换/校验在
# motor_hardware_interface/topic_bridge.py，由单测共用）。
install(PROGRAMS scripts/firmware_topic_bridge.py scripts/workbench_control.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(can_bridge_msgs controller_interface realtime_tools sensor_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs robot_transmission)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_position_profile test/test_position_profile.cpp)
  target_link_libraries(test_position_profile ${PROJECT_NAME})
  ament_add_gtest(test_joint_follow_controller test/test_joint_follow_controller.cpp)
  target_link_libraries(test_joint_follow_controller ${PROJECT_NAME})
  ament_add_gtest(test_micro_ros_system test/test_micro_ros_system.cpp)
  target_link_libraries(test_micro_ros_system ${PROJECT_NAME})
  ament_add_pytest_test(test_mock_control_launch test/test_mock_control_launch.py)
  ament_add_pytest_test(test_host_trajectory_launch test/test_host_trajectory_launch.py)
  ament_add_pytest_test(test_topic_bridge test/test_topic_bridge.py)
  ament_add_pytest_test(test_control_state test/test_control_state.py)
  ament_add_pytest_test(test_control_subset test/test_control_subset.py)
  ament_add_pytest_test(test_workbench_control test/test_workbench_control.py)
  ament_add_pytest_test(test_transmission_bridge test/test_transmission_bridge.py)
  ament_add_pytest_test(test_joint_limits_parameters test/test_joint_limits_parameters.py)
endif()

ament_python_install_package(motor_hardware_interface)
ament_package()
