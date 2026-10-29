cmake_minimum_required(VERSION 3.8)
project(motor_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(motor_drive REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(robot_transmission REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/canfd_system.cpp
  src/canfd_logic.cpp
  src/micro_ros_system.cpp
  src/motor_config.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} motor_drive::motor_drive yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  can_bridge_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle robot_transmission std_msgs)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(FILES hardware_plugins.xml DESTINATION share/${PROJECT_NAME})

# 固件 ↔ ROS 标准话题适配桥（节点入口；转换/校验在
# motor_hardware_interface/topic_bridge.py，由单测共用）。
install(PROGRAMS scripts/firmware_topic_bridge.py scripts/workbench_control.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(can_bridge_msgs hardware_interface motor_drive pluginlib rclcpp rclcpp_lifecycle std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_motor_config test/test_motor_config.cpp)
  target_compile_definitions(test_motor_config PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_motor_config ${PROJECT_NAME})
  ament_add_gtest(test_canfd_logic test/test_canfd_logic.cpp)
  target_link_libraries(test_canfd_logic ${PROJECT_NAME})
  ament_add_gtest(test_transmission_system test/test_transmission_system.cpp)
  target_compile_definitions(test_transmission_system PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_transmission_system ${PROJECT_NAME})
  ament_add_gtest(test_micro_ros_system test/test_micro_ros_system.cpp)
  target_link_libraries(test_micro_ros_system ${PROJECT_NAME})
  ament_add_pytest_test(test_hardware_launch test/test_hardware_launch.py)
  ament_add_pytest_test(test_host_trajectory_launch test/test_host_trajectory_launch.py)
  ament_add_pytest_test(test_xacro_consistency test/test_xacro_consistency.py)
  ament_add_pytest_test(test_topic_bridge test/test_topic_bridge.py)
  ament_add_pytest_test(test_control_state test/test_control_state.py)
  ament_add_pytest_test(test_workbench_control test/test_workbench_control.py)
  ament_add_pytest_test(test_transmission_bridge test/test_transmission_bridge.py)
endif()

ament_python_install_package(motor_hardware_interface)
ament_package()
