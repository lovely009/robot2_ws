cmake_minimum_required(VERSION 3.8)
project(motor_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(motor_drive REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(robot_transmission REQUIRED)
find_package(robot_description REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/canfd_system.cpp
  src/canfd_logic.cpp
  src/motor_config.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} motor_drive::motor_drive yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle robot_transmission)

# The legacy direct-CAN implementation remains testable as a library, but is
# deliberately not exported as a loadable ros2_control hardware plugin. Real
# motors are owned exclusively by the micro-ROS MCU.
# An empty resource also shadows an older installed plugin in an underlay.
ament_index_register_resource("hardware_interface__pluginlib__plugin" CONTENT "")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME}
  PATTERN "motor.yaml" EXCLUDE)
# Backward-compatible path, installed from the single model-library source.
install(FILES "${robot_description_DIR}/../config/motor.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# 固件 ↔ ROS 标准话题适配桥（节点入口；转换/校验在
# motor_hardware_interface/topic_bridge.py，由单测共用）。
install(PROGRAMS scripts/firmware_topic_bridge.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface motor_drive pluginlib rclcpp rclcpp_lifecycle)

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
  ament_add_pytest_test(test_hardware_launch test/test_hardware_launch.py)
  ament_add_pytest_test(test_standard_trajectory_launch test/test_standard_trajectory_launch.py
    TIMEOUT 90
    APPEND_ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=194
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros2-ecosystem-tests")
  ament_add_pytest_test(test_xacro_consistency test/test_xacro_consistency.py)
  ament_add_pytest_test(test_topic_bridge test/test_topic_bridge.py)
  ament_add_pytest_test(test_control_state test/test_control_state.py)
  ament_add_pytest_test(test_transmission_bridge test/test_transmission_bridge.py)
endif()

ament_python_install_package(motor_hardware_interface)
ament_package()
