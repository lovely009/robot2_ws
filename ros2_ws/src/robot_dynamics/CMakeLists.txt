cmake_minimum_required(VERSION 3.8)
project(robot_dynamics)

find_package(ament_cmake REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

# 计算库：只依赖 Pinocchio，不依赖 ROS。轨迹前馈的逐路点算法也在这里（纯函数，
# 不碰 ROS 消息类型），单测因此不需要起节点。
add_library(${PROJECT_NAME}
  src/arm_model.cpp
  src/trajectory_feedforward.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_controller SHARED src/gravity_compensation_controller.cpp)
target_compile_features(${PROJECT_NAME}_controller PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_controller
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle rcl_interfaces realtime_tools)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# ROS 层节点库：GravityTorqueNode 与 GravityFeedforwardOverlayNode 供独立可执行与
# 测试链接；可执行目标名与本库刻意区分，避免 CMake 目标重名。
add_library(${PROJECT_NAME}_ros
  src/ros/gravity_torque_node.cpp
  src/ros/gravity_feedforward_overlay_node.cpp)
target_compile_features(${PROJECT_NAME}_ros PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_ros PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_ros
  can_bridge_msgs rclcpp sensor_msgs)

add_executable(gravity_torque_node
  src/ros/gravity_torque_node_main.cpp)
target_compile_features(gravity_torque_node PRIVATE cxx_std_17)
target_compile_options(gravity_torque_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gravity_torque_node ${PROJECT_NAME}_ros)

add_executable(gravity_feedforward_overlay
  src/ros/gravity_feedforward_overlay_main.cpp)
target_compile_features(gravity_feedforward_overlay PRIVATE cxx_std_17)
target_compile_options(gravity_feedforward_overlay PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gravity_feedforward_overlay ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros ${PROJECT_NAME}_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gravity_torque_node gravity_feedforward_overlay
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(can_bridge_msgs controller_interface hardware_interface pinocchio pluginlib
  rclcpp rclcpp_lifecycle rcl_interfaces realtime_tools sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gravity_compensation_controller test/test_gravity_compensation_controller.cpp)
  target_compile_features(test_gravity_compensation_controller PRIVATE cxx_std_17)
  target_compile_definitions(test_gravity_compensation_controller PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_gravity_compensation_controller ${PROJECT_NAME}_controller)

  ament_add_gtest(test_pinocchio_dependency test/test_pinocchio_dependency.cpp)
  target_compile_features(test_pinocchio_dependency PRIVATE cxx_std_17)
  target_compile_options(test_pinocchio_dependency PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_pinocchio_dependency ${PROJECT_NAME})

  ament_add_gtest(test_arm_model test/test_arm_model.cpp)
  target_compile_features(test_arm_model PRIVATE cxx_std_17)
  target_compile_options(test_arm_model PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_arm_model PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_arm_model ${PROJECT_NAME})

  ament_add_gtest(test_gravity_torque_node test/test_gravity_torque_node.cpp)
  target_compile_features(test_gravity_torque_node PRIVATE cxx_std_17)
  target_compile_options(test_gravity_torque_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_gravity_torque_node PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_gravity_torque_node ${PROJECT_NAME}_ros)
  ament_target_dependencies(test_gravity_torque_node rclcpp sensor_msgs)

  ament_add_gtest(test_trajectory_feedforward
    test/test_trajectory_feedforward.cpp)
  target_compile_features(test_trajectory_feedforward PRIVATE cxx_std_17)
  target_compile_options(test_trajectory_feedforward PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_trajectory_feedforward PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_trajectory_feedforward ${PROJECT_NAME})

  ament_add_gtest(test_gravity_feedforward_overlay
    test/test_gravity_feedforward_overlay.cpp)
  target_compile_features(test_gravity_feedforward_overlay PRIVATE cxx_std_17)
  target_compile_options(test_gravity_feedforward_overlay PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_gravity_feedforward_overlay PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_gravity_feedforward_overlay ${PROJECT_NAME}_ros)
  ament_target_dependencies(test_gravity_feedforward_overlay
    can_bridge_msgs rclcpp sensor_msgs)
endif()

ament_package()
