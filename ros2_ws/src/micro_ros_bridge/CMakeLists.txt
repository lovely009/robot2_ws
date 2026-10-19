cmake_minimum_required(VERSION 3.8)
project(micro_ros_bridge)

find_package(ament_cmake REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# codec：header-only、纯函数、不依赖 rclcpp（只依赖消息类型）。
# 独立成 interface 目标，将来 ros2_control 的 SystemInterface 复用同一份换算。
add_library(${PROJECT_NAME}_codec INTERFACE)
target_compile_features(${PROJECT_NAME}_codec INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME}_codec INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_codec INTERFACE can_bridge_msgs)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(can_bridge_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_contract test/test_contract.cpp)
  target_compile_features(test_contract PRIVATE cxx_std_17)
  target_compile_options(test_contract PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_contract ${PROJECT_NAME}_codec)
endif()

ament_package()
