cmake_minimum_required(VERSION 3.8)
project(upper_body_canfd_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  upper_body_canfd_hardware
  SHARED
  src/rp_mit.cpp
  src/wrist_2530.cpp
  src/canfd_socket.cpp
  src/canfd_motor_system.cpp
)

target_include_directories(
  upper_body_canfd_hardware
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(
  upper_body_canfd_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rp_mit test/test_rp_mit.cpp)
  if(TARGET test_rp_mit)
    target_link_libraries(test_rp_mit upper_body_canfd_hardware)
  endif()

  ament_add_gtest(test_wrist_2530 test/test_wrist_2530.cpp)
  if(TARGET test_wrist_2530)
    target_link_libraries(test_wrist_2530 upper_body_canfd_hardware)
  endif()

  ament_add_gtest(test_canfd_socket test/test_canfd_socket.cpp)
  if(TARGET test_canfd_socket)
    target_link_libraries(test_canfd_socket upper_body_canfd_hardware)
  endif()

  ament_add_gtest(test_canfd_motor_system test/test_canfd_motor_system.cpp)
  if(TARGET test_canfd_motor_system)
    target_link_libraries(test_canfd_motor_system upper_body_canfd_hardware)
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS upper_body_canfd_hardware
  EXPORT export_upper_body_canfd_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES upper_body_canfd_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/fake_slaves.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface upper_body_canfd_hardware.xml)

ament_export_include_directories(include)
ament_export_libraries(upper_body_canfd_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
