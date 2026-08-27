cmake_minimum_required(VERSION 3.8)
project(motor_sim)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(motor_protocol
  src/protocol/motor_protocol.cpp
  src/protocol/rp_protocol.cpp
  src/protocol/vaxor_protocol.cpp
  src/protocol/cerb_protocol.cpp)
target_compile_features(motor_protocol PUBLIC cxx_std_17)
target_compile_options(motor_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(motor_transport
  src/transport/frame_record.cpp
  src/transport/serial_transport.cpp
  src/transport/socket_can_transport.cpp)
target_compile_features(motor_transport PUBLIC cxx_std_17)
target_compile_options(motor_transport PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_transport PUBLIC motor_protocol Threads::Threads)

add_library(motor_sim_core
  src/core/config.cpp
  src/core/drive_lifecycle.cpp
  src/core/frame_router.cpp
  src/core/transmission.cpp
  src/core/plant_model.cpp
  src/core/safety_monitor.cpp)
target_compile_features(motor_sim_core PUBLIC cxx_std_17)
target_compile_options(motor_sim_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_sim_core PUBLIC motor_protocol yaml-cpp)

add_library(motor_sim_node
  src/ros/motor_sim_node.cpp)
target_compile_features(motor_sim_node PUBLIC cxx_std_17)
target_compile_options(motor_sim_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_sim_node motor_sim_core motor_transport)
ament_target_dependencies(motor_sim_node
  diagnostic_msgs rclcpp sensor_msgs std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS motor_protocol motor_transport motor_sim_core motor_sim_node
  EXPORT export_motor_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_motor_sim HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_package_surface test/test_package_surface.cpp)
  target_compile_features(test_package_surface PUBLIC cxx_std_17)
  target_include_directories(test_package_surface PRIVATE include)
  target_link_libraries(test_package_surface motor_protocol motor_transport)

  ament_add_gtest(test_motor_protocol test/test_motor_protocol.cpp)
  target_compile_features(test_motor_protocol PUBLIC cxx_std_17)
  target_compile_options(test_motor_protocol PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_motor_protocol motor_protocol)

  ament_add_gtest(test_cerb_protocol test/test_cerb_protocol.cpp)
  target_compile_features(test_cerb_protocol PUBLIC cxx_std_17)
  target_compile_options(test_cerb_protocol PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_cerb_protocol motor_protocol)

  ament_add_gtest(test_frame_record test/test_frame_record.cpp)
  target_compile_features(test_frame_record PUBLIC cxx_std_17)
  target_compile_options(test_frame_record PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_frame_record motor_transport)

  ament_add_gtest(test_serial_transport test/test_serial_transport.cpp)
  target_compile_features(test_serial_transport PUBLIC cxx_std_17)
  target_compile_options(test_serial_transport PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_serial_transport motor_transport util)

  ament_add_gtest(test_socket_can_transport test/test_socket_can_transport.cpp)
  target_compile_features(test_socket_can_transport PUBLIC cxx_std_17)
  target_compile_options(test_socket_can_transport PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_socket_can_transport motor_transport)

  ament_add_gtest(test_config test/test_config.cpp)
  target_compile_features(test_config PUBLIC cxx_std_17)
  target_compile_options(test_config PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_config PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_config motor_sim_core)

  foreach(CORE_TEST transmission plant_model safety_monitor drive_lifecycle)
    ament_add_gtest(test_${CORE_TEST} test/test_${CORE_TEST}.cpp)
    target_compile_features(test_${CORE_TEST} PUBLIC cxx_std_17)
    target_compile_options(test_${CORE_TEST} PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(test_${CORE_TEST} motor_sim_core)
  endforeach()

  ament_add_gtest(test_frame_router test/test_frame_router.cpp)
  target_compile_features(test_frame_router PUBLIC cxx_std_17)
  target_compile_options(test_frame_router PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_frame_router PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_frame_router motor_sim_core)

  ament_add_gtest(test_motor_sim_node test/test_motor_sim_node.cpp)
  target_compile_features(test_motor_sim_node PUBLIC cxx_std_17)
  target_compile_options(test_motor_sim_node PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_motor_sim_node motor_sim_node util)
  ament_target_dependencies(test_motor_sim_node rclcpp)
endif()

ament_package()
