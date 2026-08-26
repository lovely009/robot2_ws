cmake_minimum_required(VERSION 3.8)
project(motor_sim)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(motor_protocol
  src/protocol/motor_protocol.cpp
  src/protocol/rp_protocol.cpp
  src/protocol/vaxor_protocol.cpp)
target_compile_features(motor_protocol PUBLIC cxx_std_17)
target_compile_options(motor_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(motor_transport
  src/transport/frame_record.cpp
  src/transport/serial_transport.cpp)
target_compile_features(motor_transport PUBLIC cxx_std_17)
target_compile_options(motor_transport PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_transport PUBLIC motor_protocol Threads::Threads)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS motor_protocol motor_transport
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

  ament_add_gtest(test_frame_record test/test_frame_record.cpp)
  target_compile_features(test_frame_record PUBLIC cxx_std_17)
  target_compile_options(test_frame_record PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_frame_record motor_transport)

  ament_add_gtest(test_serial_transport test/test_serial_transport.cpp)
  target_compile_features(test_serial_transport PUBLIC cxx_std_17)
  target_compile_options(test_serial_transport PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_serial_transport motor_transport util)
endif()

ament_package()
