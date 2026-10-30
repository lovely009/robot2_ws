cmake_minimum_required(VERSION 3.8)
project(motor_drive)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/motor_driver.cpp
  src/motor_protocol.cpp
  src/protocol.cpp
  src/serial_slcan_transport.cpp
  src/slcan.cpp
  src/socketcan.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
  ament_add_gtest(test_motor_protocol test/test_motor_protocol.cpp)
  target_link_libraries(test_motor_protocol ${PROJECT_NAME})
  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver ${PROJECT_NAME})
  ament_add_gtest(test_slcan test/test_slcan.cpp)
  target_link_libraries(test_slcan ${PROJECT_NAME})
  find_package(Threads REQUIRED)
  ament_add_gtest(test_serial_slcan_transport test/test_serial_slcan_transport.cpp)
  target_link_libraries(test_serial_slcan_transport ${PROJECT_NAME} Threads::Threads)
  ament_add_pytest_test(test_motor_probe test/test_motor_probe.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
