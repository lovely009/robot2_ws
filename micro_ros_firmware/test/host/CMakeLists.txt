cmake_minimum_required(VERSION 3.16)
project(firmware_host_tests LANGUAGES C CXX)

enable_testing()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

add_executable(test_motor_protocol
  test_motor_protocol.cpp
  ../../lib/motor_protocol/src/motor_protocol.c)

target_include_directories(test_motor_protocol PRIVATE
  ../../lib/motor_protocol/include)

target_link_libraries(test_motor_protocol PRIVATE m)

add_test(NAME motor_protocol COMMAND test_motor_protocol)
