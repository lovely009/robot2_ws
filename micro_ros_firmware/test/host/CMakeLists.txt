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

add_executable(test_motor_protocol_adapter
  test_motor_protocol_adapter.cpp
  ../../lib/motor_protocol/src/motor_protocol.c
  ../../lib/motor_protocol/src/vaxor_adapter.c
  ../../lib/motor_protocol/src/creb_adapter.c
  ../../lib/motor_control/src/motor_control_profile.c)

target_include_directories(test_motor_protocol_adapter PRIVATE
  ../../lib/motor_protocol/include
  ../../lib/motor_control/include)

target_link_libraries(test_motor_protocol_adapter PRIVATE m)

add_test(NAME motor_protocol_adapter COMMAND test_motor_protocol_adapter)

add_executable(test_motor_control
  test_motor_control.cpp
  ../../lib/motor_control/src/motor_control.c
  ../../lib/motor_control/src/motor_control_discovery.c
  ../../lib/motor_control/src/motor_control_config.c
  ../../lib/motor_control/src/motor_control_profile.c
  ../../lib/motor_protocol/src/motor_protocol.c
  ../../lib/motor_protocol/src/vaxor_adapter.c
  ../../lib/motor_protocol/src/creb_adapter.c)

target_include_directories(test_motor_control PRIVATE
  ../../lib/motor_control/include
  ../../lib/motor_protocol/include)

target_link_libraries(test_motor_control PRIVATE m)

add_test(NAME motor_control COMMAND test_motor_control)

add_executable(test_control_mailbox
  test_control_mailbox.cpp
  ../../lib/control_mailbox/src/control_command_mailbox.c
  ../../lib/control_mailbox/src/control_mailbox.c)

target_include_directories(test_control_mailbox PRIVATE
  ../../lib/control_mailbox/include)

add_test(NAME control_mailbox COMMAND test_control_mailbox)

add_executable(test_can_rx_ring
  test_can_rx_ring.cpp
  ../../lib/fdcan/src/can_rx_ring.c)

target_include_directories(test_can_rx_ring PRIVATE
  ../../lib/fdcan/include)

add_test(NAME can_rx_ring COMMAND test_can_rx_ring)

add_executable(test_can_tx_scheduler
  test_can_tx_scheduler.cpp
  ../../lib/fdcan/src/can_tx_scheduler.c)

target_include_directories(test_can_tx_scheduler PRIVATE
  ../../lib/fdcan/include)

add_test(NAME can_tx_scheduler COMMAND test_can_tx_scheduler)

add_executable(test_imu_snapshot
  test_imu_snapshot.cpp
  ../../lib/imu/src/imu_snapshot.c)

target_include_directories(test_imu_snapshot PRIVATE
  ../../lib/imu/include)

add_test(NAME imu_snapshot COMMAND test_imu_snapshot)

add_executable(test_bmi088_transport_status
  test_bmi088_transport_status.cpp
  ../../lib/imu/src/bmi088_transport_status.c)

target_include_directories(test_bmi088_transport_status PRIVATE
  ../../lib/imu/include)

add_test(NAME bmi088_transport_status COMMAND test_bmi088_transport_status)

add_executable(test_w25q64_geometry
  test_w25q64_geometry.cpp
  ../../lib/w25q64/src/w25q64_geometry.c)

target_include_directories(test_w25q64_geometry PRIVATE
  ../../lib/w25q64/include)

add_test(NAME w25q64_geometry COMMAND test_w25q64_geometry)

add_executable(test_led_status
  test_led_status.cpp
  ../../src/led_task.cpp)

target_include_directories(test_led_status PRIVATE
  ../../include)

# led_task.cpp 用 HOST_TEST 跳过 Arduino/FreeRTOS 段，host 只测颜色策略
target_compile_definitions(test_led_status PRIVATE
  HOST_TEST=1)

add_test(NAME led_status COMMAND test_led_status)
