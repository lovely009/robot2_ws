cmake_minimum_required(VERSION 3.16)
project(firmware_host_tests LANGUAGES C CXX)

enable_testing()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# 原则：测试直接编译固件真实 .c，无桩无复制。固件源码按库组织成 OBJECT
# 库单点维护源文件列表，新增内核 .c 只改本文件一处；include 目录随库
# PUBLIC 传播，测试目标只声明依赖哪个库。
add_library(fw_motor_protocol OBJECT
  ../../lib/motor_protocol/src/motor_protocol.c)
target_include_directories(fw_motor_protocol PUBLIC
  ../../lib/motor_protocol/include)

# 协议适配器 + 型号目录（profile 被适配器与内核两侧共用，归此库单点编译）
add_library(fw_adapters OBJECT
  ../../lib/motor_protocol/src/vaxor_adapter.c
  ../../lib/motor_protocol/src/cerb_adapter.c
  ../../lib/motor_control/src/motor_control_profile.c)
target_include_directories(fw_adapters PUBLIC
  ../../lib/motor_protocol/include
  ../../lib/motor_control/include)
target_link_libraries(fw_adapters PUBLIC fw_motor_protocol)

# 电机运行时内核（白盒测试另需 -I 内核 src，见 test_motor_control）
add_library(fw_motor_control OBJECT
  ../../lib/motor_control/src/motor_control.c
  ../../lib/motor_control/src/motor_control_discovery.c
  ../../lib/motor_control/src/motor_control_config.c
  ../../lib/motor_control/src/motor_control_params.c)
target_include_directories(fw_motor_control PUBLIC
  ../../lib/motor_control/include)
target_link_libraries(fw_motor_control PUBLIC fw_adapters)

add_library(fw_mailbox OBJECT
  ../../lib/control_mailbox/src/control_command_mailbox.c
  ../../lib/control_mailbox/src/control_mailbox.c)
target_include_directories(fw_mailbox PUBLIC
  ../../lib/control_mailbox/include)

add_library(fw_can_rx_ring OBJECT
  ../../lib/fdcan/src/can_rx_ring.c)
target_include_directories(fw_can_rx_ring PUBLIC ../../lib/fdcan/include)

add_library(fw_can_tx_scheduler OBJECT
  ../../lib/fdcan/src/can_tx_scheduler.c)
target_include_directories(fw_can_tx_scheduler PUBLIC ../../lib/fdcan/include)

add_library(fw_imu_snapshot OBJECT
  ../../lib/imu/src/imu_snapshot.c)
target_include_directories(fw_imu_snapshot PUBLIC ../../lib/imu/include)

add_library(fw_bmi088_status OBJECT
  ../../lib/imu/src/bmi088_transport_status.c)
target_include_directories(fw_bmi088_status PUBLIC ../../lib/imu/include)

# imu.c 与 MahonyAHRS 未调用的传输函数靠链接器裁剪，另带 status 提供真实符号
add_library(fw_imu_core OBJECT
  ../../lib/imu/src/imu.c
  ../../lib/imu/src/MahonyAHRS.c)
target_include_directories(fw_imu_core PUBLIC ../../lib/imu/include)
target_link_libraries(fw_imu_core PUBLIC fw_bmi088_status)

add_library(fw_param_store OBJECT
  ../../lib/param_store/src/param_store_blob.c
  ../../lib/param_store/src/param_name.c)
target_include_directories(fw_param_store PUBLIC
  ../../lib/param_store/include)

add_library(fw_w25_geometry OBJECT
  ../../lib/w25q64/src/w25q64_geometry.c)
target_include_directories(fw_w25_geometry PUBLIC ../../lib/w25q64/include)

# 声明一个 host 测试目标：SOURCES 测试自身源文件，OBLIBS 依赖的固件 OBJECT
# 库，INCLUDES 额外包含目录，DEFINES 编译宏（如 led 的 HOST_TEST）。
# 注意 CMake 3.16 不传递 OBJECT 库的目标文件（只传播 include），
# OBLIBS 必须列出链接所需的全部库（含间接依赖）。
function(fw_host_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;OBLIBS;INCLUDES;DEFINES" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  if(ARG_OBLIBS)
    target_link_libraries(${name} PRIVATE ${ARG_OBLIBS})
  endif()
  if(ARG_INCLUDES)
    target_include_directories(${name} PRIVATE ${ARG_INCLUDES})
  endif()
  if(ARG_DEFINES)
    target_compile_definitions(${name} PRIVATE ${ARG_DEFINES})
  endif()
  target_link_libraries(${name} PRIVATE m)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

fw_host_test(test_motor_protocol
  SOURCES test_motor_protocol.cpp
  OBLIBS fw_motor_protocol)

fw_host_test(test_motor_protocol_adapter
  SOURCES test_motor_protocol_adapter.cpp
  OBLIBS fw_adapters fw_motor_protocol)

fw_host_test(test_motor_control
  SOURCES test_motor_control.cpp
  OBLIBS fw_motor_control fw_adapters fw_motor_protocol)

# config kind 枚举契约 static_assert 需要协议枚举定义
fw_host_test(test_control_mailbox
  SOURCES test_control_mailbox.cpp
  OBLIBS fw_mailbox
  INCLUDES ../../lib/motor_protocol/include)

fw_host_test(test_can_rx_ring
  SOURCES test_can_rx_ring.cpp
  OBLIBS fw_can_rx_ring)

fw_host_test(test_can_tx_scheduler
  SOURCES test_can_tx_scheduler.cpp
  OBLIBS fw_can_tx_scheduler)

fw_host_test(test_imu_snapshot
  SOURCES test_imu_snapshot.cpp
  OBLIBS fw_imu_snapshot)

fw_host_test(test_imu_units
  SOURCES test_imu_units.cpp
  OBLIBS fw_imu_core fw_bmi088_status)

fw_host_test(test_bmi088_transport_status
  SOURCES test_bmi088_transport_status.cpp
  OBLIBS fw_bmi088_status)

fw_host_test(test_bmi088_register_config
  SOURCES test_bmi088_register_config.cpp
  INCLUDES ../../lib/imu/include)

fw_host_test(test_param_store
  SOURCES test_param_store.cpp
  OBLIBS fw_param_store)

fw_host_test(test_w25q64_geometry
  SOURCES test_w25q64_geometry.cpp
  OBLIBS fw_w25_geometry)

# led_task.cpp 用 HOST_TEST 跳过 Arduino/FreeRTOS 段，host 只测颜色策略
fw_host_test(test_led_status
  SOURCES test_led_status.cpp ../../src/led_task.cpp
  INCLUDES ../../include
  DEFINES HOST_TEST=1)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_test(
  NAME fdcan_motor_sim
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fdcan_motor_sim.py)
