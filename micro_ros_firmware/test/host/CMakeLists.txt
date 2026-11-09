cmake_minimum_required(VERSION 3.16)
project(firmware_host_tests LANGUAGES C CXX)

enable_testing()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 应用层头按域归组，子目录显式列出；根目录保留全局配置头
include_directories(
  ../../include
  ../../include/board
  ../../include/micro_ros
  ../../include/motor
  ../../include/sensor
  ../../include/storage)

# 原则：测试直接编译固件真实 .c，无桩无复制。固件源码按库组织成 OBJECT
# 库单点维护源文件列表，新增内核 .c 只改本文件一处；include 目录随库
# PUBLIC 传播，测试目标只声明依赖哪个库。
add_library(fw_motor_protocol OBJECT
  ../../lib/motor/src/motor_protocol.c)
target_include_directories(fw_motor_protocol PUBLIC
  ../../lib/motor/include)

# 协议适配器 + 型号目录（profile 被适配器与内核两侧共用，归此库单点编译）
add_library(fw_adapters OBJECT
  ../../lib/motor/src/vaxor_adapter.c
  ../../lib/motor/src/cerb_adapter.c
  ../../lib/motor/src/motor_protocol_registry.c
  ../../lib/motor/src/motor_protocol_batch.c
  ../../lib/motor/src/motor_control_profile.c)
target_include_directories(fw_adapters PUBLIC
  ../../lib/motor/include
  ../../lib/motor/include)
target_link_libraries(fw_adapters PUBLIC fw_motor_protocol)

# 电机运行时内核（白盒测试另需 -I 内核 src，见 test_motor_control）
add_library(fw_motor_control OBJECT
  ../../lib/motor/src/motor_control.c
  ../../lib/motor/src/motor_control_discovery.c
  ../../lib/motor/src/motor_control_config.c
  ../../lib/motor/src/motor_control_params.c)
target_include_directories(fw_motor_control PUBLIC
  ../../lib/motor/include)
target_link_libraries(fw_motor_control PUBLIC fw_adapters)

# 会话同步纯逻辑（应用源码；host 可直编）
add_library(fw_session_sync OBJECT
  ../../src/micro_ros/session_sync.c)
target_include_directories(fw_session_sync PUBLIC
  ../../lib/control/include
  ../../lib/motor/include ../../lib/motor/include)
target_link_libraries(fw_session_sync PUBLIC fw_mailbox fw_motor_control)

add_library(fw_mailbox OBJECT
  ../../lib/control/src/control_command_mailbox.c
  ../../lib/control/src/control_trajectory_mailbox.c
  ../../lib/control/src/control_policy_mailbox.c
  ../../lib/control/src/control_mailbox.c)
target_include_directories(fw_mailbox PUBLIC
  ../../lib/control/include)

add_library(fw_can_rx_ring OBJECT
  ../../lib/board/src/can_rx_ring.c)
target_include_directories(fw_can_rx_ring PUBLIC ../../lib/board/include)

add_library(fw_can_tx_scheduler OBJECT
  ../../lib/board/src/can_tx_scheduler.c)
target_include_directories(fw_can_tx_scheduler PUBLIC ../../lib/board/include)

add_library(fw_motor_tx_router OBJECT
  ../../lib/motor/src/motor_tx_router.c)
target_include_directories(fw_motor_tx_router PUBLIC
  ../../lib/motor/include ../../lib/board/include
  ../../lib/motor/include)
target_link_libraries(fw_motor_tx_router PUBLIC
  fw_can_tx_scheduler fw_adapters fw_motor_protocol)

add_library(fw_imu_snapshot OBJECT
  ../../lib/board/src/imu_snapshot.c)
target_include_directories(fw_imu_snapshot PUBLIC ../../lib/board/include)

add_library(fw_bmi088_status OBJECT
  ../../lib/board/src/bmi088_transport_status.c)
target_include_directories(fw_bmi088_status PUBLIC ../../lib/board/include)

# imu.c 与 MahonyAHRS 未调用的传输函数靠链接器裁剪，另带 status 提供真实符号
add_library(fw_imu_core OBJECT
  ../../lib/board/src/imu.c
  ../../lib/board/src/MahonyAHRS.c)
target_include_directories(fw_imu_core PUBLIC ../../lib/board/include)
target_link_libraries(fw_imu_core PUBLIC fw_bmi088_status)

add_library(fw_param_store OBJECT
  ../../lib/storage/src/param_store_blob.c
  ../../lib/storage/src/param_field.c)

target_include_directories(fw_param_store PUBLIC
  ../../lib/storage/include)


add_library(fw_storage_projection OBJECT ../../lib/storage/src/storage_projection.c)
target_include_directories(fw_storage_projection PUBLIC
  ../../lib/storage/include ../../include ../../lib/storage/include
  ../../lib/motor/include ../../lib/motor/include)
target_link_libraries(fw_storage_projection PUBLIC fw_param_store)

# 定点编解码纯逻辑（lib/control；无 ROS 依赖，host 可直编）
add_library(fw_wire_codec OBJECT
  ../../lib/control/src/wire_codec.c)
target_include_directories(fw_wire_codec PUBLIC
  ../../lib/control/include)

add_library(fw_control_lease OBJECT ../../lib/control/src/control_lease.c)
target_include_directories(fw_control_lease PUBLIC ../../lib/control/include)

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

add_library(fw_info_page OBJECT ../../src/micro_ros/info_page.cpp)
target_include_directories(fw_info_page PUBLIC ../../include/micro_ros)
target_link_libraries(fw_info_page PUBLIC fw_motor_control fw_config_transaction)
fw_host_test(test_info_page SOURCES test_info_page.cpp OBLIBS fw_info_page)

add_library(fw_agent_protocol OBJECT
  ../../lib/control/src/agent_protocol.c
  ../../lib/control/src/agent_control_gate.c)
target_include_directories(fw_agent_protocol PUBLIC ../../lib/control/include)
target_link_libraries(fw_agent_protocol PUBLIC fw_control_lease fw_mailbox fw_motor_control fw_session_sync fw_wire_codec)
fw_host_test(test_agent_protocol SOURCES test_agent_protocol.cpp
  OBLIBS fw_agent_protocol fw_session_sync fw_mailbox fw_motor_control fw_adapters fw_motor_protocol fw_wire_codec fw_control_lease
  INCLUDES ../../lib/motor/src)
fw_host_test(test_agent_liveness SOURCES test_agent_liveness.cpp
  INCLUDES ../../lib/control/include)

fw_host_test(test_motor_protocol
  SOURCES test_motor_protocol.cpp
  OBLIBS fw_motor_protocol)

fw_host_test(test_control_lease
  SOURCES test_control_lease.cpp
  OBLIBS fw_control_lease fw_mailbox fw_motor_control fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/src)

fw_host_test(test_motor_protocol_adapter
  SOURCES test_motor_protocol_adapter.cpp
  OBLIBS fw_adapters fw_motor_protocol)

# 协议族隔离：认领集不相交、跨族真帧拒绝、绑定协议门禁。
fw_host_test(test_protocol_family_isolation
  SOURCES test_protocol_family_isolation.cpp
  OBLIBS fw_adapters fw_motor_protocol)

fw_host_test(test_motor_control
  SOURCES test_motor_control.cpp
  OBLIBS fw_motor_control fw_adapters fw_motor_protocol fw_config_transaction
  # 白盒：直读 ctx->entries 等内部布局（internal.h 不向 app 导出）
  INCLUDES ../../lib/motor/src)

# mailbox 与协议 kind 共享完整契约断言；枚举类型仍独立。
fw_host_test(test_control_mailbox
  SOURCES test_control_mailbox.cpp
  OBLIBS fw_mailbox
  INCLUDES ../../include ../../lib/motor/include)
find_package(Threads REQUIRED)
target_link_libraries(test_control_mailbox PRIVATE Threads::Threads)

fw_host_test(test_config_kind_contract
  SOURCES test_config_kind_contract.cpp
  OBLIBS fw_mailbox fw_motor_protocol
  INCLUDES ../../include ../../lib/motor/include)

fw_host_test(test_trajectory_mailbox
  SOURCES test_trajectory_mailbox.cpp
  OBLIBS fw_mailbox)

fw_host_test(test_policy_mailbox
  SOURCES test_policy_mailbox.cpp
  OBLIBS fw_mailbox)

fw_host_test(test_can_rx_ring
  SOURCES test_can_rx_ring.cpp
  OBLIBS fw_can_rx_ring)

fw_host_test(test_can_tx_scheduler
  SOURCES test_can_tx_scheduler.cpp
  OBLIBS fw_can_tx_scheduler)

fw_host_test(test_motor_tx_router
  SOURCES test_motor_tx_router.cpp
  OBLIBS fw_motor_tx_router fw_can_tx_scheduler fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/include)

# 跨层停机回归：真实内核、协议适配器、调度器及 CERB 聚合器共同执行。
fw_host_test(test_stop_tx
  SOURCES test_stop_tx.cpp
  OBLIBS fw_motor_control fw_motor_tx_router fw_can_tx_scheduler
    fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/src)

fw_host_test(test_37_axis_schedule
  SOURCES test_37_axis_schedule.cpp
  OBLIBS fw_motor_control fw_motor_tx_router fw_can_tx_scheduler
    fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/src)


add_library(fw_clock_health OBJECT ../../src/micro_ros/clock_health.c)
fw_host_test(test_clock_health SOURCES test_clock_health.cpp OBLIBS fw_clock_health)

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
  INCLUDES ../../lib/board/include)



# led_task.cpp 用 HOST_TEST 跳过 Arduino/FreeRTOS 段，host 只测颜色策略
fw_host_test(test_led_status
  SOURCES test_led_status.cpp ../../src/sensor/led_task.cpp
  INCLUDES ../../include
  DEFINES HOST_TEST=1)


# heater.cpp 用 HOST_TEST 跳过 Arduino/HardwareTimer 段，host 只测纯 PID
fw_host_test(test_heater_pid
  SOURCES test_heater_pid.cpp ../../src/sensor/heater.cpp
  INCLUDES ../../include
  DEFINES HOST_TEST=1)

# 白盒：直读内核 ctx 字段（-I 内核 src）
fw_host_test(test_session_sync
  SOURCES test_session_sync.cpp
  OBLIBS fw_session_sync fw_mailbox fw_motor_control fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/src)

fw_host_test(test_wire_codec
  SOURCES test_wire_codec.cpp
  OBLIBS fw_wire_codec)

fw_host_test(test_reliable_schedule SOURCES test_reliable_schedule.cpp)

# 直接编译线上 poll，ROS 端口由测试替身提供；源码改动时自动重新提取。
# shortcut: poll 目前位于文件末尾；布局改变时须更新提取边界。
set(_bridge_source "${CMAKE_CURRENT_SOURCE_DIR}/../../src/motor/motor_bridge.cpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_bridge_source}")
file(READ "${_bridge_source}" _bridge_text)
string(FIND "${_bridge_text}" "void motor_bridge_poll(void)" _poll_start)
if(_poll_start LESS 0)
  message(FATAL_ERROR "motor_bridge_poll entry point not found")
endif()
string(SUBSTRING "${_bridge_text}" ${_poll_start} -1 _poll_text)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/motor_bridge_poll.inc" "${_poll_text}")
fw_host_test(test_motor_bridge_schedule SOURCES test_motor_bridge_schedule.cpp
  INCLUDES "${CMAKE_CURRENT_BINARY_DIR}")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_test(
  NAME isolate_ros_environment
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_isolate_ros_environment.py)

add_test(
  NAME fdcan_motor_sim
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fdcan_motor_sim.py)

add_test(
  NAME verify_microros_repo_lock
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_verify_microros_repo_lock.py)

add_library(fw_trajectory_transaction OBJECT
  ../../lib/control/src/trajectory_transaction.cpp)
target_include_directories(fw_trajectory_transaction PUBLIC ../../lib/control/include)
target_link_libraries(fw_trajectory_transaction PUBLIC fw_motor_control fw_mailbox)
fw_host_test(test_trajectory_transaction SOURCES test_trajectory_transaction.cpp
  OBLIBS fw_trajectory_transaction fw_motor_control fw_adapters fw_motor_protocol
  INCLUDES ../../lib/motor/src)

add_library(fw_config_transaction OBJECT
  ../../lib/control/src/config_transaction.cpp)
target_include_directories(fw_config_transaction PUBLIC ../../lib/control/include)
fw_host_test(test_config_transaction
  SOURCES test_config_transaction.cpp
  OBLIBS fw_config_transaction)

add_library(fw_serial_write_budget OBJECT ../../src/micro_ros/serial_write_budget.c)
fw_host_test(test_serial_write_budget SOURCES test_serial_write_budget.cpp OBLIBS fw_serial_write_budget)

# 按固件锁定源码构建的宿主 XRCE 库通过 CMAKE_PREFIX_PATH 指定。
find_package(microxrcedds_client QUIET CONFIG)
if(microxrcedds_client_FOUND)
  fw_host_test(test_reliable_publish SOURCES test_reliable_publish.c ../../src/micro_ros/reliable_publish.cpp)
  target_link_libraries(test_reliable_publish PRIVATE microxrcedds_client)
  target_link_options(test_reliable_publish PRIVATE -Wl,--wrap=uxr_prepare_output_stream_fragmented)
  target_compile_options(test_reliable_publish PRIVATE -UNDEBUG)
endif()

fw_host_test(test_param_store SOURCES test_param_store.cpp OBLIBS fw_param_store
  INCLUDES ../../lib/motor/include)
fw_host_test(test_storage_boot_projection SOURCES test_storage_boot_projection.cpp
  OBLIBS fw_storage_projection fw_param_store)

fw_host_test(test_storage_boot_loader
  SOURCES test_storage_boot_loader.cpp ../../src/storage/storage_boot.cpp
  OBLIBS fw_storage_projection fw_param_store
  INCLUDES ../../lib/board/include ../../include)
