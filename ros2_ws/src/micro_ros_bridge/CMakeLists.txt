cmake_minimum_required(VERSION 3.8)
project(micro_ros_bridge)

find_package(ament_cmake REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# codec：header-only、纯函数、不依赖 rclcpp（只依赖消息类型）。
add_library(${PROJECT_NAME}_codec INTERFACE)
target_compile_features(${PROJECT_NAME}_codec INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME}_codec INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_codec INTERFACE can_bridge_msgs)

# 消息级编解码：依赖 codec 的标量换算，不依赖 rclcpp。
add_library(${PROJECT_NAME}_messages src/messages.cpp src/agent_messages.cpp)
target_compile_features(${PROJECT_NAME}_messages PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_messages PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_messages PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_messages PUBLIC ${PROJECT_NAME}_codec)
ament_target_dependencies(${PROJECT_NAME}_messages PUBLIC can_bridge_msgs)

# 桥节点：主机侧 /motor/* 与线上 /micro_ros/* 之间的唯一翻译点。
add_library(${PROJECT_NAME}_node src/bridge_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_messages)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs can_bridge_msgs)

# 库与可执行目标必须异名；OUTPUT_NAME 保持 launch 使用的 micro_ros_bridge_node。
add_executable(${PROJECT_NAME}_node_main src/bridge_node_main.cpp)
set_target_properties(${PROJECT_NAME}_node_main PROPERTIES OUTPUT_NAME micro_ros_bridge_node)
target_link_libraries(${PROJECT_NAME}_node_main ${PROJECT_NAME}_node)
install(TARGETS ${PROJECT_NAME}_node_main RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(can_bridge_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_contract test/test_contract.cpp)
  target_compile_features(test_contract PRIVATE cxx_std_17)
  target_compile_options(test_contract PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_contract ${PROJECT_NAME}_codec)

  ament_add_gtest(test_codec test/test_codec.cpp)
  target_compile_features(test_codec PRIVATE cxx_std_17)
  target_compile_options(test_codec PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_codec ${PROJECT_NAME}_codec)

  ament_add_gtest(test_messages test/test_messages.cpp)
  target_compile_features(test_messages PRIVATE cxx_std_17)
  target_compile_options(test_messages PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_messages ${PROJECT_NAME}_messages)

  ament_add_gtest(test_low_state_assembler test/test_low_state_assembler.cpp)
  target_compile_features(test_low_state_assembler PRIVATE cxx_std_17)
  target_compile_options(test_low_state_assembler PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_low_state_assembler ${PROJECT_NAME}_messages)

  ament_add_gtest(test_agent_messages test/test_agent_messages.cpp)
  target_compile_features(test_agent_messages PRIVATE cxx_std_17)
  target_link_libraries(test_agent_messages ${PROJECT_NAME}_messages)

  ament_add_gtest(test_bridge_node test/test_bridge_node.cpp
    APPEND_ENV ROS_LOCALHOST_ONLY=1)
  target_compile_features(test_bridge_node PRIVATE cxx_std_17)
  target_compile_options(test_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_bridge_node ${PROJECT_NAME}_node ${PROJECT_NAME}_messages)
  ament_target_dependencies(test_bridge_node rclcpp)
endif()

ament_package()
