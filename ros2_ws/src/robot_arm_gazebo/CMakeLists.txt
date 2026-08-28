cmake_minimum_required(VERSION 3.8)
project(robot_arm_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(motor_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(robot_arm_brake_coordinator src/brake_coordinator.cpp)
target_compile_features(robot_arm_brake_coordinator PUBLIC cxx_std_17)
target_compile_options(robot_arm_brake_coordinator PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(robot_arm_brake_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_arm_brake_coordinator
  controller_manager_msgs diagnostic_msgs rclcpp std_msgs)

add_executable(robot_arm_brake_coordinator_node src/brake_coordinator_main.cpp)
target_compile_features(robot_arm_brake_coordinator_node PUBLIC cxx_std_17)
target_compile_options(robot_arm_brake_coordinator_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robot_arm_brake_coordinator_node
  robot_arm_brake_coordinator)
ament_target_dependencies(robot_arm_brake_coordinator_node rclcpp)

add_executable(robot_arm_motor_sim_node
  src/motor_sim_main.cpp
  src/topology.cpp)
target_compile_features(robot_arm_motor_sim_node PUBLIC cxx_std_17)
target_compile_options(robot_arm_motor_sim_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(robot_arm_motor_sim_node PRIVATE include)
target_link_libraries(robot_arm_motor_sim_node motor_sim::motor_sim_node)
ament_target_dependencies(robot_arm_motor_sim_node ament_index_cpp rclcpp)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  robot_arm_motor_sim_node
  robot_arm_brake_coordinator
  robot_arm_brake_coordinator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf vendor worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/robot_arm_actions DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_motor_config
    test/test_motor_config.cpp
    src/topology.cpp)
  target_compile_features(test_motor_config PUBLIC cxx_std_17)
  target_compile_options(test_motor_config PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_motor_config PRIVATE include)
  target_compile_definitions(test_motor_config PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_motor_config motor_sim::motor_sim_core)
  ament_add_gtest(test_brake_coordinator
    test/test_brake_coordinator.cpp)
  target_compile_features(test_brake_coordinator PUBLIC cxx_std_17)
  target_compile_options(test_brake_coordinator PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_brake_coordinator PRIVATE include)
  target_link_libraries(test_brake_coordinator robot_arm_brake_coordinator)
  ament_target_dependencies(test_brake_coordinator
    controller_manager_msgs rclcpp std_msgs)
  file(GLOB PYTEST_FILES CONFIGURE_DEPENDS test/test_*.py)
  foreach(PYTEST_FILE ${PYTEST_FILES})
    get_filename_component(TEST_NAME ${PYTEST_FILE} NAME_WE)
    ament_add_pytest_test(${TEST_NAME} ${PYTEST_FILE})
  endforeach()
endif()

ament_package()
