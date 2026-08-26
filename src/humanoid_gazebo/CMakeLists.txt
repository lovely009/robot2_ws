cmake_minimum_required(VERSION 3.8)
project(humanoid_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(motor_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(humanoid_motor_sim_node
  src/motor_sim_main.cpp
  src/topology.cpp)
target_compile_features(humanoid_motor_sim_node PUBLIC cxx_std_17)
target_compile_options(humanoid_motor_sim_node PRIVATE
  -Wall -Wextra -Wpedantic)
target_include_directories(humanoid_motor_sim_node PRIVATE include)
target_link_libraries(humanoid_motor_sim_node motor_sim::motor_sim_node)
ament_target_dependencies(humanoid_motor_sim_node ament_index_cpp rclcpp)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS humanoid_motor_sim_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ros_actions
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_motor_config test/test_motor_config.cpp src/topology.cpp)
  target_compile_features(test_motor_config PUBLIC cxx_std_17)
  target_compile_options(test_motor_config PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_motor_config PRIVATE include)
  target_compile_definitions(test_motor_config PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_motor_config motor_sim::motor_sim_core)

  ament_add_gtest(test_frame_router
    test/test_frame_router.cpp
    src/topology.cpp)
  target_compile_features(test_frame_router PUBLIC cxx_std_17)
  target_compile_options(test_frame_router PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_frame_router PRIVATE include)
  target_compile_definitions(test_frame_router PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_frame_router motor_sim::motor_sim_core)

  ament_add_gtest(test_pty_integration
    test/test_pty_integration.cpp
    src/topology.cpp)
  target_compile_features(test_pty_integration PUBLIC cxx_std_17)
  target_compile_options(test_pty_integration PRIVATE
    -Wall -Wextra -Wpedantic)
  target_include_directories(test_pty_integration PRIVATE include)
  target_compile_definitions(test_pty_integration PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_pty_integration
    motor_sim::motor_sim_node util)
  ament_target_dependencies(test_pty_integration
    rclcpp sensor_msgs std_msgs)

  file(GLOB PYTEST_FILES CONFIGURE_DEPENDS test/test_*.py)
  foreach(PYTEST_FILE ${PYTEST_FILES})
    get_filename_component(TEST_NAME ${PYTEST_FILE} NAME_WE)
    ament_add_pytest_test(${TEST_NAME} ${PYTEST_FILE})
  endforeach()
endif()

ament_package()
