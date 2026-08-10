cmake_minimum_required(VERSION 3.8)
project(motor_canfd_sim)

find_package(ament_cmake REQUIRED)
find_package(motor_canfd_drive REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(motor_canfd_sim_node src/motor_canfd_sim_node.cpp)
target_compile_features(motor_canfd_sim_node PUBLIC cxx_std_17)
target_compile_options(motor_canfd_sim_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(motor_canfd_sim_node motor_canfd_drive yaml-cpp)
ament_target_dependencies(motor_canfd_sim_node rclcpp sensor_msgs std_msgs)

install(TARGETS motor_canfd_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_cpp_sim_launch test/test_gazebo_cpp_sim_launch.py)
endif()

ament_package()
