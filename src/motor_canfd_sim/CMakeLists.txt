cmake_minimum_required(VERSION 3.8)
project(motor_canfd_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(motor_canfd_drive REQUIRED)
find_package(motor_hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_canfd_sim_node src/motor_canfd_sim_node.cpp)
target_compile_features(motor_canfd_sim_node PUBLIC cxx_std_17)
target_compile_options(motor_canfd_sim_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motor_canfd_sim_node PRIVATE include)
target_link_libraries(motor_canfd_sim_node
  motor_hardware_interface::motor_hardware_interface
  motor_canfd_drive::motor_canfd_drive)
ament_target_dependencies(motor_canfd_sim_node rclcpp sensor_msgs std_msgs)

install(TARGETS motor_canfd_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_plant_model test/test_plant_model.cpp)
  target_compile_features(test_plant_model PUBLIC cxx_std_17)
  target_compile_options(test_plant_model PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_plant_model PRIVATE include)
  target_link_libraries(test_plant_model motor_canfd_drive::motor_canfd_drive)
  ament_add_pytest_test(test_motor_canfd_sim_launch test/test_motor_canfd_sim_launch.py)
endif()

ament_python_install_package(motor_canfd_sim)
ament_package()
