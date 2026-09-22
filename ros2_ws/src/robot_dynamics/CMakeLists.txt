cmake_minimum_required(VERSION 3.8)
project(robot_dynamics)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pinocchio_dependency test/test_pinocchio_dependency.cpp)
  target_compile_features(test_pinocchio_dependency PRIVATE cxx_std_17)
  target_compile_options(test_pinocchio_dependency PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_pinocchio_dependency ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)
ament_package()
