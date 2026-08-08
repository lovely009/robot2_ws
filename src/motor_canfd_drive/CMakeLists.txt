cmake_minimum_required(VERSION 3.8)
project(motor_canfd_drive)

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/config_loader.cpp
  src/motor_canfd_driver.cpp
  src/motor_protocol.cpp
  src/protocol.cpp
  src/socketcan.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(yaml_cpp_vendor)
ament_package()
