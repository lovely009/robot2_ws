cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# One asset tree shared by MCU bringup, TF, planning, simulation and the web UI.
# CAD exports are reference sources, not nested packages to build.
install(DIRECTORY models config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_installed_models test/test_installed_models.py)
endif()

ament_package()
