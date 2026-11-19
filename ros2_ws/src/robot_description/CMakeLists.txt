cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# 主机、TF、规划和网页共享机械资源。
# CAD exports are reference sources, not nested packages to build.
install(DIRECTORY models config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_installed_models test/test_installed_models.py)
  ament_add_pytest_test(test_full_body_robot test/test_full_body_robot.py)
  ament_add_pytest_test(test_full_body_native test/test_full_body_native.py)
endif()

ament_package()
