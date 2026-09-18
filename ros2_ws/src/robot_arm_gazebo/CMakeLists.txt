cmake_minimum_required(VERSION 3.8)
project(robot_arm_gazebo)

find_package(ament_cmake REQUIRED)

# 纯资源包：models/（每模型一个自包含目录）+ 共享 config/launch，无编译目标。
# dist/ 是对外发布件（ROS 无关），不随包安装。
install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
  ament_add_pytest_test(test_launch_models test/test_launch_models.py)
endif()

ament_package()
