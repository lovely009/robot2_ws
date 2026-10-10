cmake_minimum_required(VERSION 3.8)
project(robot_arm_gazebo)

find_package(ament_cmake REQUIRED)

# 模型/launch 集成包：models/（每模型一个自包含目录）+ 仿真拓扑配置和
# 两个启动期辅助节点，无编译目标。dist/ 是 ROS 无关的对外发布件。
install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/simulation_gate.py
  scripts/agent_after_sim_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
  ament_add_pytest_test(test_launch_models test/test_launch_models.py)
endif()

ament_package()
