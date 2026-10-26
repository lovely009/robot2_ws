cmake_minimum_required(VERSION 3.8)
project(robot_sim)

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
  scripts/mujoco_effort_bridge.py
  # 桥按关节名下发命令用的纯映射模块；桥按同目录 import，所以必须和它装在一起。
  scripts/mujoco_command_mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
  ament_add_pytest_test(test_launch_models test/test_launch_models.py)
  ament_add_pytest_test(test_mujoco_command_mapping test/test_mujoco_command_mapping.py)
  ament_add_pytest_test(test_simulation_config test/test_simulation_config.py)
  ament_add_pytest_test(test_transmission_hil test/test_transmission_hil.py)
endif()

ament_package()
