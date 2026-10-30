cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

# 仿真启动与辅助节点；公共机械模型由 robot_description 独立安装。
install(DIRECTORY
  config
  launch
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
