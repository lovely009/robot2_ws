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
  ament_add_pytest_test(test_motor_frames test/test_motor_frames.py)
  ament_add_pytest_test(test_right_arm_4dof test/test_right_arm_4dof.py)
  ament_add_pytest_test(test_trial_mujoco test/test_trial_mujoco.py)
  ament_add_pytest_test(test_trial_arm_7dof test/test_trial_arm_7dof.py)
  ament_add_pytest_test(test_arm_mass_inertia test/test_arm_mass_inertia.py)
  ament_add_pytest_test(test_full_body_robot test/test_full_body_robot.py)
  ament_add_pytest_test(test_full_body_native test/test_full_body_native.py)
  ament_add_pytest_test(test_step_alignment test/test_step_alignment.py)
  ament_add_pytest_test(test_step_linkages test/test_step_linkages.py)
  ament_add_pytest_test(test_hips_mounting test/test_hips_mounting.py)
  ament_add_pytest_test(test_full_body_geometry_limits test/test_full_body_geometry_limits.py)
endif()

ament_package()
