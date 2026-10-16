cmake_minimum_required(VERSION 3.5)
project(bench_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  # 台架控制服务：仿真 / Agent / 电机后端 / 重力前馈（bench_manager 提供实现）
  "srv/GetBenchOptions.srv"
  "srv/GetAgentStatus.srv"
  "srv/StartSimulation.srv"
  "srv/StopSimulation.srv"
  "srv/StartAgent.srv"
  "srv/StopAgent.srv"
  "srv/StartMotorBackend.srv"
  "srv/StopMotorBackend.srv"
  "srv/MotorBackendCommand.srv"
  "srv/GetUnitLog.srv"
  "srv/StartGravityFeedforward.srv"
  "srv/StopGravityFeedforward.srv"
  "srv/SetGravityFeedforwardGain.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
