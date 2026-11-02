# 仿真

本包统一提供 Gazebo / MuJoCo 启动、协议级电机模拟和 HIL 辅助工具。模型与传动配置位于 `robot_description`，不在本包复制。

## 入口

- `simulation.launch.py`：工作台使用的组合仿真入口。
- `gazebo.launch.py` / `mujoco.launch.py`：对应物理引擎。
- `display.launch.py`：模型显示。
- `motor_sim.launch.py`：独立电机协议模拟，参数由 YAML 配置提供。

```bash
colcon build --packages-select robot_sim
source install/setup.bash
ros2 launch robot_sim motor_sim.launch.py motor_config_file:=/path/to/motors.yaml
```

原 `motor_sim` 发行包已合入本包；旧命令中的包名改为 `robot_sim`，可执行文件仍为 `motor_sim_node`。节点名、话题、参数和 `motor_sim/` 头文件路径不变。CMake 消费者使用 `find_package(robot_sim REQUIRED)`，导出目标为 `robot_sim::motor_protocol`、`robot_sim::motor_transport`、`robot_sim::motor_sim_core`、`robot_sim::motor_sim_ros`。

独立电机节点需要外部关节反馈和 effort 接收端；它通过 YAML 指定的总线端点发送协议帧。串口或物理 CAN 配置可能接入真实设备，应先确认端点与物理总线隔离。

测试：`colcon test --packages-select robot_sim`，然后 `colcon test-result --verbose`。单元与传动 HIL 测试使用替身或伪终端，不需要物理 CAN、串口或电机；不等同于完整 Gazebo / MuJoCo 闭环验收。
