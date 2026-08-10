# ROS 2 硬件驱动分层与参数文件规范调研

调研日期：2026-08-10
调研问题（来自 grilling 设计树）：在这一层级中，底层硬件驱动包需不需要依赖 ROS？硬件参数文件应该怎么放？

## TL;DR

1. **ROS 2 没有"底层驱动必须依赖 ROS"的规范。** ros2_control 官方文档只对"硬件接口插件包"提出了明确的依赖要求：至少依赖 `hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`。官方生态里 `ros2_socketcan` 是"一个 ROS 包内含节点 + SocketCAN 通信"的混合模式，说明官方并不强制"纯库 + ROS 适配层"的拆分——本工程的分层是**允许且更利于测试/复用**的设计决策，不是规范要求。
2. **yaml-cpp 不是 ROS 依赖。** `yaml_cpp_vendor` 只是 yaml-cpp 的构建包装，其 `package.xml` 不依赖 rclcpp。即使把 YAML 解析下沉到 `motor_canfd_drive` 也不会引入 ROS 运行时依赖；但按官方惯例与"单一配置源"原则，YAML 放在消费它的 ROS 侧包更合理。
3. **硬件参数文件按官方规范放在"使用它的包"的 `config/` 目录。** ROS 2 Developer Guide 明确规定 `config` 目录用于存放 YAML 参数文件等配置。RRBot 官方示例把 `rrbot_controllers.yaml` 放在 `ros2_control_demo_example_1/config/`，并用 `$(ros2 pkg prefix ... --share)/config/` 引用。`motor_canfd.yaml` 位于 `motor_hardware_interface/config/` 符合该惯例；仿真包通过 launch 引用同一文件（单一配置源）可接受。
4. **真正要处理的是"解析代码重复"，不是文件位置。** `canfd_system.cpp` 与 `motor_canfd_sim_node.cpp` 各有一份 `parse_driver_parameters` / `parse_driver_config`。建议把 YAML→`DriverConfig` 的解析提升为 `motor_hardware_interface` 的公开 API 供 sim 复用。

---

## 1. 底层驱动是否必须依赖 ROS

### 1.1 ros2_control 对"硬件接口包"的明确要求

ros2_control 官方文档 *Writing a Hardware Component* 规定，硬件接口包的 `CMakeLists.txt` 与 `package.xml` 至少需要：

> Under the line `find_package(ament_cmake REQUIRED)` add further dependencies. Those are at least: `hardware_interface`, `pluginlib`, `rclcpp` and `rclcpp_lifecycle`.
>
> Add at least the following packages into `<depend>` tag: `hardware_interface`, `pluginlib`, `rclcpp`, and `rclcpp_lifecycle`.

同时规定通信连接在 `on_configure` 中建立：

> Write the `on_configure` method where you usually setup the communication to the hardware and set everything up so that the hardware can be activated.

也就是说，**ROS 2 规范只约束"硬件接口插件包"（即本工程的 `motor_hardware_interface`）**，对它要求的就是 `rclcpp` 生命周期 + 插件机制。规范没有要求插件包之下还要有一层"必须依赖 ROS"的驱动——也没有禁止把通信与协议放在独立纯库中。

参考：<https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>

### 1.2 官方生态的两种组织方式（都没有违背规范）

**方式一：单包混合（官方 ros2_socketcan）**

`ros2_socketcan`（Humble 1.3.0）把 SocketCAN 桥接节点与底层通信放在同一个 ROS 包中：

```
<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
<depend>rclcpp_lifecycle</depend>
<depend>lifecycle_msgs</depend>
<depend>can_msgs</depend>
<depend>ros2_socketcan_msgs</depend>
```

本地交叉验证：`/opt/ros/humble/share/ros2_socketcan/package.xml`。它证明官方生态最常见的是"驱动直接就是 ROS 节点"，**混在一起是被官方接受的默认做法**。

**方式二：纯库 + ROS 适配层（本工程）**

```
motor_canfd_drive（无 ROS 依赖的 ament 包）
  ↑
motor_hardware_interface（依赖 hardware_interface/pluginlib/rclcpp/rclcpp_lifecycle）
```

本工程 `motor_canfd_drive/package.xml` 只有 `<buildtool_depend>ament_cmake</buildtool_depend>`，无任何 ROS 运行时依赖；`motor_hardware_interface` 的依赖与官方文档要求逐项对应。这种拆分不是规范强制，但让底层协议栈可以被无 ROS 环境独立测试/复用，属于良好实践。

> 备注：`motor_canfd_drive` 仍是 ament 包（用 colcon 构建），只是运行时零 ROS 依赖。若要求"彻底脱离 ROS 生态"，可改为纯 CMake 库 + 由 `motor_hardware_interface` 以 FetchContent/子目录方式引入；这是可选项，不是规范要求。

### 1.3 yaml-cpp 与 ROS 的关系

`yaml_cpp_vendor`（Humble 8.0.2）是 yaml-cpp 的 vendor 包装，`package.xml` 只有：

```
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>yaml-cpp</depend>
```

本地交叉验证：`/opt/ros/humble/share/yaml_cpp_vendor/package.xml`。它不依赖 rclcpp。结论：

- "YAML 需要 ROS"的说法不准确——`yaml-cpp` 是通用 C++ YAML 库。
- 因此"选项 B（把 YAML 解析下沉到 drive）"不会让 drive 引入 ROS 依赖。
- 但下沉会推翻已确认的"底层驱动不解析 YAML、只接收纯 `DriverConfig`"决策，且与"参数文件放消费它的包"的官方惯例相左，故不推荐。

### 1.4 REP 2001 澄清

REP 2001（*ROS 2 Variants*，Active）定义的是 ros_core / ros_base / desktop 等发行变体，**不规定包目录布局**。曾误以为它规定布局，特此更正：布局规范见第 2 节的 Developer Guide。

参考：<https://github.com/ros-infrastructure/rep/blob/master/rep-2001.rst>

---

## 2. 硬件参数文件应该怎么放

### 2.1 官方目录规范：`config/`

ROS 2 Developer Guide 的 *Filesystem layout* 章节明确规定包内目录用途：

> `config`: contains configuration files, e.g. YAML parameters files and RViz config files
> `launch`: contains all launch files
> `test`: contains all automated tests and test data

参考：<https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html>

即：**YAML 参数文件应放在"使用它的包"的 `config/` 目录**，launch 文件放在 `launch/`。

### 2.2 RRBot 官方示例的归属

ros2_control demos 的 example 1（RRBot）中，控制器参数 YAML 位于 bringup 包自己的 `config/` 目录，且文档用包前缀方式引用：

> `ros2 control load_controller joint_trajectory_position_controller $(ros2 pkg prefix ros2_control_demo_example_1 --share)/config/rrbot_jtc.yaml --set-state inactive`
>
> The parameters are already set in `rrbot_controllers.yaml`

文件清单中同样列明 `Controllers yaml: rrbot_controllers.yaml`。

参考：<https://control.ros.org/humble/doc/ros2_control_demos/example_1/doc/userdoc.html>

### 2.3 ros2_control 硬件参数的传入方式

硬件接口（SystemInterface）的参数由资源管理器从 URDF 的 `<ros2_control>` 标签中读取（`<param name="...">`），在 `on_init(info)` 中经 `info_.hardware_parameters` 获得。本工程用 xacro 参数把 `motor_config_file` 注入 URDF、再让 `CanFdSystem::load_config(path)` 读取 YAML，属于合法的官方路径；sim 侧通过 ROS 参数 `motor_config_file` 获取同一路径。

### 2.4 对本工程的结论

- `motor_canfd.yaml` 留在 `motor_hardware_interface/config/` **符合官方惯例**（参数文件放消费它的 ROS 侧包）。
- `motor_canfd_sim` 与 launch（未来在 `upper_body_canfd_gazebo`）通过 `$(ros2 pkg prefix motor_hardware_interface --share)/config/motor_canfd.yaml` 引用同一文件，保持"单一配置源"。
- 需要消除的重复：`parse_driver_parameters`（sim）与 `parse_driver_config`（hardware interface）是同一份 YAML schema 的两份实现，一旦新增字段容易两处漏改。建议把解析公开为 `motor_hardware_interface` 的库接口，sim 复用。

---

## 3. 现状核对（依赖矩阵）

| 包 | 当前依赖 | 与目标链的关系 |
|---|---|---|
| `motor_canfd_drive` | 仅 `ament_cmake`（无 ROS 运行时依赖） | 最底层，符合 |
| `motor_hardware_interface` | `hardware_interface` `pluginlib` `rclcpp` `rclcpp_lifecycle` `motor_canfd_drive` `yaml_cpp_vendor` | 与官方"硬件接口包"要求逐项对应，符合 |
| `motor_canfd_sim` | build: `motor_canfd_drive` `rclcpp` `sensor_msgs` `std_msgs` `yaml_cpp_vendor`；exec: `controller_manager` `launch` `launch_ros` `motor_hardware_interface` **`upper_body_canfd_gazebo`** | ⚠️ exec 依赖了 `upper_body_canfd_gazebo`（因 launch 引用其资源），与目标依赖链方向相反；launch 移入 gazebo 包后应反转 |
| `upper_body_canfd_gazebo` | 各 Gazebo/控制包 + exec `motor_hardware_interface`（尚无 `motor_canfd_sim`） | 需在 launch 迁入后补上对 `motor_canfd_sim` 的依赖 |

这与已确认的"4 包单向链"一致：`motor_canfd_drive ← motor_hardware_interface ← motor_canfd_sim ← upper_body_canfd_gazebo`。当前 `motor_canfd_sim` 的 launch 资源依赖造成反向边，是重构时要处理的具体点。

---

## 参考资料

- ROS 2 Developer Guide（Filesystem layout）：<https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html>
- ros2_control Writing a Hardware Component：<https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>
- ros2_control demos example 1（RRBot）：<https://control.ros.org/humble/doc/ros2_control_demos/example_1/doc/userdoc.html>
- REP 2001 ROS 2 Variants：<https://github.com/ros-infrastructure/rep/blob/master/rep-2001.rst>
- 本地包：`/opt/ros/humble/share/ros2_socketcan/package.xml`、`/opt/ros/humble/share/yaml_cpp_vendor/package.xml`
