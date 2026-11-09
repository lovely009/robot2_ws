# robot_transmission

共享执行器／关节传动层，沿用 ROS `transmission_interface` 与真实 `pluginlib` 加载。Python `robot_transmission` 绑定与 C++ `Model` 使用同一算法；调用方不得复制空间连杆公式。

## 配置和完整组

`Model.load(driver_file)` 从 `motor_hardware.transmissions_file` 加载传动；未在传动文件中使用的执行器可由 driver 的 `joint` 字段生成直连组。几何、方向和轴序保存在 YAML，不按电机名字字典序重新排序。

- Simple/Differential 等原插件行为保持不变。
- `robot_transmission/SpatialLinkageTransmission` 是两输入／两输出球头空间定长连杆插件；当前配置见 `robot_description/models/trial_arm_7dof/config/transmissions.yaml`。
- 空间插件要求9个几何参数：电机轴原点／单位方向／零位球心、关节1轴原点／单位方向、关节2轴原点／单位方向／零位球心、杆长，单位 m；向量为三个空格分隔数，角度 rad。
- 两个 `actuator_reduction` 只允许 ±1；`joint_reduction` 必须为1，`joint_offset` 为0。安装相位由球心坐标表达，不加到命令上。
- 只允许完整组换算。一般反馈缺组时不生成该组输出；目标缺组时拒绝。纯力矩请求触及非线性组时，无论方向，都必须完整提供该组 effort 与输入侧位置。

## 姿态与力矩契约

完整 `Axis(position, velocity, effort)` 的力矩按同一个 Axis 的位置计算；速度使用雅可比，力矩使用其转置／逆转置。非线性组元数据 `position_dependent` 由实际插件类型派生，是 Python 只读字段，不是新 YAML 开关。

```python
from robot_transmission import Model

model = Model.load(driver_file)
motor_effort = model.joint_effort_to_actuator(
    joint_effort, joint_positions=measured_joint_positions)
joint_effort = model.actuator_effort_to_joint(
    motor_effort, actuator_positions=measured_actuator_positions)
```

- 位置 map 的键和值属于**输入侧空间**。线性插件允许省略第二参数以兼容旧调用；非线性插件缺姿态、部分 effort、NaN/Inf 或未知键时明确抛错。
- `Model` 不缓存姿态、不判断时钟；调用方负责新鲜度、组内同步和来源完整性。实时输出必须用实际测量，不能用目标角或零位替代。
- 主机将目标 P/V 换算与实测姿态力矩分开；仿真桥仅给有效组传入当次实际电机姿态，失效组输出零，不复用旧缓存。
- 非线性组的未来非零 effort 轨迹当前拒绝；未来实际姿态未知，只有明确设计轨迹前馈或实时固件转换后才能扩展。零 effort P/V 轨迹及旧线性 effort 轨迹保持支持。
- `Model` 单线程使用；不能并发调用同一个实例。`Group` 布局和 C++ 力矩接口已变，升级时须重建全部下游包，不承诺旧二进制 ABI。

## 数学分支与拒绝条件

解析旋转圆约束选取包含零位的固定导数同号分支，输出角固定在 `(-pi, pi]`。输入电机2的几何相位须在同侧开半圆内，不 wrap 越界输入，不按上次状态跳换镜像根。

解析定义域仅容忍最多64倍机器 epsilon 的舍入；归一化闭环和导数阈值为 `1e-10`。另要求 `sin(alpha)^2 * 1e-10 > 64 * epsilon`，拒绝 acos 近重根的病态反解；正向也复核输入圆反解，避免位置残差虽小而速度／力矩往返严重失真。以上是数值可辨识性保护，**不是机械安全限位**。

不可达、奇异、近切点数值不可逆、无穷／非数、无效几何或缺／重复 P/V/E handle 均拒绝整组；完整输出算完并校验后才写 handles。

## 离线检查

在工作区 source ROS 与安装环境后构建和测试：

```bash
colcon build --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host
colcon test-result --verbose
```

节点回归只使用合成反馈／ScriptedTransport，设置独立 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY=1`。测试覆盖 CAD 样例、正逆位置／速度／力矩、有限差分、功率、符号、分支／近切点拒绝、实际姿态调用和安装发现性。不能将测试通过视为硬件方向、编码器或行程验收。
