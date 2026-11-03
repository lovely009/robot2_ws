# BVH → ROS 2 TF

将 `bvh_broadcaster` 的文件回放功能迁入 ROS 2 Humble。解析与广播层为 Python 3
重写，运行时不依赖 ROS 1 或厂商二进制 SDK。人体回放输出 `/tf`，保留原始人体骨架
及 End Site。另提供 `half_body_robot` 双臂 IK 离线转换与独立机器人预览，
不连接电机控制链路。

## 构建与回放

```bash
cd /home/ly/robot2_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bvh_broadcaster --symlink-install
source install/setup.bash
ros2 launch bvh_broadcaster play.launch.py \
  bvh_file:=/absolute/path/to/full-body.bvh \
  rviz:=true
```

将示例路径替换为实际 BVH；双臂 IK 需要含躯干和四肢的完整骨架。
内置动作位于 `web_host/action_library/`，其中 `1-4.bvh` 是精简骨架，不适用于双臂 IK。

按文件 `Frame Time` 播放；默认一次播放并持续广播末帧，Ctrl-C 停止。
循环追加 `loop:=true`，半速追加 `speed:=0.5`。RViz 配置默认 Fixed Frame 为 `world`；
修改 `base_frame` 时同步修改 RViz Fixed Frame。

| 参数 | launch 默认值 | 说明 |
|---|---|---|
| `bvh_file` | 必填 | BVH 路径，可含中文 |
| `base_frame` | `world` | 根 TF 的父坐标系 |
| `frame_prefix` | `mocap` | 隔离骨架名称，例如 `mocap/Hips` |
| `scale` | `0.01` | 位置/骨长换算为米；默认按厘米解释 |
| `up_axis` | `y_up` | `y_up` 将 `(X,Y,Z)` 映射为 ROS `(Z,X,Y)`；`z_up` 保持原坐标 |
| `apply_root_offset` | `false` | 是否将根 OFFSET 叠加到根平移通道 |
| `speed` | `1.0` | 播放倍率，必须为正数 |
| `loop` | `false` | 循环 |
| `rviz` | `false` | 启动 RViz2 |

根偏移须按导出器约定核对：launch 默认不叠加根 OFFSET，标准加法语义的文件
应传 `apply_root_offset:=true`。直接 `ros2 run` 节点时默认 true；需要忽略 OFFSET
时显式设置 false。

旋转严格按 CHANNELS 声明的内禀顺序组合。
有位置通道的节点要求位置通道排在旋转之前；不支持的布局会报错。
提供 TF 时，位置和旋转一起做坐标基变换，单位统一为米，四元数为 ROS `xyzw`。

## half_body_robot 双臂映射

映射采用 Pinocchio + SciPy IK。`config/half_body_robot.yaml` 指定人体躯干、
左右肩/肘/手与机器人连杆的语义对应及坐标基变换。**首帧必须是参考 T-pose**；
转换前须核对输入文件。用上臂、前臂的单位方向适配骨长差异，手腕朝向相对于
参考姿态标定；人体根平移、躯干转向和腿部动作不映射到固定底座双臂。
本转换器使用现有 `y_up`/厘米 BVH 约定；其他导出器须先核对约定。

```bash
source /opt/ros/humble/setup.bash
source /home/ly/robot2_ws/ros2_ws/install/setup.bash
ros2 run bvh_broadcaster retarget_bvh \
  /absolute/path/to/full-body.bvh \
  --urdf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/half_body_robot/urdf/half_body_robot.urdf \
  --output /tmp/half_body_robot-preview.npz
ros2 launch bvh_broadcaster retarget.launch.py \
  motion_file:=/tmp/half_body_robot-preview.npz rviz:=true
```

输出 `.npz` 保存关节角、帧时间、逐帧误差与模型/配置摘要，旁边 `.json` 为误差报告；
拒绝覆盖已有输出。转换时每帧比较多个初值，减少局部极小值，再按 URDF
逐关节速度限制逼近目标，保持原帧时间。因此不可达或变化过快的动作会出现误差、滞后。
方向误差分别统计上臂/前臂夹角，手腕误差为相对标定目标的 SO(3) 旋转角；
`nonconverged_solves` 包含比较时丢弃的候选，不能将生成成功视为精确复现。

预览只发布 `/mocap_robot/joint_states`，其专用 `robot_state_publisher` 输出
`retarget_robot/` 前缀的 TF，RViz 机器人位于人体左侧 1.5 m。
不发布现有 `/joint_states` 或任何电机/命令话题。若需同时看原始人体骨架，
另开终端运行前述 `play.launch.py`；两者独立启动，不保证起始时刻同步。

模型更新后的处理：

| 变化 | 处理 |
|---|---|
| 骨长、关节原点/轴、限位、速度 | 重新转换时读取最新 URDF，无需改 IK 代码；需重新检查误差 |
| 肩/肘/手连杆改名或语义改变 | 更新 YAML 对应并重新转换 |
| 同一独立链增减单自由度关节 | 关节及其数量从 URDF 读取；检查语义对应后重新转换 |
| 新增腿部、共用躯干链、连续/多自由度关节 | 当前双臂映射会明确拒绝，需扩展映射任务或求解器 |
| 只改外观网格、惯量或传动关系 | 运动学结果可能不变，但动力学/控制验收需另做 |

预览启动核对完整 URDF 的 SHA-256；URDF 任何内容变更都会拒绝旧动作，须重新转换。
网格外部文件变化不会改变 URDF 摘要，不会使关节动作失效。测试使用隔离 ROS 域。
关节角是 URDF 关节侧，未来接实机应接入工程现有 `transmission_interface` 传动层。
当前未做自碰撞、动力学、加速度/冲击或上机初始姿态过渡检查，输出只用于预览。

## 测试

```bash
cd /home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster
python3 -m pytest test
```

覆盖内禀旋转顺序、父子变换、单位/轴转换、根偏移、End Site、非法文件和回放边界；
IK 雅可比数值核对、逐帧限速、躯干变换不变性、模型更新/缺失对象和隔离 ROS 预览。
运行验收应先在独立 `ROS_DOMAIN_ID` 中进行，避免将测试 TF 混入正在运行的工作台。
