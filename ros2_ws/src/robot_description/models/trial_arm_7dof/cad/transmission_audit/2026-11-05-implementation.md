# trial_arm_7dof 空间传动离线实施验收

实施／验证日期：2026-11-06。文件名沿用2026-11-05获批设计和几何证据的编号。

## 交付范围

- 共享 `robot_transmission/SpatialLinkageTransmission` 真实 pluginlib 插件：空间球头定长闭环、固定零位分支、解析雅可比、双向位置／速度／力矩换算。Web、主机与仿真桥不复制几何公式。
- `Group.position_dependent` 由实际插件类型派生；纯力矩 API 接收显式输入侧姿态，非线性组缺成员、缺姿态或非法值拒绝。旧线性无姿态调用兼容；C++ ABI改变，下游均已重建。
- 主机目标 P/V 与实测姿态力矩分开；现有100 ms运动反馈、300 ms控制授权、整组／锁存保护不变。新增读取成功后、写入前运动反馈到期的回归。
- 只读反馈桥对几何非法组清除样本及旧 held，恢复后以新测量重建未指定关节。非线性未来非零 effort 轨迹原子拒绝，线性 effort 和零 effort P/V 轨迹保留。
- 仿真桥只为当次完整、新鲜且同步的测量组传入电机位置；缺失、500 ms过期、50 ms以上偏差或几何非法时整组输出零，不使用缓存旧姿态，不影响其他有效组。
- 模型ID1～7对应 `Joint_Link1`～`Joint_Link7`；1/2联合换算，3～7直连。1/2/5/6/7为 `cerb_r48_101`，3/4为 `cerb_r58_101`，无端口、总线、PD或HIL配置。
- 增加 `Joint_Link2_axis` 和 `Joint_Link2_body_fixed`：当前15连杆／14关节，7转动＋7固定；原14个实体的XML参数、网格字节、零位世界位姿（绝对误差阈值1e-12）不变。总声明质量仍为 **3.2478525407320693 kg**。新增空框架不放宽其他实体惯性检查。
- Web现有元数据逻辑即可表达1/2不可展平、3～7直连、空 policy_defaults；没有新增Web产品算法。实际模型仍被 Workbench 的零 P/V/E 限位检查拒绝。

配置入口：`models/trial_arm_7dof/config/motor_driver.yaml`、`transmissions.yaml`。来源及球心／轴线见同目录 [STEP证据](2026-11-05-step.md)与JSON；使用当前STEP的20 mm／约34.329 mm／105.365452169 mm，不混用旧口述尺寸。57.1745°是装配相位，不另加电机命令偏置。

## 数学与来源验收

真实 Model／pluginlib 测试覆盖：

- 四组CAD样例 `(theta1, theta2, q2)`（度）：`(0,0,0)`、`(0,10,5.362931449174114)`、`(10,0,-1.0109202367883747)`、`(10,10,4.530462691371301)`；独立球心杆长闭环核对。
- 零位与非零位中心差分雅可比；零位 `J21≈-0.06708715259239355`、`J22≈0.5206392907848132`；速度／力矩往返与功率守恒，四种方向符号。
- 不可达俯仰、输入开半圆边界、调用顺序无状态、错轴数、错参数、非单位／零轴、球心落轴、NaN/Inf、缺／重复／空handle、重载与move、非字典序YAML轴名。
- 数值保护除归一化 `1e-10` 导数／闭环阈值外，要求 `sin(alpha)^2 * 1e-10 > 64 * epsilon`，并正向复核输入圆反解。它拒绝近切点不可辨识姿态，不是实测机械行程。
- 独立URDF FK验证正确轴线通过 `(0.0025,0.06775,0.5025)`、方向 `(0,-1,0)`，非零q1/q2的球头轨迹符合设计。惯性审查仅豁免指定的空轴框架，删除实体惯性仍失败。

## 执行命令与最终结果

所有命令在 `ros2_ws/` 执行，同次shell source `/opt/ros/humble/setup.bash` 与相应安装环境。合成ROS节点使用 `ROS_LOCALHOST_ONLY=1` 和独立域178／179；未接真实设备。

```bash
colcon build --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host
source install/setup.bash
colcon test --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host --event-handlers console_direct+
colcon test-result --verbose
/usr/bin/python3 -m pytest src/robot_description/test src/web_host/test -q -ra
```

现有工作区构建为 `RelWithDebInfo`。所有5包构建／测试通过；最终整工作区结果索引汇总为1301条、0失败、10跳过（包含其他包的既有结果，**不把这个数字当作本次独立用例数**）。description＋Web直接pytest为 **215 passed, 1 skipped**。

独立临时安装根：`/tmp/robot2-spatial-verify-ZgsDs8`。不复用这5包的build/install目录；未改动的 `can_bridge_msgs`、`robot_dynamics`、`bvh_broadcaster` 仍从原工作区underlay提供，故不是整个依赖树的无underlay构建。

```bash
VERIFY_ROOT=/tmp/robot2-spatial-verify-ZgsDs8
colcon --log-base "$VERIFY_ROOT/log" build \
  --build-base "$VERIFY_ROOT/build" --install-base "$VERIFY_ROOT/install" \
  --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host \
  --allow-overriding robot_transmission robot_description motor_hardware_interface robot_sim web_host \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source "$VERIFY_ROOT/install/setup.bash"
colcon --log-base "$VERIFY_ROOT/log" test \
  --build-base "$VERIFY_ROOT/build" --install-base "$VERIFY_ROOT/install" \
  --packages-select robot_transmission robot_description motor_hardware_interface robot_sim web_host \
  --event-handlers console_direct+
colcon test-result --test-result-base "$VERIFY_ROOT/build" --verbose
/usr/bin/python3 -m pytest src/robot_description/test src/web_host/test -q -ra
```

最终独立安装的colcon汇总 **746条、0错误、0失败、10跳过**，其中包含CTest容器。去除容器后的独立testcase结果如下，合计 **696通过、10跳过**：

| 包 | 通过 | 跳过 | 失败 |
|---|---:|---:|---:|
| robot_transmission | 26 | 0 | 0 |
| robot_description | 40 | 1 | 0 |
| motor_hardware_interface | 293 | 0 | 0 |
| robot_sim | 162 | 9 | 0 |
| web_host | 175 | 0 | 0 |

同一临时overlay下description＋Web直接pytest再次得到 **215通过、1跳过**。另在清空环境后只source ROS、临时 `robot_transmission`／`robot_description` 的package-local setup，description全测 **40通过、1跳过**；`colcon list --packages-up-to robot_description` 包含这两个包，确认新增test依赖未被全工作区overlay掩盖。

独立安装来源断言核实：5包ament prefix、实际载入的 `_core`、`/proc/self/maps` 中的 `librobot_transmission.so`、plugin XML及ament插件索引均来自临时根；由安装share定位driver，7个ID／6个组／4个CAD样例全部通过。安装URDF通过 `check_urdf`。

## 独立审查、失败与修复记录

分别使用映射内C++数学／生命周期、跨包集成、Python审查角色，只读检查本次tracked diff和新增文件；未调用外部provider，也没有声称跨供应商验证。四项问题均处理：

1. **近切点双向换算失真**：输入 `theta1=.17`、`theta2=1.9132826696561025` 时，旧实现4 Nm往返变48.7965 Nm；另一样例正向接受、反向拒绝。新增非零姿态近切点拒绝测试先失败，再增加根条件数和正向输入圆反解校验；正常可辨识附近姿态仍通过速度／力矩往返。
2. **非法几何反馈异常与旧held残留**：有效反馈→已有保持目标→完整新鲜但theta2=4 rad的非法反馈→恢复序列先复现ValueError。反馈边界现将整组失效、清除样本和held；恢复后的稀疏目标从新测量取得同组位置。
3. **description未声明新测试依赖**：补 `robot_transmission` 的test_depend，并完成上述仅两包环境的回归。
4. **read到write之间过期未覆盖**：新增非零effort与非零实测姿态用例，先成功read／write，再不调用read而等待运动反馈超过100 ms，确认write拒绝、只发布空快照并锁存。产品保护逻辑未放宽。

实施阶段保留各任务红／绿证据；Task1基础数学测试先因缺插件失败，但后来增加的单个解析器／handle拒绝用例未分别经历逐guard红灯，不把它们声称为逐项独立TDD证据。编译中ADL误选 `std::bind`、Humble handle无默认构造及测试YAML转换类型错误均已修正，最终构建无这些错误。

首轮默认无优化的独立新根构建曾使既有 `test_ros_segment_burst_preserves_every_point_in_one_goal` 的1536点轨迹处理超过500 ms新鲜度保护；独立重现和现场探针记录了过期拒绝。检查发现原工作区实际为 `RelWithDebInfo`（O2），按同一档位重建后该用例通过，未增加保护窗口。后台重建期间新增回归曾遇安装Python副本尚未更新；最终重新构建／运行已消除，不能用中间结果冒充最终绿灯。Debug／O0大轨迹吞吐不在本次已通过的构建档位内，仍需单独评估。

日志保留于本地 `.superpowers/sdd/2026-11-05-trial-arm-transmission/` 与临时验证根，包括首轮失败、诊断及 `final-build.log`、`final-test.log`、`final-result.log`、`final-pytest.log`、`provenance.log`；不提交这些临时记录。未自动提交或推送代码。

## 跳过与未验收

- `robot_description/test/test_right_arm_4dof.py::test_mujoco_loads_with_original_actuator_limits`：系统测试Python没有 `mujoco`，跳过1项。
- `robot_sim/test/test_mujoco_effort_limits.py::test_optional_joint_limit_keeps_actuator_limits`：参数组合 `(-100,False)`、`(-100,True)`、`(-.25,False)`、`(-.25,True)`、`(.25,False)`、`(.25,True)`、`(100,False)`、`(100,True)`，同一解释器缺MuJoCo，共8项。
- `robot_sim/test/test_model_consistency.py::test_dist_artifact_matches_model[token]`：仓库没有可匹配的dist产物参数集合，跳过1项。
- 这不表示整机没有MuJoCo：此前已确认实际launch使用的专用venv拥有MuJoCo；本次没有为消除跳过项安装依赖或启动物理引擎。
- 七个运动关节的原零P/V/E限位保留；没有猜测方向、编码器零位、行程、碰撞裕量或PD；没有启动电机、烧录、更改固件或其他模型默认值，没有上传CAD。
- 三处电机惯性仍为经批准的均匀密度凸包近似；R48暂用244g与厂商260g差异仍待确认。几何／软件验收不是实物质量分布或重力补偿验收。
- 本模型仍无MJCF、standalone URDF或仿真／HIL拓扑。数学分支和数值拒绝不是机械限位，配置可加载也不是硬件可运行。

格式检查：维护代码／文档及新增文本无尾随空白，新增文本有末尾换行；普通 `git diff --check` 对保留CRLF的运行URDF新增行报警，`git -c core.whitespace=cr-at-eol diff --check` 通过。未为消除此报警改写CAD原件或归一化来源文件。
