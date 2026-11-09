# STM32 / can0 接通验收（2026-11-06）

用户确认总线仅连接STM32和仿真器，没有真实电机。工作台保持MuJoCo / firmware / SocketCAN / can0，STM32通过USB CDC Agent连接。

- can0：CAN FD，1 Mbps / 5 Mbps；现有后端can_up.sh将数据采样点调整为约0.8125、txqueuelen=200，非持久化配置。
- 仿真、Agent、电机后端均running；MCU在线、时钟正常。
- MCU control/status与后端status均确认ID1～7全部online/linked；3/4为R58，其余为R48。
- MCU逐轴policy_enabled=false、motion_enabled=false，control_lifecycle=INACTIVE，execution_state=IDLE，故障码均0；测量年龄约3～41ms。后端7台enabled均false，工作台显示7/7在线与未使能。
- 未烧录、未发送运动或使能命令。重力补偿保持停止。
- 配置/launch/HIL传动/原模型针对性回归51 passed；独立审查无阻断，另复核配置/HIL31 passed、管理器50 passed。

## 保留的控制门控

主机host_trajectory会话因来源URDF中Joint_Link3等轴的零位置/速度限位而启动失败，自动重试已停止。现有发现、状态反馈和MuJoCo仿真仍运行；滑杆和轨迹控制不可用。未将独立仿真限位覆盖写入来源URDF。后续若要求在此HIL中控制关节，需要显式为主机控制路径选择并验收仿真URDF与映射。

## 本轮文件

- config/motor_driver.simulation.yaml：独立7轴隔离仿真mapping；原始mapping不增加总线或增益。
- config/simulation_topology.yaml：7台仿真电机及输入侧范围，肩部范围为31×31输出网格反解包络，不代表全输入组合可达。
- robot_sim/launch/simulation.launch.py：仿真mapping优先读取；独立电机后端不依赖被控对象控制器文件，仍验证拓扑/传动/CAN。
- micro_ros_agent/tools/fdcan_motor_sim.py：非线性组用当拍新鲜输入侧姿态换算力矩；过期整组仍归零。
