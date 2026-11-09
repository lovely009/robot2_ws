# HIL 控制模型限位写入（2026-11-06）

已写入 `urdf/trial_arm_7dof.standalone.urdf`，位置上下限来自 geometry_limits.json；速度取型号库 rated_speed_rad_s，力矩取 peak_torque_nm。来源URDF保留原零限位，仅firmware HIL选择仿真mapping和standalone URDF，非HIL仍使用来源配置。

| 轴 | 型号 | 峰值力矩 N·m | 额定输出速度 rad/s |
|---|---|---:|---:|
| 1、2、5、6、7 | R48 101:1 | 11 | 3.1416 |
| 3、4 | R58 101:1 | 45 | 3.1416 |

1/2的topology限位是电机输入包络，URDF限位是输出关节范围，不能直接数值等同。961个肩部输出采样经真实传动反解，均落入对应输入包络。URDF速度用于规划/命令约束，MuJoCo不额外将真实速度硬夹到该数值；本次不把配置上限宣传为物理硬止挡。

## 验证

- 限位生成/资源隔离/实际host_trajectory硬件描述函数：MuJoCo目标6项通过。
- 相关描述/仿真配置42项、feedback manager与node91项通过（两组存在重叠，不相加宣称独立总数）。
- robot_transmission colcon记录29项，0错误0失败；新增外层hardware pluginlib加载空间传动测试1项通过，且没有直接链接核心库掩盖嵌套装载条件。
- 独立审查无新增阻断。先前对J2输入与输出范围差异的疑问经961点反解复核已撤回。
- 模型/管理器已安装，STM32/can0/MuJoCo连接恢复；页面最终7/7在线、全部未使能，未烧录或发送运动。

## 运行限制（未掩盖）

限位校验已通过，嵌套插件工厂装载错误也已消除。注册宏现仅在独立 `librobot_transmission_plugins.so`，核心库不包含插件注册，避免外层ros2_control装载核心依赖时占用注册归属。

主机会话曾报告反馈新鲜且控制器就绪，但后续验收再次触发现有MCU控制流时效保护：示例source_age约301.99ms、receive_age约141ms，随后activation等待完整新鲜反馈超时。最终主机自动重试停止，控制会话未稳定就绪；工作台CAN发现/反馈和MuJoCo仍运行。本次没有放宽300ms门限、重试上限或其他保护。
