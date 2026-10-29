# 主机轨迹执行软件验收记录

2026-10-29。首阶段软件改造、隔离构建与测试已完成。用户授权更换模型后，gpt-6.1-sol 完成独立审查；发现的 1 项 P1、3 项 P2 已补回归并修复，针对性复核确认四项均已解决，本轮范围内无新增实质阻塞。未部署到真机。

## 已实施

- MCU 共同电机入口、周期目标保持、TX 前检查增加真实运动反馈 100 ms 门禁；SDO 不能刷新测量年龄，失效清目标并冻结逐电机授权，恢复需显式授权。
- 新 MicroRosSystem 复用 robot_transmission，接入真实反馈及 policy 状态，提供 P/V command 与 P/V/effort state；传递并检查真实 URDF 限位。
- JTC 控制循环直接发布 /motor/host_command_array。command mode switch 控制写权限，停用 JTC 后 manager 即使继续 write 也不再续旧目标，无独立重发 timer。
- 主机桥新增只读启动模式，host 仅接收新来源、拒绝 MCU 轨迹上传，接管先 STOP_ALL 并等后续状态确认旧授权撤销。
- robot.launch.py 新增可选主机路线；默认 firmware，现有 Web 动作库仍使用旧路线。首版主机入口是标准 FollowJointTrajectory action。

设计：[host-trajectory-execution.md](/home/ly/robot2_ws/docs/architecture/host-trajectory-execution.md)
使用：[host-trajectory-usage.md](/home/ly/robot2_ws/docs/architecture/host-trajectory-usage.md)
独立审查与修复复核：[host-trajectory-review.md](/home/ly/robot2_ws/outputs/host-trajectory-review.md)

## 实际验证

| 验证 | 结果 |
|---|---|
| 固件宿主 CTest | 33/33 通过 |
| STM32H723 PIO 构建 | 成功，未上传 |
| 三个受影响 ROS 包隔离构建 | 成功 |
| 三包测试汇总 | 359 项，0 errors / 0 failures / 0 skipped |
| 新插件公共 API 测试 | 包含在上述总数中，22/22 通过 |
| 真实 JTC→插件→V3桥→模拟 Agent | 包含在上述总数中，完成/取消/JTC停用/循环停止均通过 |
| 启动和模型定向测试 | 18 项通过，包含在上述总数中 |
| 独立审查及针对性复核 | 四项发现全部修复，复核无新增实质阻塞 |
| git diff --check | 通过 |

固件构建 RAM 322388 / 577536 bytes，Flash 292680 / 1048576 bytes。构建产物位于 [firmware.bin](/home/ly/robot2_ws/outputs/host-trajectory-firmware/firmware.bin)、[firmware.elf](/home/ly/robot2_ws/outputs/host-trajectory-firmware/firmware.elf)，[SHA256SUMS](/home/ly/robot2_ws/outputs/host-trajectory-firmware/SHA256SUMS) 记录精确候选版本。

运行目录和证据：

- 固件宿主：`/tmp/robot2-host-trajectory-fw-tests/Testing/Temporary/LastTest.log`
- PIO：`/tmp/robot2-host-trajectory-pio.log`
- 隔离 ROS 安装：`/tmp/robot2-host-trajectory-colcon/install`
- ROS 构建/测试：`/tmp/robot2-host-trajectory-final-build.log`、`/tmp/robot2-host-trajectory-final-test.log`
- `colcon test-result --test-result-base /tmp/robot2-host-trajectory-colcon/build --verbose` 返回 `359 tests, 0 errors, 0 failures, 0 skipped`
- 审查修复后重建并整包复跑插件测试：`/tmp/robot2-host-trajectory-review-final-build.log`、`/tmp/robot2-host-trajectory-review-final-test.log`。桥及其它启动测试使用同源已通过结果；更新后真实 JTC 集成通过 pytest 定向重跑，1 项通过（3.38 s），见 `/tmp/robot2-host-trajectory-review-integration.log`。
- 六个反例的红灯记录：`/tmp/robot2-host-trajectory-review-red.log`；当前构建库全 22 项通过：`/tmp/robot2-host-trajectory-review-green-current.log`。
- 测试使用独立 ROS domains 176/178/179/181，模拟 Agent 不连接 CAN/串口。

## 联调中修正的问题

1. 插件初版把 motion_enabled 当成首次目标的前提，导致与 MCU“首次目标触发使能序列”互相等待。已按 policy ACTIVE 权限判断，并允许 ENABLING 整定阶段；没有把本地使能电平当物理确认。
2. 控制器停用后 manager 仍可调用硬件 write。已用 command mode switch 撤销写权限，避免续发最后目标；重新占用从实测初始化。
3. 硬件 active 但尚未 armed 时也可能已有 JTC goal。会话变化现在一律闭锁，不允许旧会话目标自动进入新授权。
4. `last_failure_reason` 是历史诊断；当前 ACTIVE/policy 且 unified/vendor fault 为零时，历史失败记录不应成为新的运动门禁。公开回归先红后绿，避免一次已处理的策略拒绝永久阻断主机轨迹。
5. 主代理编写的模拟设备曾把量化 scale 误当乘数。已依据 V3 `q = round(value * 32767 / scale)` 修正，保留独立的实际位置断言，未修改产品代码掩盖测试错误。
6. 独立审查发现未武装时的当前故障可能在重新授权后恢复已有目标。配置组的 unified/vendor fault 或 ERROR_PROCESSING 现在立即闭锁，激活时同样拒绝；历史 last_failure_reason 仍不作为当前故障。
7. 聚合反馈原先要求与配置组完全相等。现在校验全部 ID 合法、唯一，要求配置组完整，只提取配置组的数据和权限；其它设备的离线、无测量或故障不会扩大本组控制范围。
8. 激活等待中可能先闭锁后收到好帧，原来仍返回 SUCCESS。现在出现闭锁立即返回 ERROR，后续好帧不能掩盖失败。
9. 插件原先漏算桥至回调的排队延迟。现在把非负源时间差保守计入 state_age_ms，再按单调钟老化；源延迟中桥已计入的部分可能重复计算，接近边界时可能提前拒收。新增六个反例均先红后绿，另加缺失/重复 ID 验证。

## 未执行与后续边界

- 未烧录、写 RAM/参数/SAVE、驱动电机或重启线上服务，未提交或推送。
- 现行固件轨迹和工作台入口保留；主机默认切换、Web action 客户端迁移、删除旧轨迹协议及 MCU 缓冲等待真机台架验收。
- 真实 JTC 测试在普通 Linux 调度下通过；测试日志显示未取得 FIFO 实时调度权限。平均 100 Hz 不能证明 USB/调度最大间隔满足 30 ms 窗口。
- USB 校时/负载抖动、实际使能确认、CAN bus-off 后电机停机、重力关节停止方式均未获真机验证。空快照/Topic 接收不代表物理停止。
- 主机路线本版支持位置/速度目标，力矩仅反馈；MIT kp/kd 仍走原 policy，未实现新力矩外环/前馈轨迹。
- 当前线上临时桥登记为 robot2-micro-ros-bridge-100hz.service。后续部署必须先核实并退出旧桥，避免双桥；本次未改变它。
