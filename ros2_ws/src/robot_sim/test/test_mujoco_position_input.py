"""外部位置数组按输入轴名字驱动既有 MuJoCo position 执行器。"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def bridge_module(monkeypatch):
    pytest.importorskip('mujoco')
    pytest.importorskip('rclpy')
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / 'scripts'))
    import mujoco_effort_bridge
    return mujoco_effort_bridge


def test_position_input_maps_names_and_clamps_to_existing_model_ranges(monkeypatch):
    module = bridge_module(monkeypatch)
    path = Path(__file__).parents[2] / 'robot_description/models/full_body_robot/mujoco/full_body_robot.xml'
    bridge = module.MujocoEffortBridge.__new__(module.MujocoEffortBridge)
    bridge.model = module.mujoco.MjModel.from_xml_path(str(path))
    bridge.data = module.mujoco.MjData(bridge.model)
    # 命令顺序故意不同于 MJCF 声明顺序，包含直接轴和差动输入轴。
    bridge.command_pairs = [(0, 13), (1, 7)]
    before = bridge.data.ctrl.copy()
    bridge._apply_position(np.array([100., .02]))
    assert bridge.data.ctrl[13] == bridge.model.actuator_ctrlrange[13, 1]
    assert bridge.data.ctrl[7] == .02
    np.testing.assert_array_equal(np.delete(bridge.data.ctrl, [7, 13]), np.delete(before, [7, 13]))
    module.mujoco.mj_step(bridge.model, bridge.data, nstep=10)
    assert np.isfinite(bridge.data.qpos).all()


def test_position_input_rejects_nonfinite_array_without_changing_target(monkeypatch):
    module = bridge_module(monkeypatch)
    bridge = module.MujocoEffortBridge.__new__(module.MujocoEffortBridge)
    bridge.command_joint_names = ['a', 'b']
    bridge.command = np.array([.1, .2])
    bridge.command_stamp = None
    bridge.get_logger = lambda: SimpleNamespace(warning=lambda *a, **kw: None)
    bridge._on_command(SimpleNamespace(data=[float('nan'), .3]))
    np.testing.assert_array_equal(bridge.command, [.1, .2])
    assert bridge.command_stamp is None


@pytest.mark.parametrize('speed', [.5, 1., 2.])
def test_position_steps_follow_elapsed_time_despite_slow_viewer(monkeypatch, speed):
    module = bridge_module(monkeypatch)
    bridge = module.MujocoEffortBridge.__new__(module.MujocoEffortBridge)
    bridge.model = SimpleNamespace(opt=SimpleNamespace(timestep=.002))
    bridge.steps_per_tick = round(speed / .2)
    bridge._speed = speed
    bridge.command_timeout_ns = 500_000_000
    # 首拍使用配置步数，后续慢回调模拟渲染阻塞。
    initial = bridge._position_step_count(100.)
    times = [100. + i * .043 for i in range(1, 101)]
    steps = sum(bridge._position_step_count(now) for now in times)
    assert initial == bridge.steps_per_tick
    assert steps * .002 == pytest.approx(4.3 * speed, abs=.002)


def test_position_steps_bound_catchup_after_command_timeout(monkeypatch):
    module = bridge_module(monkeypatch)
    bridge = module.MujocoEffortBridge.__new__(module.MujocoEffortBridge)
    bridge.model = SimpleNamespace(opt=SimpleNamespace(timestep=.002))
    bridge.steps_per_tick = 5
    bridge._speed = 1.
    bridge.command_timeout_ns = 500_000_000
    bridge._position_step_count(100.)
    assert bridge._position_step_count(110.) == 250
    assert bridge._position_step_count(110.01) == 5


def test_position_steps_accumulate_substep_intervals(monkeypatch):
    module = bridge_module(monkeypatch)
    bridge = module.MujocoEffortBridge.__new__(module.MujocoEffortBridge)
    bridge.model = SimpleNamespace(opt=SimpleNamespace(timestep=.002))
    bridge.steps_per_tick = 5
    bridge._speed = 1.
    bridge.command_timeout_ns = 500_000_000
    bridge._position_step_count(100.)
    assert bridge._position_step_count(100.0005) == 0
    assert bridge._position_step_count(100.001) == 0
    assert bridge._position_step_count(100.0015) == 0
    assert bridge._position_step_count(100.002) == 1


def test_live_position_servo_gains_follow_registration_and_committed_parameters(monkeypatch, tmp_path):
    module = bridge_module(monkeypatch)
    from rclpy.parameter import Parameter

    path = tmp_path / 'servos.xml'
    path.write_text("""
      <mujoco><custom><text name="robot_sim_control_mode" data="viewer_position"/></custom>
        <worldbody>
          <body><joint name="ankle"/><geom size=".1" mass="1"/></body>
          <body pos="1 0 0"><joint name="knee"/><geom size=".1" mass="1"/></body>
        </worldbody><actuator>
          <position joint="ankle" kp="12" kv="1.2" ctrlrange="-.3 .3"/>
          <position joint="knee" kp="8" kv=".7"/>
        </actuator></mujoco>""")
    module.rclpy.init(args=['--ros-args', '-p', f'mujoco_model:={path}',
                            '-p', 'headless:=true', '-p', 'position_input:=true',
                            '-p', 'position_servo.knee:=[15.0, 0.8]'])
    bridge = None
    try:
        bridge = module.MujocoEffortBridge()

        def tick(registered):
            msg = module.JointState(name=registered)
            msg.header.stamp = bridge.get_clock().now().to_msg()
            bridge._on_registry(msg)
            bridge._tick()

        def gains():
            return np.column_stack((bridge.model.actuator_gainprm[:, 0],
                                    bridge.model.actuator_biasprm[:, 1:3]))

        assert bridge.get_parameter('position_servo.ankle').value == [12., 1.2]
        tick(['knee'])
        np.testing.assert_allclose(gains(), [[0, 0, 0], [15, -15, -.8]])
        set_gains = bridge.set_parameters_atomically
        assert set_gains([Parameter('position_servo.ankle', value=[48., 1.2])]).successful
        tick(['knee'])
        np.testing.assert_allclose(gains()[0], 0.)
        tick(['ankle', 'knee'])
        np.testing.assert_allclose(gains(), [[48, -48, -1.2], [15, -15, -.8]])
        for bad in ([4.], [-1., 1.], [4., float('nan')], [4., -1.]):
            result = set_gains([Parameter('position_servo.ankle', value=[24., .6]),
                                Parameter('position_servo.knee', value=bad)])
            assert not result.successful
        tick([])
        np.testing.assert_allclose(gains(), 0.)
        tick(['ankle'])
        np.testing.assert_allclose(gains(), [[48, -48, -1.2], [0, 0, 0]])
        np.testing.assert_allclose(bridge.model.actuator_ctrlrange[0], [-.3, .3])
    finally:
        if bridge is not None:
            bridge.destroy_node()
        module.rclpy.shutdown()


def test_position_servo_parameters_require_an_unambiguous_position_actuator(monkeypatch, tmp_path):
    module = bridge_module(monkeypatch)
    path = tmp_path / 'shared_axis.xml'
    path.write_text("""
      <mujoco><custom><text name="robot_sim_control_mode" data="viewer_position"/></custom>
        <worldbody>
          <body><joint name="shared"/><geom size=".1" mass="1"/></body>
          <body pos="1 0 0"><joint name="speed"/><geom size=".1" mass="1"/></body>
        </worldbody><actuator>
          <position joint="shared" kp="4"/><position joint="shared" kp="7"/>
          <intvelocity joint="speed" kp="2" actrange="-.5 .5"/>
        </actuator></mujoco>""")
    module.rclpy.init(args=['--ros-args', '-p', f'mujoco_model:={path}',
                            '-p', 'headless:=true', '-p', 'position_input:=true'])
    bridge = None
    try:
        bridge = module.MujocoEffortBridge()
        assert not bridge.has_parameter('position_servo.shared')
        assert not bridge.has_parameter('position_servo.speed')
    finally:
        if bridge is not None:
            bridge.destroy_node()
        module.rclpy.shutdown()


def test_viewer_sync_is_rate_limited_to_viewer_hz(monkeypatch, tmp_path):
    module = bridge_module(monkeypatch)
    path = tmp_path / 'viewer_sync.xml'
    path.write_text("""
      <mujoco><custom><text name="robot_sim_control_mode" data="viewer_position"/></custom>
        <worldbody><body><joint name="j"/><geom size=".1" mass="1"/></body></worldbody>
        <actuator><position joint="j" kp="12" kv="1.2"/></actuator>
      </mujoco>""")
    module.rclpy.init(args=['--ros-args', '-p', f'mujoco_model:={path}',
                            '-p', 'headless:=true', '-p', 'viewer_hz:=20.0'])
    bridge = None
    try:
        bridge = module.MujocoEffortBridge()
        sync_calls = []
        bridge.viewer = SimpleNamespace(
            is_running=lambda: True,
            sync=lambda: sync_calls.append(True),
        )
        clock = [100.00]
        monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
        for t in (100.00, 100.01, 100.02, 100.04, 100.052, 100.06, 100.105):
            clock[0] = t
            bridge._tick()
        assert len(sync_calls) == 3
    finally:
        if bridge is not None:
            bridge.destroy_node()
        module.rclpy.shutdown()


def test_decoupled_viewer_sync_never_blocks_tick(monkeypatch, tmp_path):
    import threading
    import time
    module = bridge_module(monkeypatch)
    path = tmp_path / 'decoupled_viewer.xml'
    path.write_text("""
      <mujoco><custom><text name="robot_sim_control_mode" data="viewer_position"/></custom>
        <worldbody><body><joint name="j"/><geom size=".1" mass="1"/></body></worldbody>
        <actuator><position joint="j" kp="12" kv="1.2"/></actuator>
      </mujoco>""")
    module.rclpy.init(args=['--ros-args', '-p', f'mujoco_model:={path}',
                            '-p', 'headless:=true', '-p', 'position_input:=true',
                            '-p', 'viewer_hz:=0.0'])
    bridge = None
    try:
        bridge = module.MujocoEffortBridge()
        bridge._viewer_data = module.mujoco.MjData(bridge.model)
        synced = threading.Event()
        positions = []

        def slow_sync():
            positions.append(float(bridge._viewer_data.qpos[0]))
            synced.set()
            time.sleep(0.08)

        bridge.viewer = SimpleNamespace(is_running=lambda: True, sync=slow_sync)
        bridge._viewer_thread = threading.Thread(
            target=bridge._viewer_sync_loop, daemon=True)
        bridge._viewer_thread.start()

        started = time.monotonic()
        for step in range(5):
            bridge.data.qpos[0] = 0.1 * (step + 1)
            bridge._tick()
        elapsed = time.monotonic() - started
        assert elapsed < 0.05
        assert synced.wait(timeout=1.0)
        assert positions
    finally:
        if bridge is not None:
            bridge.destroy_node()
        module.rclpy.shutdown()
