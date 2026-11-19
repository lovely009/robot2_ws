"""Exercise launch parameter serialization and actual TF delivery in an isolated domain."""

import os
import signal
import subprocess
import time

import pytest


def test_ros_launch_broadcasts_tf(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    from tf2_msgs.msg import TFMessage
    try:
        get_package_prefix('bvh_broadcaster')
    except PackageNotFoundError:
        pytest.skip('Build and source bvh_broadcaster before the ROS integration test')
    path = tmp_path / '动作.bvh'
    path.write_text('''HIERARCHY
ROOT Hips {
 OFFSET 0 100 0
 CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation
 End Site { OFFSET 0 10 0 }
}
MOTION
Frames: 2
Frame Time: 0.02
0 100 0 0 0 0
10 100 0 0 0 0
''')
    domain = 97
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('bvh_test_observer')
    received = []
    subscription = node.create_subscription(TFMessage, '/tf', received.append, 10)
    with (tmp_path / 'launch.log').open('w+') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'bvh_broadcaster', 'play.launch.py',
             f'bvh_file:={path}', 'loop:=true'],
            env={**os.environ, 'ROS_DOMAIN_ID': str(domain)},
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while len(received) < 5 and proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            log.flush()
            assert len(received) >= 5, (tmp_path / 'launch.log').read_text()
            transforms = received[-1].transforms
            assert {t.child_frame_id for t in transforms} == {'mocap/Hips', 'mocap/Hips__end'}
            root = next(t for t in transforms if t.child_frame_id == 'mocap/Hips')
            assert root.header.frame_id == 'world'
            assert root.transform.translation.z == pytest.approx(1.0)
            assert root.transform.rotation.w == pytest.approx(1.0)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
