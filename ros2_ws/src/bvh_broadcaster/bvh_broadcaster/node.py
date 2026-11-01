"""Publish a BVH skeleton using rclpy and tf2_ros; no robot commands."""

import math
import re

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .bvh import BvhClip, playback_frame


class BvhBroadcaster(Node):
    def __init__(self):
        super().__init__('bvh_broadcaster')
        defaults = {'bvh_file': '', 'base_frame': 'world', 'frame_prefix': 'mocap',
                    'scale': 0.01, 'up_axis': 'y_up', 'apply_root_offset': True,
                    'speed': 1.0, 'loop': False}
        self.declare_parameters('', list(defaults.items()))
        params = {name: self.get_parameter(name).value for name in defaults}
        if not params['bvh_file']:
            raise ValueError('bvh_file is required')
        for name in ('base_frame', 'frame_prefix'):
            if not re.fullmatch(r'[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', params[name]):
                raise ValueError(f'{name} must be a non-empty relative TF frame name')
        self.clip = BvhClip.load(params['bvh_file'])
        self.scale, self.up_axis = params['scale'], params['up_axis']
        self.apply_root_offset = params['apply_root_offset']
        self.speed, self.loop = params['speed'], params['loop']
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('speed must be finite and positive')
        self.clip.local_transforms(0, self.scale, self.up_axis, self.apply_root_offset)
        names = [params['frame_prefix'] + '/' + j.name for j in self.clip.joints]
        if params['base_frame'] in names:
            raise ValueError('base_frame conflicts with a skeleton frame')
        self.messages = []
        for joint, name in zip(self.clip.joints, names):
            msg = TransformStamped()
            msg.header.frame_id = params['base_frame'] if joint.parent < 0 else names[joint.parent]
            msg.child_frame_id = name
            self.messages.append(msg)
        self.broadcaster = TransformBroadcaster(self)
        self.started = self.get_clock().now()
        self.finished = False
        self.timer = self.create_timer(self.clip.frame_time / self.speed, self.publish_frame)
        self.get_logger().info(
            f'{len(self.clip.frames)} frames, {1 / self.clip.frame_time:.3f} Hz, '
            f'{len(names)} TF edges; prefix={params["frame_prefix"]}, '
            f'apply_root_offset={self.apply_root_offset}')
        self.publish_frame()

    def publish_frame(self):
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        if elapsed < 0:
            self.started, elapsed = now, 0.0
            self.finished = False
        index = playback_frame(elapsed, self.clip.frame_time, len(self.clip.frames),
                               self.speed, self.loop)
        poses = self.clip.local_transforms(index, self.scale, self.up_axis, self.apply_root_offset)
        stamp = now.to_msg()
        for msg, (position, quaternion) in zip(self.messages, poses):
            msg.header.stamp = stamp
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, position)
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, quaternion)
        self.broadcaster.sendTransform(self.messages)
        if not self.loop and not self.finished and elapsed * self.speed >= len(self.clip.frames) * self.clip.frame_time:
            self.get_logger().info('Playback finished; holding the last frame until stopped.')
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BvhBroadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
