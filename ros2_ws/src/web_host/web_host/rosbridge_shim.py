"""rosbridge 启动垫片：兼容有界字符串（``string<=31``）。

背景：can_bridge_msgs 的 ``MotorTopologyEntry.model_name`` 使用有界字符串。
Humble 的 rclpy 把字段类型报告为 ``string<31>``（``ros2 interface show`` 记法为
``string<=31``），而 rosbridge_library 2.0.7 的原语类型表只含 ``string``，导致：

- 出站：序列化 MotorTopology 时抛 ``inst is not a ROS Message, but a str``；
- 入站：网页 publish 含该字段的消息时同样在原语判断处失败。

本垫片在运行官方 rosbridge 入口脚本前把两种记法补进原语表，行为与
``string`` 完全一致（长度约束在固件侧仍由 32B 静态缓冲保证，网页发送值
也不超过 31）。rosbridge 上游修复或消息包去掉有界字符串后可删除本文件
与 launch 中的接线。

官方入口是安装前缀下的可执行脚本（不是包内可导入模块），用 runpy 执行。
"""

import os
import runpy

from ament_index_python.packages import get_package_prefix

import rosbridge_library.internal.message_conversion as message_conversion
from rosbridge_library.internal import ros_loader


def _patch_bounded_strings() -> None:
    # 出站：ros_primitive_types；入站：type_map['str']（原为单个字符串
    # "string"，成员判断靠子串；换成元组并补上有界记法）。
    extra = ('string<31>', 'string<=31>')
    existing = tuple(message_conversion.ros_primitive_types)
    message_conversion.ros_primitive_types = tuple(
        dict.fromkeys((*existing, *extra)))
    message_conversion.type_map['str'] = ('string', *extra)


def _patch_standard_trajectory_action() -> None:
    """Resolve generated Action services through rosbridge's service protocol.

    ROS 2 places SendGoal/GetResult classes in ``package.action``. rosbridge's
    service loader otherwise looks in ``package.srv`` even when discovery
    reports an ``/action/`` type. Register only the standard trajectory action;
    the payload and ROS services remain the official generated interfaces.
    """
    from control_msgs.action import FollowJointTrajectory

    services = (FollowJointTrajectory.Impl.SendGoalService,
                FollowJointTrajectory.Impl.GetResultService)
    messages = [FollowJointTrajectory.Goal, FollowJointTrajectory.Result,
                FollowJointTrajectory.Feedback,
                FollowJointTrajectory.Impl.FeedbackMessage]
    with ros_loader._srvs_lock:
        for service in services:
            for middle in ('/', '/action/'):
                ros_loader._loaded_srvs['control_msgs' + middle + service.__name__] = service
            messages.extend((service.Request, service.Response))
    with ros_loader._msgs_lock:
        for message in messages:
            for middle in ('/', '/action/'):
                ros_loader._loaded_msgs['control_msgs' + middle + message.__name__] = message


def _patch_workbench_origin(port: int) -> None:
    from rosbridge_server.websocket_handler import RosbridgeWebSocket
    from tornado.websocket import WebSocketHandler
    from .static_server import _same_origin

    def check_origin(self, origin):
        return any(_same_origin(origin, f'{host}:{port}')
                   for host in ('localhost', '127.0.0.1', '[::1]'))

    async def get(self, *args, **kwargs):
        # Tornado 会跳过无 Origin 的检查，须在真正升级前封住这个旁路。
        headers = self.request.headers
        origin = headers.get('Origin', headers.get('Sec-Websocket-Origin'))
        if not check_origin(self, origin):
            self.set_status(403)
            self.finish('只接受本机工作台来源')
            return
        await WebSocketHandler.get(self, *args, **kwargs)

    RosbridgeWebSocket.check_origin = check_origin
    RosbridgeWebSocket.get = get


def main(args=None) -> None:
    _patch_workbench_origin(int(os.environ.get('WORKBENCH_WEB_PORT', '8080')))
    _patch_bounded_strings()
    _patch_standard_trajectory_action()
    script = os.path.join(
        get_package_prefix('rosbridge_server'),
        'lib', 'rosbridge_server', 'rosbridge_websocket')
    runpy.run_path(script, run_name='__main__')


if __name__ == '__main__':
    main()
