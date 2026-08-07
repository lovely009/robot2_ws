from upper_body_canfd_gazebo import protocol_2530 as wrist
from upper_body_canfd_gazebo import protocol_rp as rp
from upper_body_canfd_gazebo.canfd_action_player import CanFdActionPlayer, ZERO_VELOCITY
from upper_body_canfd_gazebo.motion_library import T_POSE


class FakeBus:
    def __init__(self) -> None:
        self.sent = []

    def send(self, arbitration_id, data, **kwargs) -> None:
        self.sent.append((arbitration_id, data, kwargs))

    def recv_available(self, max_frames=128):
        return []


def _make_player_without_linux_can() -> CanFdActionPlayer:
    # 跳过 __init__，避免单元测试依赖 Linux vcan；这里只验证动作到 CAN 帧的映射。
    player = CanFdActionPlayer.__new__(CanFdActionPlayer)
    player.buses = {"left": FakeBus(), "right": FakeBus()}
    player.rp_kp = 32.0
    player.rp_kd = 2.2
    player.wrist_kp = 0.008
    player.wrist_kd = 1.0
    return player


def test_action_pose_maps_to_four_rp_slots_and_three_wrist_nodes_per_arm() -> None:
    player = _make_player_without_linux_can()
    player._send_pose(T_POSE, ZERO_VELOCITY)

    for side in ("left", "right"):
        expected = getattr(T_POSE, side)
        frames = player.buses[side].sent
        assert len(frames) == 4  # 1 个 RP 广播 + 3 个腕部 MIT 帧

        rp_frame = frames[0]
        assert rp_frame[0] == rp.MIT_BROADCAST_ID
        decoded_rp = rp.unpack_broadcast(rp_frame[1])
        for index in range(4):
            assert abs(decoded_rp[index].position - expected[index]) < 5e-4

        for node in (1, 2, 3):
            can_id, payload, _kwargs = frames[node]
            assert can_id == wrist.mit_command_id(node)
            decoded_wrist = wrist.unpack_mit_command(payload)
            assert abs(decoded_wrist.position - expected[3 + node]) < 5e-6

