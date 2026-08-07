import math

from upper_body_canfd_gazebo import protocol_2530 as wrist
from upper_body_canfd_gazebo import protocol_rp as rp


def test_rp_manual_neutral_example() -> None:
    # The RP manual explicitly gives this slot for q=v=tau=kp=kd=0.
    assert rp.pack_mit_slot(rp.RpMitCommand()).hex(" ") == "7f ff 7f f0 00 00 07 ff"


def test_rp_broadcast_and_round_trip() -> None:
    commands = [
        rp.RpMitCommand(position=0.4, velocity=-1.2, kp=25.0, kd=1.5, torque_ff=2.0),
        rp.RpMitCommand(),
        rp.RpMitCommand(position=-0.7, velocity=2.0, kp=40.0, kd=2.0, torque_ff=-3.0),
        rp.RpMitCommand(position=1.0, velocity=0.0, kp=20.0, kd=1.0, torque_ff=0.0),
    ]
    packed = rp.pack_broadcast(commands)
    assert len(packed) == 64
    decoded = rp.unpack_broadcast(packed)
    assert abs(decoded[0].position - 0.4) < 5e-4
    assert abs(decoded[0].velocity + 1.2) < 0.04
    assert abs(decoded[0].torque_ff - 2.0) < 0.04


def test_rp_state_round_trip() -> None:
    payload = rp.pack_state(0.2, -0.8, 3.5, voltage_v=24, mos_temperature_c=42)
    assert len(payload) == 12
    state = rp.unpack_state(payload)
    assert abs(state.position - 0.2) < 5e-4
    assert abs(state.velocity + 0.8) < 0.04
    assert abs(state.torque - 3.5) < 0.04
    assert state.voltage_v == 24
    assert state.mos_temperature_c == 42


def test_2530_manual_command_example() -> None:
    # Manual section 3.1 example: signed multi-turn position code = 65537.
    q = 65537 / wrist.TICKS_PER_REVOLUTION * (2.0 * math.pi)
    command = wrist.WristMitCommand(position=q, velocity=0.0, kp=0.008, kd=1.0, current_ff=0.0)
    assert wrist.pack_mit_command(command).hex(" ") == "01 00 01 80 00 80 20 03 64 00 00 80"


def test_2530_ids_and_reply_round_trip() -> None:
    assert wrist.mit_command_id(1) == 0x505
    assert wrist.motor_control_id(1) == 0x506
    assert wrist.mit_reply_id(1) == 0x705
    assert wrist.status_id(1) == 0x706

    payload = wrist.pack_mit_reply(2, 0.35, -0.7, 1.25)
    reply = wrist.unpack_mit_reply(payload)
    assert reply.command_can_id == wrist.mit_command_id(2)
    assert abs(reply.position - 0.35) < 5e-6
    assert abs(reply.velocity + 0.7) < 0.02
    assert abs(reply.current - 1.25) < 0.01


def test_2530_status_length() -> None:
    payload = wrist.pack_status(3, 0.0, 0.0, 0.0, motor_type=0x02)
    assert len(payload) == 20
