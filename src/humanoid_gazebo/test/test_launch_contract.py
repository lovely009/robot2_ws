"""静态验证最终的三个 launch 入口，不启动 Gazebo 或串口设备。"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from humanoid_gazebo.launch_support import validate_serial_ports


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch"


def _load(name: str):
    path = LAUNCH / name
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _defaults(name: str) -> dict[str, str]:
    context = LaunchContext()
    declarations = {
        entity.name: entity
        for entity in _load(name).generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    for declaration in declarations.values():
        declaration.execute(context)
    return dict(context.launch_configurations)


def test_only_final_launch_entrypoints_remain() -> None:
    assert {path.name for path in LAUNCH.glob("*.launch.py")} == {
        "gazebo.launch.py",
        "usb_sim.launch.py",
        "canfd_sim.launch.py",
    }


def test_gazebo_launch_uses_final_model_and_controllers() -> None:
    source = (LAUNCH / "gazebo.launch.py").read_text()
    assert "humanoid.urdf.xacro" in source
    assert "humanoid_world.sdf" in source
    assert "controllers.yaml" in source
    assert "joint_state_broadcaster" in source
    assert "effort_controller" in source
    assert '"on_exit_shutdown": "true"' in source
    assert "_start_after_success" in source
    assert "upper_body" not in source


@pytest.mark.parametrize(
    ("name", "mode", "allow"),
    [
        ("usb_sim.launch.py", "usb_serial", "false"),
        ("canfd_sim.launch.py", "serial_slcan", "false"),
    ],
)
def test_serial_launch_contract(name: str, mode: str, allow: str) -> None:
    defaults = _defaults(name)
    assert defaults["transport_mode"] == mode
    assert defaults["allow_external_canfd"] == allow
    assert {"left_arm_port", "right_arm_port", "legs_port"} <= defaults.keys()


def test_three_ports_must_be_stable_and_unique() -> None:
    valid = {
        "left_arm": "/dev/serial/by-id/usb-left",
        "right_arm": "/dev/serial/by-id/usb-right",
        "legs": "/dev/serial/by-id/usb-legs",
    }
    validate_serial_ports(valid, "usb_serial", allow_external_canfd=False)

    with pytest.raises(ValueError, match="/dev/serial/by-id"):
        validate_serial_ports(
            {**valid, "legs": "/dev/ttyUSB2"},
            "usb_serial",
            allow_external_canfd=False,
        )
    with pytest.raises(ValueError, match="unique"):
        validate_serial_ports(
            {**valid, "legs": valid["left_arm"]},
            "usb_serial",
            allow_external_canfd=False,
        )


def test_canfd_launch_is_explicitly_opt_in() -> None:
    ports = {
        "left_arm": "/dev/serial/by-id/usb-left",
        "right_arm": "/dev/serial/by-id/usb-right",
        "legs": "/dev/serial/by-id/usb-legs",
    }
    with pytest.raises(ValueError, match="allow_external_canfd"):
        validate_serial_ports(ports, "serial_slcan", allow_external_canfd=False)
    validate_serial_ports(ports, "serial_slcan", allow_external_canfd=True)
