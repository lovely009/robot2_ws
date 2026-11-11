"""Small geometry-scanner regressions, using the system Coal/NumPy ABI."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

if int(np.__version__.split(".")[0]) >= 2:
    pytest.skip("Run the Coal scanner tests with system Python/NumPy", allow_module_level=True)
pytest.importorskip("coal")

MODEL = Path(__file__).parents[1] / "models/full_body_robot"
spec = importlib.util.spec_from_file_location(
    "full_body_limit_scanner", MODEL / "tools/derive_simulation_limits.py")
scanner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scanner)


def test_coupled_region_keeps_only_the_neutral_connected_component():
    first, second = np.arange(-2, 3), np.arange(-3, 4)
    mask = np.zeros((5, 7), dtype=bool)
    mask[1:4, 2:5] = True
    mask[0, 0] = True
    region = scanner.neutral_component(mask, first, second)
    assert region.sum() == 9
    assert region[2, 3] and not region[0, 0]
    mask[2, 3] = False
    with pytest.raises(ValueError, match='neutral'):
        scanner.neutral_component(mask, first, second)


def test_mating_exclusion_does_not_hide_other_components_of_the_same_link():
    vertices = "0 0 0 .1 0 0 0 .1 0 0 0 .1 1 0 0 1.1 0 0 1 .1 0 1 0 .1"
    faces = "0 2 1 0 1 3 0 3 2 1 2 3 4 6 5 4 5 7 4 7 6 5 6 7"
    model = scanner.mujoco.MjModel.from_xml_string(f"""<mujoco>
      <asset><mesh name="parts" vertex="{vertices}" face="{faces}"/></asset>
      <worldbody>
        <geom name="fixed" type="mesh" mesh="parts"/>
        <body name="moving"><joint type="slide" axis="1 0 0"/>
          <geom name="rotating" type="mesh" mesh="parts"/></body>
      </worldbody></mujoco>""")
    data = scanner.mujoco.MjData(model)
    scanner.mujoco.mj_kinematics(model, data)
    collisions = scanner.SurfaceCollisions(model, data)
    assert len(collisions.exclusions) == 2
    assert collisions.check(data) is None
    data.qpos[:] = 1
    scanner.mujoco.mj_kinematics(model, data)
    assert collisions.check(data) is not None


def test_selected_joint_assembly_filter_keeps_external_obstacles():
    model = scanner.mujoco.MjModel.from_xml_string('''<mujoco>
      <asset><mesh name="part" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"
        face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset>
      <worldbody>
        <geom name="housing" type="mesh" mesh="part" pos=".25 0 0"/>
        <body name="moving"><joint name="assembly_axis" type="slide" axis="1 0 0"/>
          <geom name="rotor" type="mesh" mesh="part"/></body>
        <body name="external" pos="1 0 0"><geom name="obstacle" type="mesh" mesh="part"/></body>
      </worldbody></mujoco>''')
    data = scanner.mujoco.MjData(model)
    scanner.mujoco.mj_kinematics(model, data)
    strict = scanner.SurfaceCollisions(model, data)
    filtered = scanner.SurfaceCollisions(model, data, assembly_joints=['assembly_axis'])
    data.qpos[:] = .25
    scanner.mujoco.mj_kinematics(model, data)
    assert strict.check(data) is not None
    assert filtered.check(data) is None
    data.qpos[:] = 1
    scanner.mujoco.mj_kinematics(model, data)
    assert 'obstacle' in filtered.check(data)


def test_calculated_limits_match_the_geometry_and_exported_configuration():
    report = json.loads((MODEL / "mujoco/geometry_limits.json").read_text())
    config = yaml.safe_load((MODEL / "config/simulation_limits.yaml").read_text())
    assert report["source_sha256"] == config["source_sha256"] == scanner.source_fingerprint()
    assert len(report["joint_limits"]) == len(config["joint_limits"]) == 37
    assert len(report['actuator_limits']) == len(config['actuator_limits']) == 37
    for name, bounds in report['actuator_limits'].items():
        assert [config['actuator_limits'][name]['lower'], config['actuator_limits'][name]['upper']] == bounds
    for name, bounds in report["joint_limits"].items():
        assert [config["joint_limits"][name]["lower"], config["joint_limits"][name]["upper"]] == bounds
    for side in ('Left', 'Right'):
        bound = config["joint_limits"][f"BN_{side}UpperLeg_axis_2"]
        np.testing.assert_allclose(np.rad2deg([bound["lower"], bound["upper"]]), [-15, 64])
    assert len(report['adjacent_assembly_exclusions']) == len(scanner.ASSEMBLY_JOINTS) == 18
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    assert set(config['coupled_limits']) == set(topology['differential_groups'])
    for name, workspace in config['coupled_limits'].items():
        assert workspace == report['differential_groups'][name]['coupled_workspace']
        assert len({len(row) for row in workspace['valid_rows']}) == 1
    assert len(report["differential_groups"]) == 6
    assert report["step_deg"] == 1 and report["margin_deg"] == 2
