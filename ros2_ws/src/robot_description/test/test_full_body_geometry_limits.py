"""Small geometry-scanner regressions, using the system Coal/NumPy ABI."""

import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

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
sys.path.insert(0, str(MODEL / 'tools'))
try:
    spec.loader.exec_module(scanner)
finally:
    sys.path.remove(str(MODEL / 'tools'))


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


def test_native_preview_keeps_collision_validation_explicitly_pending():
    report = json.loads((MODEL / "mujoco/geometry_limits.json").read_text())
    config = yaml.safe_load((MODEL / "config/simulation_limits.yaml").read_text())
    assert config['collision_validation'].startswith('pending:')
    assert report['source_sha256'] != config['geometry_sha256'] == scanner.source_fingerprint()
    assert config['source_sha256'] == hashlib.sha256(
        (MODEL / 'urdf/full_body_robot.urdf').read_bytes()).hexdigest()
    assert 'symmetric' in config['preview_limit_policy']


def test_native_search_caps_retain_legacy_evidence_without_claiming_collision_limits():
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    config = yaml.safe_load((MODEL / 'config/simulation_limits.yaml').read_text())
    aliases = json.loads((MODEL / 'config/joint_names.json').read_text())['joints']
    assert len(report['joint_limits']) == len(config['joint_limits']) == 37
    assert len(report['actuator_limits']) == len(config['actuator_limits']) == 37
    for old_name, bounds in report['joint_limits'].items():
        assert [config['legacy_scalar_caps'][aliases[old_name]][key]
                for key in ('lower', 'upper')] == bounds
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    series = {f'motor_{number:02d}': motor['joint']
              for number, motor in topology['motors'].items() if 'joint' in motor}
    assert len(series) == 25
    for actuator, joint in series.items():
        magnitude = max(abs(v) for v in config['legacy_scalar_caps'][joint].values())
        expected = [-magnitude, magnitude]
        assert [config['joint_limits'][joint][key] for key in ('lower', 'upper')] == expected
        assert [config['actuator_limits'][actuator][key] for key in ('lower', 'upper')] == expected
    for actuator, bounds in report['actuator_limits'].items():
        if actuator not in series:
            assert [config['actuator_limits'][actuator][key] for key in ('lower', 'upper')] == bounds
    for group in topology['differential_groups'].values():
        for joint in group['joints']:
            assert config['joint_limits'][joint] == config['legacy_scalar_caps'][joint]


def test_native_coupled_domains_match_the_display_model():
    from robot_transmission.coupled_limits import CoupledLimits
    config = yaml.safe_load((MODEL / 'config/simulation_limits.yaml').read_text())
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    root = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    embedded = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    assert embedded['coupled_limits'] == config['coupled_limits']
    assert embedded['joint_limits'] == {name: [bound['lower'], bound['upper']]
                                       for name, bound in config['joint_limits'].items()}
    assert set(config['coupled_limits']) == set(topology['differential_groups'])
    for name, workspace in config['coupled_limits'].items():
        assert workspace['joints'] == topology['differential_groups'][name]['joints']
        assert len({len(row) for row in workspace['valid_rows']}) == 1
    CoupledLimits(config['coupled_limits'], config['joint_limits']).validate(
        dict.fromkeys(config['joint_limits'], 0.))
