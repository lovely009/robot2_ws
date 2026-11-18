"""HIL 使用真实输入轴施加力矩，保留原位置预览和闭环机构。"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'


def builder():
    spec = importlib.util.spec_from_file_location('build_hil_config', MODEL / 'tools/build_hil_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hil_config_preserves_position_model_and_maps_all_motor_inputs(tmp_path):
    source = MODEL / 'mujoco/full_body_robot.xml'
    before = source.read_bytes()
    builder().build(MODEL, tmp_path)
    driver = yaml.safe_load((tmp_path / 'config/motor_driver.simulation.yaml').read_text())['motor_hardware']
    topology = yaml.safe_load((tmp_path / 'config/simulation_topology.yaml').read_text())['simulation']
    assert topology['control_mode'] == 'position'
    assert topology['mujoco_model'] == 'full_body_robot.xml'
    assert sorted(m['id'] for m in driver['motors'].values()) == list(range(1, 38))
    assert 'transmissions_file' not in driver
    inputs = [m['joint'] for m in topology['motors']]
    assert len(set(inputs)) == 37
    assert inputs[1:5] == ['waist_flange_plus_x', 'waist_flange_minus_x',
                           'hips_flange_plus_x', 'hips_flange_minus_x']
    assert inputs[12:14] == ['left_wrist_flange_proximal', 'left_wrist_flange_distal']
    assert source.read_bytes() == before
    actuators = ET.parse(source).find('actuator')
    assert len(actuators) == 37 and all(a.tag == 'position' for a in actuators)
    assert {a.get('joint') for a in actuators} == set(inputs)
    assert not (tmp_path / 'mujoco/full_body_robot.hil.xml').exists()


@pytest.mark.parametrize('types', [{1: 'cerb_r48_101'}, {i: 'unknown' for i in range(1, 38)}])
def test_hil_builder_requires_complete_known_motor_types(tmp_path, types):
    with pytest.raises(ValueError):
        builder().build(MODEL, tmp_path, types)
    assert not (tmp_path / 'config/simulation_topology.yaml').exists()


def test_cad_motor_types_cover_actual_r58_r48_and_x25s():
    types, evidence = builder().cad_motor_types(MODEL)
    assert [i for i, t in sorted(types.items()) if t == 'cerb_r58_101'] == [1, 2, 3, 4, 5, 8, 9, 17, 18, 24, 25, 26, 31, 32, 33]
    assert [i for i, t in sorted(types.items()) if t == 'vaxor_2550'] == [13, 14, 22, 23, 30, 37]
    assert sum(t == 'cerb_r48_101' for t in types.values()) == 16
    assert evidence['motors'][8]['cad_link'] == 'Joint_link3.2'


def test_hil_control_limits_use_model_positions_and_motor_output_speeds(tmp_path):
    import math
    import copy
    specs = yaml.safe_load((MODEL.parents[1] / 'config/motor.yaml').read_text())['motor_types']
    source = ET.parse(MODEL / 'urdf/full_body_robot.standalone.urdf').getroot()
    before = ET.tostring(source)
    builder().build(MODEL, tmp_path)
    controlled = ET.parse(tmp_path / 'urdf/full_body_robot.hil.urdf').getroot()
    motors = yaml.safe_load((tmp_path / 'config/motor_driver.simulation.yaml').read_text())['motor_hardware']['motors']
    actuators = {a.get('joint'): a for a in ET.parse(MODEL / 'mujoco/full_body_robot.xml').findall('actuator/position')}
    topology = yaml.safe_load((tmp_path / 'config/simulation_topology.yaml').read_text())['simulation']
    mapped = {m['joint']: topology['profiles'][m['profile']] for m in topology['motors']}
    assert len(motors) == 37
    for motor in motors.values():
        name = motor['joint']
        original = source.find(f"joint[@name='{name}']")
        joint = controlled.find(f"joint[@name='{name}']")
        limit = joint.find('limit')
        lower, upper = map(float, actuators[name].get('ctrlrange').split())
        if original.get('type') != 'continuous':
            lower = max(lower, float(original.find('limit').get('lower')))
            upper = min(upper, float(original.find('limit').get('upper')))
        assert joint.get('type') == 'revolute'
        assert float(limit.get('lower')) == pytest.approx(lower)
        assert float(limit.get('upper')) == pytest.approx(upper)
        assert math.radians(mapped[name]['command_lower_deg']) == pytest.approx(lower)
        assert math.radians(mapped[name]['command_upper_deg']) == pytest.approx(upper)
        params = specs[motor['type']]['parameters']
        assert float(limit.get('velocity')) == params['rated_speed_rad_s']
        assert float(limit.get('effort')) == params.get('peak_torque_nm', params['rated_torque_nm'])
        expected = copy.deepcopy(original)
        actual = copy.deepcopy(joint)
        for node in (expected, actual):
            node.attrib.pop('type', None)
            for element in node.findall('limit'):node.remove(element)
            for element in node.iter():
                if element.text is not None and not element.text.strip():element.text = None
                if element.tail is not None and not element.tail.strip():element.tail = None
        assert ET.tostring(actual) == ET.tostring(expected)
        assert lower < upper and all(math.isfinite(x) for x in (lower, upper))
    assert ET.tostring(ET.parse(MODEL / 'urdf/full_body_robot.standalone.urdf').getroot()) == before


@pytest.mark.parametrize('values', [(0, 1, 0, 10), (0, 1, -1, 10), (0, 1, float('nan'), 10),
                                   (0, 1, 2, 0), (0, 1, 2, float('inf')), (2, 3, 2, 10)])
def test_hil_control_limits_reject_invalid_specs_or_disjoint_model_ranges(tmp_path, values):
    (tmp_path / 'urdf').mkdir()
    (tmp_path / 'urdf/full_body_robot.standalone.urdf').write_text(
        '<robot name="fixture"><joint name="axis" type="revolute">'
        '<limit lower="-1" upper="1" velocity="0" effort="0"/></joint></robot>')
    with pytest.raises(ValueError, match='规格无效'):
        builder().control_urdf(tmp_path, {'axis': values})
