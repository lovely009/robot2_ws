from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from robot_calibration.cli import main
from robot_calibration.validation import (
    compute_fingerprint, create_template, validate_configuration,
)


@pytest.fixture
def files(tmp_path):
    (tmp_path / 'specs.yaml').write_text(yaml.safe_dump({
        'motor_types': {'cerb_r48_101': {'protocol': 'cerb', 'parameters': {}}}}))
    (tmp_path / 'transmissions.yaml').write_text(yaml.safe_dump({'transmissions': {
        'wrist': {'plugin': 'transmission_interface/DifferentialTransmission',
                  'actuators': ['a', 'b'], 'joints': ['j1', 'j2'],
                  'actuator_reduction': [1., 1.], 'joint_reduction': [1., 1.],
                  'joint_offset': [.1, -.2]}}}))
    driver = tmp_path / 'driver.yaml'
    driver.write_text(yaml.safe_dump({'motor_hardware': {
        'motor_specs_file': 'specs.yaml', 'transmissions_file': 'transmissions.yaml',
        'buses': {'arm': {'transport': 'serial_slcan', 'tty': '/dev/not-opened'}},
        'motors': {'a': {'id': 1, 'bus': 'arm', 'type': 'cerb_r48_101'},
                   'b': {'id': 2, 'bus': 'arm', 'type': 'cerb_r48_101'}}}}))
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('''<robot name="two_axes">
      <link name="base"/><link name="one"/><link name="two"/>
      <joint name="j1" type="revolute"><parent link="base"/><child link="one"/>
        <limit lower="-1" upper="1" velocity="2" effort="3"/></joint>
      <joint name="j2" type="revolute"><parent link="one"/><child link="two"/>
        <limit lower="-2" upper="2" velocity="3" effort="4"/></joint></robot>''')
    manifest = tmp_path / 'manifest.yaml'
    return driver, urdf, manifest


def template(files):
    return create_template(str(files[0]), str(files[1]), 'two_axes', 'assembly-001')


def accepted(files):
    record = template(files)
    record.update(calibration_id='commissioning-1', commissioned=True,
                  acceptance={'operator': 'operator-1', 'date': '2020-01-02',
                              'evidence': 'measurements/commissioning-1.csv'})
    record['buses']['arm'] = 1
    for motor in record['motors'].values():
        motor.update(bus_index=1, identity_verified=True)
    for joint in record['joints'].values():
        joint.update(zero_verified=True, direction_verified=True,
                     limits_verified=True, evidence='measurements/joint-check.csv')
    return record


def check(files, record):
    files[2].write_text(yaml.safe_dump(record))
    return validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')


def test_accepted_differential_configuration_uses_real_shared_model(files):
    record = accepted(files)
    assert record['joints']['j1']['direction'] == {'a': 1, 'b': 1}
    assert record['joints']['j2']['direction'] == {'a': 1, 'b': -1}
    result = check(files, record)
    assert result.valid and result.commissioned
    assert result.motor_ids == [1, 2]
    assert result.bus_indices == [1, 1]
    assert result.protocols == [1, 1]
    assert result.joint_names == ['j1', 'j2']
    assert result.asdict()['config_hash'] == compute_fingerprint(str(files[0]), str(files[1]))


def test_generated_template_never_commissions_itself(files):
    record = template(files)
    assert record['commissioned'] is False
    assert record['buses'] == {'arm': None}
    result = check(files, record)
    assert not result.valid and not result.commissioned
    assert 'assembly is not commissioned' in result.blockers
    assert any('identity is not verified' in reason for reason in result.blockers)


@pytest.mark.parametrize('filename', ['driver.yaml', 'specs.yaml', 'transmissions.yaml', 'robot.urdf'])
def test_every_effective_configuration_file_is_fingerprinted(files, filename):
    record = accepted(files)
    path = files[0].parent / filename
    path.write_text(path.read_text() + '\n')
    result = check(files, record)
    assert not result.valid
    assert 'configuration fingerprint mismatch' in result.blockers


@pytest.mark.parametrize('change,reason', [
    (lambda d: d.update(schema_version=True), 'schema_version'),
    (lambda d: d.update(robot_id='another-assembly'), 'robot_id'),
    (lambda d: d.update(model_name='another-model'), 'model_name'),
    (lambda d: d.update(commissioned='true'), 'not commissioned'),
    (lambda d: d.update(calibration_id=''), 'calibration_id'),
    (lambda d: d['acceptance'].update(operator=''), 'acceptance.operator'),
    (lambda d: d['acceptance'].update(evidence=''), 'acceptance.evidence'),
    (lambda d: d['acceptance'].update(date='not-a-date'), 'ISO-8601'),
    (lambda d: d['buses'].update(arm=True), 'buses.arm'),
    (lambda d: d['motors']['a'].update(bus_index=0), 'a.bus_index'),
    (lambda d: d['motors']['a'].update(motor_id=True), 'a.motor_id'),
    (lambda d: d['motors']['a'].update(protocol='vaxor'), 'a.protocol'),
    (lambda d: d['motors']['a'].update(model_name='another-type'), 'a.model_name'),
    (lambda d: d['motors']['a'].update(identity_verified=False), 'identity'),
    (lambda d: d['joints']['j2']['direction'].update(b=1), 'j2.direction'),
    (lambda d: d['joints']['j2']['direction'].update(b=True), 'j2.direction'),
    (lambda d: d['joints']['j1'].update(zero_offset_rad=.3), 'zero_offset_rad'),
    (lambda d: d['joints']['j1'].update(velocity_limit_rad_s=3.), 'velocity_limit'),
    (lambda d: d['joints']['j1'].update(lower_rad=-2.), 'lower_rad'),
    (lambda d: d['joints']['j1'].update(upper_rad=float('nan')), 'finite'),
    (lambda d: d['joints']['j1'].update(upper_rad=10**500), 'finite'),
    (lambda d: d['joints']['j1'].update(evidence=''), 'j1.evidence'),
    (lambda d: d['joints']['j1'].update(direction_verified='true'), 'direction_verified'),
    (lambda d: d['motors'].pop('b'), 'actuator set'),
    (lambda d: d['joints'].pop('j2'), 'joint set'),
])
def test_incomplete_or_wrong_commissioning_evidence_blocks_motion(files, change, reason):
    record = accepted(files)
    change(record)
    result = check(files, record)
    assert not result.valid
    assert any(reason in blocker for blocker in result.blockers), result.blockers


def test_duplicate_bare_motor_ids_are_rejected_even_with_different_buses(files):
    driver = yaml.safe_load(files[0].read_text())
    driver['motor_hardware']['buses']['second'] = {'transport': 'socketcan', 'interface': 'none'}
    driver['motor_hardware']['motors']['b'].update(id=1, bus='second')
    files[0].write_text(yaml.safe_dump(driver))
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid
    assert 'bare motor IDs' in result.blockers[0]


@pytest.mark.parametrize('protocol,invalid_id', [('cerb', 128), ('vaxor', 61)])
def test_protocol_specific_firmware_node_ranges_are_checked(files, protocol, invalid_id):
    specs = files[0].parent / 'specs.yaml'
    specs.write_text(yaml.safe_dump({'motor_types': {'cerb_r48_101': {'protocol': protocol}}}))
    driver = yaml.safe_load(files[0].read_text())
    driver['motor_hardware']['motors']['a']['id'] = invalid_id
    files[0].write_text(yaml.safe_dump(driver))
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid and 'firmware node range' in result.blockers[0]


@pytest.mark.parametrize('old,new', [
    ('upper="1"', 'upper="-1"'),
    ('velocity="2"', 'velocity="nan"'),
    ('effort="3"', 'effort="0"'),
    ('name="j1" type="revolute"', 'name="j1" type="continuous"'),
    ('name="j1" type="revolute"', 'name="other" type="revolute"'),
])
def test_invalid_urdf_cannot_authorize_anything(files, old, new):
    files[1].write_text(files[1].read_text().replace(old, new))
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid and result.blockers


def test_duplicate_yaml_keys_are_rejected(files):
    files[2].write_text('schema_version: 1\ncommissioned: false\ncommissioned: true\n')
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid and 'duplicate YAML key' in result.blockers[0]


def test_invalid_protocol_and_invalid_path_return_blockers(files):
    specs = files[0].parent / 'specs.yaml'
    specs.write_text(yaml.safe_dump({'motor_types': {'cerb_r48_101': {'protocol': []}}}))
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid and 'unsupported firmware protocol' in result.blockers[0]
    result = validate_configuration('\0', str(files[1]), str(files[2]), 'two_axes', 'assembly-001')
    assert not result.valid and 'usable file path' in result.blockers[0]


def test_missing_and_malformed_files_return_blockers(files):
    files[2].write_text('[]')
    assert not validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001').valid
    files[1].unlink()
    result = validate_configuration(*(str(p) for p in files), 'two_axes', 'assembly-001')
    assert not result.valid and 'cannot read' in result.blockers[0]


def test_implicit_transmission_dependency_is_included(files):
    driver = yaml.safe_load(files[0].read_text())
    del driver['motor_hardware']['transmissions_file']
    files[0].write_text(yaml.safe_dump(driver))
    record = accepted(files)
    path = files[0].parent / 'transmissions.yaml'
    path.write_text(path.read_text() + '\n')
    result = check(files, record)
    assert not result.valid and 'configuration fingerprint mismatch' in result.blockers


def test_ament_reference_fallback_preserves_content_fingerprint(files, monkeypatch, tmp_path):
    # The original relative reference succeeds; an installed copy resolves the
    # explicitly named package via ament. Neither location is itself hash input.
    source = tmp_path / 'source'
    source.mkdir()
    package = source / 'motor_hardware_interface'
    (package / 'config').mkdir(parents=True)
    specs = files[0].parent / 'specs.yaml'
    (package / 'config/motor.yaml').write_bytes(specs.read_bytes())
    driver = yaml.safe_load(files[0].read_text())
    driver['motor_hardware']['motor_specs_file'] = 'motor_hardware_interface/config/motor.yaml'
    src_driver = source / 'driver.yaml'
    src_driver.write_text(yaml.safe_dump(driver))
    (source / 'transmissions.yaml').write_bytes((files[0].parent / 'transmissions.yaml').read_bytes())
    first = compute_fingerprint(str(src_driver), str(files[1]))
    installed = tmp_path / 'isolated'
    installed.mkdir()
    dst_driver = installed / 'driver.yaml'
    dst_driver.write_bytes(src_driver.read_bytes())
    (installed / 'transmissions.yaml').write_bytes((source / 'transmissions.yaml').read_bytes())
    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(package) if name == 'motor_hardware_interface' else '/absent')
    assert compute_fingerprint(str(dst_driver), str(files[1])) == first
    (package / 'config/motor.yaml').write_text(specs.read_text() + '\n')
    assert compute_fingerprint(str(dst_driver), str(files[1])) != first


def test_cli_template_does_not_overwrite_and_check_has_meaningful_exit_status(files, capsys):
    options = ['--driver-file', str(files[0]), '--urdf-file', str(files[1]),
               '--model-name', 'two_axes', '--robot-id', 'assembly-001']
    assert main(['template', *options, '--output', str(files[2])]) == 0
    original = files[2].read_bytes()
    assert main(['template', *options, '--output', str(files[2])]) == 1
    assert files[2].read_bytes() == original
    assert main(['check', *options, '--manifest-file', str(files[2])]) == 1
    assert json.loads(capsys.readouterr().out)['valid'] is False
    check(files, accepted(files))
    assert main(['check', *options, '--manifest-file', str(files[2])]) == 0
    assert json.loads(capsys.readouterr().out)['valid'] is True
    assert main(['fingerprint', *options]) == 0
    assert capsys.readouterr().out.strip() == compute_fingerprint(str(files[0]), str(files[1]))


def test_repository_half_body_configuration_produces_only_uncommissioned_template():
    root = Path(__file__).resolve().parents[2] / 'robot_description/models/half_body_robot'
    config = root / 'config/motor_driver.yaml'
    urdf = root / 'urdf/half_body_robot.urdf'
    record = create_template(str(config), str(urdf), 'half_body_robot', 'example-only')
    assert record['commissioned'] is False
    assert len(record['motors']) == len(record['joints']) == 18
    assert record['joints']['joint_link9']['direction'] == {'arm_joint8': 1, 'arm_joint9': -1}
