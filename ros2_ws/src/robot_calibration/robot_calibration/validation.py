"""Read-only commissioning checks, independent of ROS node execution.

An accepted manifest records an operator's measurements. This module never
performs physical calibration and cannot verify a device serial number from the
current firmware topology protocol.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import date, datetime, timezone
import hashlib
import math
from pathlib import Path
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import yaml


class ValidationError(ValueError):
    """A configuration or acceptance record cannot establish readiness."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            if key in result:
                raise ValidationError(f'duplicate YAML key: {key!r}')
            result[key] = loader.construct_object(value_node, deep=deep)
        except TypeError as exc:
            raise ValidationError('YAML mapping keys must be scalar') from exc
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)
_MAX_FILE_BYTES = 2 * 1024 * 1024
_PROTOCOLS = {'vaxor': 0, 'cerb': 1}


@dataclass
class ValidationResult:
    robot_id: str = ''
    model_name: str = ''
    calibration_id: str = ''
    config_hash: str = ''
    valid: bool = False
    commissioned: bool = False
    motor_ids: list[int] = field(default_factory=list)
    bus_indices: list[int] = field(default_factory=list)
    protocols: list[int] = field(default_factory=list)
    model_names: list[str] = field(default_factory=list)
    joint_names: list[str] = field(default_factory=list)
    blockers: list[str] = field(default_factory=list)

    def asdict(self) -> dict:
        return asdict(self)


@dataclass
class _Configuration:
    fingerprint: str
    motors: dict[str, dict]
    joint_records: dict[str, dict]
    bus_names: list[str]


def _read(path: Path) -> bytes:
    try:
        with path.open('rb') as stream:
            content = stream.read(_MAX_FILE_BYTES + 1)
    except OSError as exc:
        raise ValidationError(f'cannot read {path}: {exc.strerror}') from exc
    if len(content) > _MAX_FILE_BYTES:
        raise ValidationError(f'configuration file exceeds {_MAX_FILE_BYTES} bytes: {path}')
    return content


def _yaml(content: bytes, label: str) -> dict:
    try:
        document = yaml.load(content, Loader=_UniqueLoader)
    except (yaml.YAMLError, UnicodeError, RecursionError) as exc:
        raise ValidationError(f'invalid YAML in {label}: {exc}') from exc
    if not isinstance(document, dict):
        raise ValidationError(f'{label} must contain a YAML object')
    return document


def _path(value: str, name: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f'{name} is required')
    try:
        return Path(value).expanduser().resolve()
    except (OSError, ValueError, RuntimeError) as exc:
        raise ValidationError(f'{name} is not a usable file path') from exc


def _finite(value: Any, name: str) -> float:
    if type(value) not in (int, float):
        raise ValidationError(f'{name} must be a finite number')
    try:
        numeric = float(value)
    except OverflowError as exc:
        raise ValidationError(f'{name} must be a finite number') from exc
    if not math.isfinite(numeric):
        raise ValidationError(f'{name} must be a finite number')
    return numeric


def _integer(value: Any, lower: int, upper: int, name: str) -> int:
    if type(value) is not int or not lower <= value <= upper:
        raise ValidationError(f'{name} must be an integer in [{lower}, {upper}]')
    return value


def _text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f'{name} must be nonempty text')
    return value


def resolve_reference(driver_file: str, reference: str) -> Path:
    """Resolve an explicit reference, with the known model-library install fix.

    Missing arbitrary files are returned at their declared location and fail
    when read. Only the existing cross-package motor specification reference
    may fall back to its ament share directory.
    """
    driver_path = _path(driver_file, 'driver_file')
    reference = _text(reference, 'configuration reference')
    try:
        path = (driver_path.parent / reference).resolve()
    except (OSError, ValueError, RuntimeError) as exc:
        raise ValidationError('configuration reference is not a usable file path') from exc
    if path.is_file():
        return path
    # Source workspaces may reference another package by ../../.../package/...
    # while isolated ament installs put that package under another prefix.
    # Only use a package explicitly named in the reference and a contained tail.
    parts = Path(reference).parts
    package = 'motor_hardware_interface'
    for index, part in enumerate(parts[:-1]):
        if part != package or parts[index + 1:] != ('config', 'motor.yaml'):
            continue
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory(package)).resolve()
            candidate = share.joinpath('config', 'motor.yaml').resolve()
            if candidate.is_relative_to(share) and candidate.is_file():
                return candidate
        except (ImportError, LookupError, OSError, ValueError, RuntimeError):
            continue
    return path


def load_model(driver_file: str):
    """Load the shared transmission model using the same resolved dependencies.

    A temporary selector file makes explicit references absolute while retaining
    all driver values. The shared model is self-contained after loading; the
    user's driver and the fingerprint inputs are never rewritten.
    """
    from robot_transmission import Model
    driver_path = _path(driver_file, 'driver_file')
    document = _yaml(_read(driver_path), 'driver_file')
    hardware = document.get('motor_hardware')
    if not isinstance(hardware, dict):
        raise ValidationError('driver_file requires motor_hardware')
    for key in ('motor_specs_file', 'transmissions_file'):
        reference = hardware.get(key)
        if reference is None and key == 'transmissions_file':
            implicit = driver_path.parent / 'transmissions.yaml'
            reference = 'transmissions.yaml' if implicit.is_file() else None
        if reference is not None:
            hardware[key] = str(resolve_reference(str(driver_path), reference))
    with tempfile.TemporaryDirectory(prefix='robot-calibration-') as directory:
        selector = Path(directory) / 'driver.yaml'
        selector.write_text(yaml.safe_dump(document), encoding='utf-8')
        return Model.load(str(selector))


def _bundle(driver_file: str, urdf_file: str) -> tuple[str, dict, bytes, dict]:
    driver_path = _path(driver_file, 'driver_file')
    urdf_path = _path(urdf_file, 'urdf_file')
    contents = {'driver': _read(driver_path), 'urdf': _read(urdf_path)}
    driver = _yaml(contents['driver'], 'driver_file')
    hardware = driver.get('motor_hardware')
    if not isinstance(hardware, dict):
        raise ValidationError('driver_file requires motor_hardware')
    specs = {}
    for key in ('motor_specs_file', 'transmissions_file'):
        reference = hardware.get(key)
        # Match robot_transmission.Model.load's implicit transmissions.yaml.
        if reference is None and key == 'transmissions_file':
            fallback = driver_path.parent / 'transmissions.yaml'
            reference = 'transmissions.yaml' if fallback.is_file() else None
        if reference is not None:
            reference = _text(reference, key)
            path = resolve_reference(str(driver_path), reference)
            contents[key] = _read(path)
            parsed = _yaml(contents[key], key)
            if key == 'motor_specs_file':
                specs = parsed.get('motor_types', {})
    if not isinstance(specs, dict) or not specs:
        raise ValidationError('motor_specs_file must provide motor_types')
    digest = hashlib.sha256()
    for role, content in sorted(contents.items()):
        digest.update(role.encode('utf-8') + b'\0')
        digest.update(len(content).to_bytes(8, 'big'))
        digest.update(content)
    return digest.hexdigest(), hardware, contents['urdf'], specs


def compute_fingerprint(driver_file: str, urdf_file: str) -> str:
    """Hash driver, URDF, model specifications and explicit/implicit transmission.

    Paths are selectors, not hash input, so unchanged installed and source
    configurations have the same fingerprint.
    """
    return _bundle(driver_file, urdf_file)[0]


def _configuration(driver_file: str, urdf_file: str) -> _Configuration:
    digest, hardware, urdf, specs = _bundle(driver_file, urdf_file)
    motors = hardware.get('motors')
    buses = hardware.get('buses')
    if not isinstance(motors, dict) or not motors or len(motors) > 64:
        raise ValidationError('motor_hardware.motors must contain 1..64 motors')
    if not isinstance(buses, dict) or not 1 <= len(buses) <= 3:
        raise ValidationError('motor_hardware.buses must contain 1..3 logical buses')
    for name, bus in buses.items():
        _text(name, 'bus name')
        if not isinstance(bus, dict):
            raise ValidationError(f'bus {name} must be an object')
    expected_motors = {}
    ids = set()
    for name, motor in motors.items():
        _text(name, 'actuator name')
        if not isinstance(motor, dict):
            raise ValidationError(f'motor {name} must be an object')
        mid = _integer(motor.get('id'), 1, 65535, f'{name}.id')
        if mid in ids:
            raise ValidationError('bare motor IDs must be unique across all buses')
        ids.add(mid)
        bus = _text(motor.get('bus'), f'{name}.bus')
        if bus not in buses:
            raise ValidationError(f'{name} references unknown bus {bus}')
        model = _text(motor.get('type'), f'{name}.type')
        profile = specs.get(model)
        if not isinstance(profile, dict):
            raise ValidationError(f'{name} has unknown model type {model}')
        protocol = profile.get('protocol', model)
        if not isinstance(protocol, str) or protocol not in _PROTOCOLS:
            raise ValidationError(f'{name} has unsupported firmware protocol {protocol!r}')
        maximum_id = 60 if protocol == 'vaxor' else 127
        if mid > maximum_id:
            raise ValidationError(f'{name}.id exceeds the {protocol} firmware node range 1..{maximum_id}')
        expected_motors[name] = {'motor_id': mid, 'bus': bus,
                                 'protocol': protocol, 'model_name': model}
    try:
        from robot_transmission import Axis
        model = load_model(driver_file)
    except (ImportError, ValueError, RuntimeError, OSError) as exc:
        raise ValidationError(f'cannot validate shared transmission model: {exc}') from exc
    if dict(model.actuator_ids) != {name: m['motor_id'] for name, m in expected_motors.items()}:
        raise ValidationError('shared transmission actuator IDs differ from driver')
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise ValidationError(f'invalid URDF: {exc}') from exc
    if root.tag != 'robot':
        raise ValidationError('URDF root must be robot')
    joints = {}
    for joint in root.findall('joint'):
        name = _text(joint.get('name'), 'URDF joint name')
        if name in joints:
            raise ValidationError(f'duplicate URDF joint {name}')
        joints[name] = joint
    offsets = {name: offset for group in model.groups
               for name, offset in zip(group.joints, group.joint_offset)}
    joint_records = {}
    for name in model.joint_names:
        joint = joints.get(name)
        if (joint is None or joint.get('type') not in ('revolute', 'prismatic')
                or joint.find('mimic') is not None):
            raise ValidationError(f'{name} must be an independent bounded revolute/prismatic joint')
        limit = joint.find('limit')
        if limit is None:
            raise ValidationError(f'{name} has no URDF limits')
        try:
            values = {key: float(limit.attrib[key])
                      for key in ('lower', 'upper', 'velocity', 'effort')}
        except (KeyError, ValueError) as exc:
            raise ValidationError(f'{name} has incomplete/non-numeric URDF limits') from exc
        for key, value in values.items():
            _finite(value, f'{name}.{key}')
        if not values['lower'] < values['upper'] or values['velocity'] <= 0 or values['effort'] <= 0:
            raise ValidationError(f'{name} has invalid URDF limits')
        group = next(g for g in model.groups if name in g.joints)
        origin = {j: Axis() for j in group.joints}
        baseline = model.joint_to_actuator(origin)
        origin[name] = Axis(1.0)
        positive = model.joint_to_actuator(origin)
        directions = {}
        for actuator in group.actuators:
            delta = positive[actuator].position - baseline[actuator].position
            if not math.isfinite(delta) or delta == 0:
                raise ValidationError(f'{name}/{actuator} has no finite nonzero direction')
            directions[actuator] = 1 if delta > 0 else -1
        joint_records[name] = {
            'zero_offset_rad': _finite(offsets[name], f'{name}.configured_offset'),
            'direction': directions,
            'lower_rad': values['lower'], 'upper_rad': values['upper'],
            'velocity_limit_rad_s': values['velocity'],
        }
    active = {name for name, j in joints.items()
              if j.get('type') != 'fixed' and j.find('mimic') is None}
    if active != set(joint_records):
        raise ValidationError('URDF independent active joints must exactly match the transmission model')
    if compute_fingerprint(driver_file, urdf_file) != digest:
        raise ValidationError('configuration changed while it was being validated')
    return _Configuration(digest, expected_motors, joint_records, list(buses))


def create_template(driver_file: str, urdf_file: str, model_name: str, robot_id: str) -> dict:
    """Produce an explicitly uncommissioned record from the selected configuration."""
    _text(model_name, 'model_name')
    _text(robot_id, 'robot_id')
    config = _configuration(driver_file, urdf_file)
    return {
        'schema_version': 1, 'robot_id': robot_id, 'model_name': model_name,
        'calibration_id': '', 'config_hash': config.fingerprint, 'commissioned': False,
        'acceptance': {'operator': '', 'date': '', 'evidence': ''},
        'buses': {name: None for name in config.bus_names},
        'motors': {name: {key: value for key, value in motor.items() if key != 'bus'}
                   | {'bus_index': None, 'identity_verified': False}
                   for name, motor in config.motors.items()},
        'joints': {name: record | {'zero_verified': False, 'direction_verified': False,
                                  'limits_verified': False, 'evidence': ''}
                   for name, record in config.joint_records.items()},
    }


def _acceptance_date(value: Any) -> None:
    if type(value) is date:
        value = value.isoformat()  # SafeLoader accepts an unquoted YAML date.
    value = _text(value, 'acceptance.date')
    try:
        day = date.fromisoformat(value)
    except ValueError as exc:
        raise ValidationError('acceptance.date must be an ISO-8601 YYYY-MM-DD date') from exc
    if day > datetime.now(timezone.utc).date():
        raise ValidationError('acceptance.date cannot be in the future')


def validate_configuration(driver_file: str, urdf_file: str, manifest_file: str,
                           model_name: str, robot_id: str) -> ValidationResult:
    """Fail closed on incomplete evidence, configuration drift or invalid topology."""
    result = ValidationResult(robot_id=robot_id if isinstance(robot_id, str) else '',
                              model_name=model_name if isinstance(model_name, str) else '')
    try:
        _text(model_name, 'model_name')
        _text(robot_id, 'robot_id')
        config = _configuration(driver_file, urdf_file)
        result.config_hash = config.fingerprint
        result.joint_names = list(config.joint_records)
        manifest = _yaml(_read(_path(manifest_file, 'manifest_file')), 'manifest_file')
    except (ValidationError, RecursionError) as exc:
        result.blockers.append(str(exc))
        return result

    def check(condition: bool, message: str) -> None:
        if not condition:
            result.blockers.append(message)

    check(type(manifest.get('schema_version')) is int and manifest['schema_version'] == 1,
          'manifest schema_version must be 1')
    check(manifest.get('robot_id') == robot_id, 'manifest robot_id mismatch')
    check(manifest.get('model_name') == model_name, 'manifest model_name mismatch')
    check(manifest.get('config_hash') == config.fingerprint, 'configuration fingerprint mismatch')
    calibration_id = manifest.get('calibration_id')
    if isinstance(calibration_id, str):
        result.calibration_id = calibration_id
    check(isinstance(calibration_id, str) and bool(calibration_id.strip()),
          'calibration_id is required')
    result.commissioned = manifest.get('commissioned') is True
    check(result.commissioned, 'assembly is not commissioned')
    acceptance = manifest.get('acceptance')
    if not isinstance(acceptance, dict):
        result.blockers.append('acceptance must contain operator/date/evidence')
    else:
        for key in ('operator', 'evidence'):
            try:
                _text(acceptance.get(key), 'acceptance.' + key)
            except ValidationError as exc:
                result.blockers.append(str(exc))
        try:
            _acceptance_date(acceptance.get('date'))
        except ValidationError as exc:
            result.blockers.append(str(exc))

    buses = manifest.get('buses')
    if not isinstance(buses, dict):
        buses = {}
    check(set(buses) == set(config.bus_names), 'manifest logical bus set mismatch')
    bus_values = {}
    for name in config.bus_names:
        try:
            bus_values[name] = _integer(buses.get(name), 0, 2, f'buses.{name}')
        except ValidationError as exc:
            result.blockers.append(str(exc))
    check(len(set(bus_values.values())) == len(bus_values), 'MCU bus indices must be unique')

    motors = manifest.get('motors')
    if not isinstance(motors, dict):
        motors = {}
    check(set(motors) == set(config.motors), 'manifest actuator set mismatch')
    for name, expected in config.motors.items():
        record = motors.get(name)
        if not isinstance(record, dict):
            record = {}
        bus_index = bus_values.get(expected['bus'], 255)
        result.motor_ids.append(expected['motor_id'])
        result.bus_indices.append(bus_index)
        result.protocols.append(_PROTOCOLS[expected['protocol']])
        result.model_names.append(expected['model_name'])
        for key in ('motor_id', 'protocol', 'model_name'):
            value = record.get(key)
            check(value == expected[key] and (key != 'motor_id' or type(value) is int),
                  f'{name}.{key} does not match configuration')
        check(type(record.get('bus_index')) is int and record['bus_index'] == bus_index
              and bus_index != 255, f'{name}.bus_index does not match the accepted bus mapping')
        check(record.get('identity_verified') is True, f'{name} identity is not verified')

    joints = manifest.get('joints')
    if not isinstance(joints, dict):
        joints = {}
    check(set(joints) == set(config.joint_records), 'manifest joint set mismatch')
    for name, expected in config.joint_records.items():
        record = joints.get(name)
        if not isinstance(record, dict):
            record = {}
        for key in ('zero_verified', 'direction_verified', 'limits_verified'):
            check(record.get(key) is True, f'{name}.{key} is not accepted')
        for key in ('zero_offset_rad', 'lower_rad', 'upper_rad', 'velocity_limit_rad_s'):
            try:
                value = _finite(record.get(key), f'{name}.{key}')
                check(math.isclose(value, expected[key], rel_tol=0.0, abs_tol=1e-12),
                      f'{name}.{key} differs from active configuration')
            except ValidationError as exc:
                result.blockers.append(str(exc))
        direction = record.get('direction')
        check(isinstance(direction, dict) and direction == expected['direction']
              and all(type(v) is int and v in (-1, 1) for v in direction.values()),
              f'{name}.direction differs from the shared transmission model')
        try:
            _text(record.get('evidence'), name + '.evidence')
        except ValidationError as exc:
            result.blockers.append(str(exc))
    result.valid = not result.blockers
    return result
