"""为单电机/完整传动组会话生成共用配置，不修改模型源文件。"""
from pathlib import Path
import yaml
from robot_transmission import Model


def materialize_control_subset(config_file, motor_ids, directory):
    source = Path(config_file).resolve(strict=True)
    try:
        ids = [int(value) for value in motor_ids.split(',')]
    except ValueError as error:
        raise ValueError('motor_ids 必须是逗号分隔的电机 ID') from error
    if not ids or len(set(ids)) != len(ids) or any(not 1 <= value <= 64 for value in ids):
        raise ValueError('motor_ids 必须唯一且在 1..64 内')
    model = Model.load(str(source))
    configured = list(model.actuator_ids.values())
    if (not 0 < len(configured) <= 48 or len(set(configured)) != len(configured)
            or any(not 1 <= value <= 64 for value in configured)):
        raise ValueError('模型 motor_id 必须唯一、在 1..64 内且最多 48 台')
    wanted = set(ids)
    if not wanted <= set(model.actuator_ids.values()):
        raise ValueError('motor_ids 包含模型以外的电机')
    groups = []
    for group in model.groups:
        members = {model.actuator_ids[name] for name in group.actuators}
        if members & wanted:
            if not members <= wanted:
                raise ValueError(f'传动组 {group.name} 必须完整接入')
            groups.append(group)
    document = yaml.safe_load(source.read_text())
    hardware = document['motor_hardware']
    hardware['motors'] = {name: value for name, value in hardware['motors'].items()
                          if model.actuator_ids[name] in wanted}
    if hardware.get('motor_specs_file'):
        hardware['motor_specs_file'] = str((source.parent / hardware['motor_specs_file']).resolve())
    target = Path(directory)
    hardware['transmissions_file'] = str(target / 'transmissions.yaml')
    transmissions = {group.name: {
        'plugin': group.plugin, 'actuators': list(group.actuators), 'joints': list(group.joints),
        'actuator_reduction': list(group.actuator_reduction),
        'joint_reduction': list(group.joint_reduction), 'joint_offset': list(group.joint_offset),
        'parameters': dict(group.parameters),
    } for group in groups}
    (target / 'transmissions.yaml').write_text(yaml.safe_dump({'transmissions': transmissions}))
    result = target / 'motor_driver.yaml'
    result.write_text(yaml.safe_dump(document))
    Model.load(str(result))
    return str(result)
