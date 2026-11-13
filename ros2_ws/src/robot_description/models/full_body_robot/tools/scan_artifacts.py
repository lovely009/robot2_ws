"""Bind limit configuration to one complete scan and its exact policy inputs."""
import hashlib
import json
import yaml
import xml.etree.ElementTree as ET

POLICY_PATHS=(
    'config/step_alignment.json','mujoco/full_body_robot.physics.xml',
    'config/motor_topology.yaml','config/native_assembly.json','config/arm_mount_alignment.json',
    'tools/derive_simulation_limits.py','tools/cad_mating.py','tools/scan_artifacts.py',
    'cad/assembly_audit/step-analytic-features.json','cad/assembly_audit/step-planes.json.gz',
    'cad/assembly_audit/2026-11-13-step-linkages.json',
    'cad/assembly_audit/2026-11-13-new-body-joint-axes.json',
)


def policy_hashes(directory):
    hashes={}
    for name in POLICY_PATHS:
        raw=(directory/name).read_bytes()
        if name=='config/native_assembly.json':
            # This status is a scan output; all assembly geometry remains bound.
            assembly=json.loads(raw);assembly.pop('collision_validation',None)
            raw=json.dumps(assembly,sort_keys=True,separators=(',',':')).encode()
        if name=='mujoco/full_body_robot.physics.xml':
            # Preserve kinematics, controls and linkage mapping, excluding scan outputs.
            root=ET.fromstring(raw)
            for joint in root.findall('.//joint'):
                joint.attrib.pop('range',None);joint.attrib.pop('limited',None)
            for actuator in root.findall('actuator/position'):actuator.attrib.pop('ctrlrange',None)
            for text in root.findall("custom/text[@name='robot_sim_joint_position']"):
                config=json.loads(text.get('data'))
                config.pop('joint_limits',None);config.pop('coupled_limits',None)
                text.set('data',json.dumps(config,sort_keys=True,separators=(',',':')))
            keyframe=root.find('keyframe')
            if keyframe is not None:root.remove(keyframe)
            raw=ET.tostring(root)
        hashes[name]=hashlib.sha256(raw).hexdigest()
    return hashes


def validate_groups(directory,report):
    expected=yaml.safe_load((directory/'config/motor_topology.yaml').read_text())['differential_groups']
    groups=report['differential_groups']
    if len(expected)!=6 or set(groups)!=set(expected):
        raise ValueError('Incomplete coupled groups: require all six topology groups')
    for name,group in groups.items():
        if group['joints']!=expected[name]['joints'] or group['coupled_workspace']['joints']!=expected[name]['joints']:
            raise ValueError('Coupled joint pairing differs from topology: '+name)


def validate(directory, limits):
    raw=(directory/'mujoco/geometry_limits.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=limits.get('scan_report_sha256'):
        raise ValueError('Scan JSON/YAML generation mismatch; regenerate simulation limits')
    report=json.loads(raw)
    validate_groups(directory,report)
    current=policy_hashes(directory)
    if report.get('policy_inputs_sha256')!=current:
        raise ValueError('Scan policy or mating evidence changed; regenerate simulation limits')
    if (report.get('scan_script_sha256')!=current['tools/derive_simulation_limits.py']
            or report.get('mating_script_sha256')!=current['tools/cad_mating.py']):
        raise ValueError('Stale scanner/mating policy hashes')
    for key in ('joint_limits','actuator_limits'):
        actual={name:[bound['lower'],bound['upper']] for name,bound in limits[key].items()}
        if actual!=report[key]:raise ValueError('Scan JSON/YAML '+key+' mismatch')
    expected={name:group['coupled_workspace'] for name,group in report['differential_groups'].items()}
    if limits['coupled_limits']!=expected:raise ValueError('Scan JSON/YAML coupled mask mismatch')
    return report
