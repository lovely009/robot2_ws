"""Bind limit configuration to one complete scan and its exact policy inputs."""
import hashlib
import json

POLICY_PATHS=(
    'tools/derive_simulation_limits.py','tools/cad_mating.py','tools/scan_artifacts.py',
    'cad/assembly_audit/step-analytic-features.json','cad/assembly_audit/step-planes.json.gz',
    'cad/assembly_audit/2026-11-13-step-linkages.json',
    'cad/assembly_audit/2026-11-13-new-body-joint-axes.json',
)


def policy_hashes(directory):
    return {name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in POLICY_PATHS}


def validate(directory, limits):
    raw=(directory/'mujoco/geometry_limits.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=limits.get('scan_report_sha256'):
        raise ValueError('Scan JSON/YAML generation mismatch; regenerate simulation limits')
    report=json.loads(raw)
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
