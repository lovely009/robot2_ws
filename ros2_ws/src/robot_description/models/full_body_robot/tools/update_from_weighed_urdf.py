#!/usr/bin/env python3
"""Update full_body_robot URDF/configs from complete_robot_combine with measured collision limits and motor specs."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

PKG_ROOT = Path(__file__).resolve().parents[3]
SRC_URDF = PKG_ROOT / 'models/complete_robot_combine/urdf/complete_robot.urdf'
DST_MODEL = PKG_ROOT / 'models/full_body_robot'

TEMPLATE_SPECS = {
    'cerb_r58_101': {'effort': 45.0, 'velocity': 3.1416},
    'cerb_r48_101': {'effort': 11.0, 'velocity': 3.1416},
    'vaxor_2550': {'effort': 1.15, 'velocity': 4.189},
}
MAX_SCAN_RAD = math.radians(178.0)  # 3.106686 rad
COARSE_STEP_RAD = math.radians(2.0)
CLEARANCE_RAD = math.radians(1.0)


def load_joint_specs() -> dict[str, dict[str, float]]:
    params = yaml.safe_load((DST_MODEL / 'config/motor_params.yaml').read_text(encoding='utf-8'))
    out: dict[str, dict[str, float]] = {}
    for entry in params['motors']:
        out[entry['joint']] = TEMPLATE_SPECS[entry['template']]
    return out


def rewrite_urdf_base() -> ET.ElementTree:
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()
    root.attrib['name'] = 'full_body_robot'
    specs = load_joint_specs()

    for mesh in root.findall('.//mesh'):
        fn = mesh.attrib['filename']
        if fn.startswith('package://complete_robot/meshes/collision/'):
            name = fn.split('/')[-1]
            mesh.attrib['filename'] = f'package://robot_description/models/full_body_robot/meshes/collision/native/{name}'
        elif fn.startswith('package://complete_robot/meshes/'):
            name = fn.split('/')[-1]
            mesh.attrib['filename'] = f'package://robot_description/models/full_body_robot/meshes/visual/native/{name}'
        else:
            raise ValueError(f'Unexpected mesh prefix: {fn}')

    for joint in root.findall('joint'):
        jname = joint.attrib['name']
        spec = specs[jname]
        limit = joint.find('limit')
        if limit is None:
            limit = ET.SubElement(joint, 'limit')
        limit.attrib['effort'] = str(spec['effort'])
        limit.attrib['velocity'] = str(spec['velocity'])
        limit.attrib['lower'] = str(round(-MAX_SCAN_RAD, 6))
        limit.attrib['upper'] = str(round(MAX_SCAN_RAD, 6))

    return tree


def scan_collision_limits(urdf_path: Path) -> dict[str, tuple[float, float]]:
    model, collision_model, _ = pin.buildModelsFromUrdf(
        str(urdf_path),
        package_dirs=[str(PKG_ROOT.parent)],
    )
    collision_model.addAllCollisionPairs()

    data = model.createData()
    geom_data = collision_model.createData()
    q0 = pin.neutral(model)
    pin.updateGeometryPlacements(model, data, collision_model, geom_data, q0)

    valid_pairs: list[tuple[int, int, int]] = []
    baseline_removed = 0
    for k, cp in enumerate(collision_model.collisionPairs):
        j1 = collision_model.geometryObjects[cp.first].parentJoint
        j2 = collision_model.geometryObjects[cp.second].parentJoint
        if j1 == j2 or model.parents[j1] == j2 or model.parents[j2] == j1:
            continue
        if pin.computeCollision(collision_model, geom_data, k):
            baseline_removed += 1
            continue
        valid_pairs.append((k, j1, j2))

    print(f'Collision model: {len(collision_model.geometryObjects)} objects, '
          f'{len(valid_pairs)} active pairs ({baseline_removed} baseline pairs exempted)')

    # Precompute descendant joint subtree for each joint
    children: dict[int, list[int]] = {}
    for jid in range(1, model.njoints):
        children.setdefault(model.parents[jid], []).append(jid)

    def get_subtree(root_jid: int) -> set[int]:
        sub = {root_jid}
        stack = [root_jid]
        while stack:
            cur = stack.pop()
            for ch in children.get(cur, ()):
                sub.add(ch)
                stack.append(ch)
        return sub

    limits: dict[str, tuple[float, float]] = {}
    for jid in range(1, model.njoints):
        jname = model.names[jid]
        q_idx = model.idx_qs[jid]
        subtree = get_subtree(jid)
        pair_indices = [k for k, j1, j2 in valid_pairs if (j1 in subtree) != (j2 in subtree)]

        def collides(q: np.ndarray) -> bool:
            pin.updateGeometryPlacements(model, data, collision_model, geom_data, q)
            return any(pin.computeCollision(collision_model, geom_data, k) for k in pair_indices)

        assert not collides(q0), f'Baseline collision still active for {jname}'

        def sweep(sign: float) -> float:
            last_free = 0.0
            step = COARSE_STEP_RAD
            ang = step
            hit = None
            while ang <= MAX_SCAN_RAD + 1e-9:
                q = q0.copy()
                q[q_idx] = sign * min(ang, MAX_SCAN_RAD)
                if collides(q):
                    hit = min(ang, MAX_SCAN_RAD)
                    break
                last_free = min(ang, MAX_SCAN_RAD)
                ang += step
            if hit is None:
                return MAX_SCAN_RAD
            lo, hi = last_free, hit
            for _ in range(10):
                mid = 0.5 * (lo + hi)
                q = q0.copy()
                q[q_idx] = sign * mid
                if collides(q):
                    hi = mid
                else:
                    lo = mid
            return max(0.0, lo - CLEARANCE_RAD)

        neg = -sweep(-1.0)
        pos = sweep(+1.0)
        limits[jname] = (round(neg, 6), round(pos, 6))

    return limits


def main() -> int:
    tree = rewrite_urdf_base()
    tmp_urdf = DST_MODEL / 'urdf/full_body_robot.urdf'
    ET.indent(tree, space='  ')
    tree.write(tmp_urdf, encoding='utf-8', xml_declaration=False)

    limits = scan_collision_limits(tmp_urdf)
    hil_limits = yaml.safe_load(
        (DST_MODEL / 'config/coupled_limits.hil.yaml').read_text(encoding='utf-8')
    ).get('joint_limits', {})
    root = tree.getroot()
    specs = load_joint_specs()

    print('\n=== 37-JOINT MEASURED LIMITS (MOST-PERMISSIVE ENVELOPE) & MOTOR SPECS ===')
    for joint in root.findall('joint'):
        jname = joint.attrib['name']
        lo, hi = limits[jname]
        if jname in hil_limits:
            h_lo, h_hi = float(hil_limits[jname][0]), float(hil_limits[jname][1])
            lo = round(min(lo, h_lo), 6)
            hi = round(max(hi, h_hi), 6)
            limits[jname] = (lo, hi)
        lim = joint.find('limit')
        lim.attrib['lower'] = str(lo)
        lim.attrib['upper'] = str(hi)
        s = specs[jname]
        print(f'  {jname:22s}: [{math.degrees(lo):+7.2f}°, {math.degrees(hi):+7.2f}°] '
              f'({lo:+.4f}, {hi:+.4f} rad) | effort={s["effort"]:5.2f} Nm | vel={s["velocity"]:.4f} rad/s')

    ET.indent(tree, space='  ')
    for rel in (
        'urdf/full_body_robot.urdf',
        'urdf/full_body_robot.standing.urdf',
        'urdf/full_body_robot.standalone.urdf',
        'urdf/full_body_robot.mapping.urdf',
    ):
        tree.write(DST_MODEL / rel, encoding='utf-8', xml_declaration=False)

    # Update standing.json, simulation_limits.yaml, motion_preview_limits.yaml
    joint_names = [j.attrib['name'] for j in root.findall('joint')]
    (DST_MODEL / 'config/standing.json').write_text(
        json.dumps({name: 0.0 for name in joint_names}, indent=2) + '\n', encoding='utf-8')

    sim_limits = {
        'joint_limits': {name: {'lower': limits[name][0], 'upper': limits[name][1]} for name in joint_names},
    }
    (DST_MODEL / 'config/simulation_limits.yaml').write_text(
        yaml.safe_dump(sim_limits, sort_keys=False), encoding='utf-8')
    (DST_MODEL / 'config/motion_preview_limits.yaml').write_text(
        yaml.safe_dump(sim_limits, sort_keys=False), encoding='utf-8')

    # Verify Pinocchio mass & gravity RNEA
    model = pin.buildModelFromUrdf(str(tmp_urdf))
    data = model.createData()
    q0 = pin.neutral(model)
    tau = pin.rnea(model, data, q0, np.zeros(model.nv), np.zeros(model.nv))
    tot_mass = sum(inert.mass for inert in model.inertias)
    sha = hashlib.sha256(tmp_urdf.read_bytes()).hexdigest()
    print(f'\nPinocchio check OK: nq={model.nq}, total_mass={tot_mass:.4f} kg, '
          f'max_g_tau={np.max(np.abs(tau)):.3f} Nm, finite={np.all(np.isfinite(tau))}')
    print(f'URDF sha256: {sha}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
