#!/usr/bin/env python3
"""Check static rod attachments against analytic STEP balls and rod-end cylinders."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

MODEL = Path(__file__).resolve().parents[1]
SURFACE_TOL = .000025
CENTER_TOL = .000025


def load_parts():
    manifest = json.loads((MODEL / 'config/display_meshes.json').read_text())
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    frames = {e['source']: (name, e) for name, e in alignment['links'].items()}
    result = {}
    for entry in manifest['links'].values():
        if 'components' not in entry:
            continue
        source = MODEL.parent / manifest['source'] / 'meshes' / entry['source']
        if hashlib.sha256(source.read_bytes()).hexdigest() != entry['source_sha256']:
            raise ValueError(f'{source.name}: repeat component review after source changes')
        body, frame = frames[entry['source']]
        transform = np.asarray(frame['world_transform'])
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(source))
        reader.Update()
        regions = vtk.vtkPolyDataConnectivityFilter()
        regions.SetInputConnection(reader.GetOutputPort())
        regions.SetExtractionModeToAllRegions()
        regions.ColorRegionsOn()
        regions.Update()
        if regions.GetNumberOfExtractedRegions() != entry['source_region_count']:
            raise ValueError(f'{source.name}: source regions changed')
        mesh = regions.GetOutput()
        vertices = vtk_to_numpy(mesh.GetPoints().GetData()).astype(float)
        ids = vtk_to_numpy(mesh.GetPointData().GetScalars()).astype(int)
        for name, indices in entry['components'].items():
            points = vertices[np.isin(ids, indices)]
            world = points @ transform[:3, :3].T + transform[:3, 3]
            axis_points = world
            if name.endswith('_flange') and entry.get('rotor_regions'):
                side = 'proximal' if 'proximal' in name else 'distal'
                rotor = vertices[np.isin(ids, entry['rotor_regions'][side])]
                axis_points = np.vstack([world, rotor @ transform[:3, :3].T + transform[:3, 3]])
            result[name] = {'source_body': body, 'source_file': source.name, 'regions': indices,
                            'points': world, 'axis_points': axis_points}
    return result


def circle_support(points, origin, axis, radius):
    relative = points - origin
    radial = relative - np.outer(relative @ axis, axis)
    selected = np.abs(np.linalg.norm(radial, axis=1) - radius) < SURFACE_TOL
    if selected.sum() < 8:
        return None
    u = np.cross(axis, [1., 0., 0.] if abs(axis[0]) < .8 else [0., 1., 0.])
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    angles = np.sort(np.mod(np.arctan2(radial[selected] @ v, radial[selected] @ u), 2 * np.pi))
    gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])))
    if gap > np.deg2rad(45):
        return None
    return selected, float(np.rad2deg(gap))


def check_endpoint(rod, mate, features):
    low, high = rod.min(0) - .009, rod.max(0) + .009
    matches = []
    for sphere in features['spheres']:
        radius = sphere['radius_m']
        center = np.asarray(sphere['center_m'])
        if not .003 <= radius <= .01 or not np.all((low < center) & (center < high)):
            continue
        support = int(np.count_nonzero(np.abs(np.linalg.norm(mate - center, axis=1) - radius) < SURFACE_TOL))
        if support < 30:
            continue
        local = rod[np.linalg.norm(rod - center, axis=1) < .013]
        holes = []
        for cylinder in features['cylinders']:
            origin, axis = np.asarray(cylinder['origin_m']), np.asarray(cylinder['axis'])
            line_error = float(np.linalg.norm(np.cross(center - origin, axis)))
            if line_error > CENTER_TOL:
                continue
            circle = circle_support(local, origin, axis, cylinder['radius_m'])
            if circle is None:
                continue
            selected, gap = circle
            axial = (local[selected] - center) @ axis
            center_error = float(abs((axial.min() + axial.max()) / 2))
            if center_error > CENTER_TOL:
                continue
            holes.append({'axis': axis.tolist(), 'radius_m': cylinder['radius_m'],
                          'rod_support': int(selected.sum()), 'max_angular_gap_deg': gap,
                          'line_error_m': line_error, 'axial_center_error_m': center_error,
                          'axial_span_m': float(np.ptp(axial))})
        if holes:
            holes.sort(key=lambda h: (h['axial_center_error_m'] + h['line_error_m'], -h['rod_support']))
            matches.append({'center_m': center.tolist(), 'ball_radius_m': radius,
                            'mate_sphere_support': support, 'rod_end_cylinder': holes[0]})
    if len(matches) != 1:
        raise ValueError(f'Expected one common ball/rod-end center, found {len(matches)}')
    return matches[0]


def motor_axis_candidates(points, features, wrist):
    middle = points.mean(0)
    span = np.linalg.norm(np.ptp(points, axis=0)) / 2
    found = []
    for cylinder in features['cylinders']:
        if cylinder['radius_m'] < (.008 if wrist else .015):
            continue
        origin, axis = np.asarray(cylinder['origin_m']), np.asarray(cylinder['axis'])
        if np.linalg.norm(np.cross(middle - origin, axis)) > span:
            continue
        circle = circle_support(points, origin, axis, cylinder['radius_m'])
        if circle is None:
            continue
        selected, gap = circle
        if selected.sum() < 40:
            continue
        point = origin + axis * np.dot(middle - origin, axis)
        if any(abs(np.dot(axis, old['axis'])) > .99999 and
               np.linalg.norm(np.cross(point - np.asarray(old['point_on_axis_m']), axis)) < .000025
               for old in found):
            continue
        found.append({'axis': axis.tolist(), 'point_on_axis_m': point.tolist(),
                      'radius_m': cylinder['radius_m'], 'support': int(selected.sum()),
                      'max_angular_gap_deg': gap})
    return sorted(found, key=lambda item: -item['support'])


def audit():
    features = json.loads((MODEL / 'cad/assembly_audit/step-analytic-features.json').read_text())
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    if features['step_sha256'] != alignment['step_sha256']:
        raise ValueError('STEP feature cache is stale')
    step = MODEL.parent / '全身不带头.STEP'
    if hashlib.sha256(step.read_bytes()).hexdigest() != features['step_sha256']:
        raise ValueError('STEP source changed; repeat analytic feature extraction and assembly calibration')
    parts = load_parts()
    rods = {}
    legacy = {}
    legacy_spans = {}
    for group in ('waist', 'hips', 'left_shoulder', 'right_shoulder', 'left_wrist', 'right_wrist'):
        model = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco' / f'{group}_differential.xml'))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for tendon in range(model.ntendon):
            legacy[f'{group}_{model.tendon(tendon).name}'] = float(data.ten_length[tendon])
        if group in ('waist', 'hips'):
            legacy_spans[group] = {
                side: float(np.linalg.norm(data.site(f'plus_{side}_ball').xpos -
                                           data.site(f'minus_{side}_ball').xpos))
                for side in ('motor', 'platform')}
    for name, rod in parts.items():
        if not name.endswith('_rod'):
            continue
        prefix = name.removesuffix('_rod')
        endpoints = {side: check_endpoint(rod['points'], parts[f'{prefix}_{side}']['points'], features)
                     for side in ('flange', 'platform')}
        start, end = (np.asarray(endpoints[side]['center_m']) for side in ('flange', 'platform'))
        length = float(np.linalg.norm(end - start))
        axes = motor_axis_candidates(parts[f'{prefix}_flange']['axis_points'], features, 'wrist' in name)
        rods[name] = {'source_body': rod['source_body'], 'source_file': rod['source_file'],
                      'rod_regions': rod['regions'], 'endpoints': endpoints, 'center_distance_m': length,
                      'legacy_center_distance_m': legacy[name], 'legacy_length_difference_m': legacy[name] - length,
                      'motor_axis_candidates': axes}
    if len(rods) != 10:
        raise ValueError('Expected ten coupled rods')
    spans = {}
    for group in ('waist', 'hips'):
        spans[group] = {'comparison_note': (
            'Legacy hips uses different constructed anchors and assembly branch; these are definition differences, not a same-anchor dimensional error.'
            if group == 'hips' else
            'Legacy waist uses a nominal 100 mm span, with each CAD side shifted inward by 1 mm.')}
        for side, legacy_side in (('flange', 'motor'), ('platform', 'platform')):
            centers = [np.asarray(rods[f'{group}_{sign}_rod']['endpoints'][side]['center_m'])
                       for sign in ('plus', 'minus')]
            spans[group][side] = {'step_distance_m': float(np.linalg.norm(centers[0] - centers[1])),
                                  'legacy_distance_m': legacy_spans[group][legacy_side]}
    return {'scope': 'Static STEP attachment geometry only; no motion, encoder sign, clearance or interference acceptance',
            'method': 'STEP analytic ball center supported by mate mesh; rod-end cylinder shares its axis and axial center, with circumferential vertex support. Native STEP-aligned assembly frames; old ICP matrices are not used.',
            'source_step_sha256': features['step_sha256'], 'surface_tolerance_m': SURFACE_TOL,
            'center_tolerance_m': CENTER_TOL, 'bilateral_ball_spans': spans, 'rods': rods,
            'motor_axis_note': 'Geometric cylinder lines only; axis sign is arbitrary and is not an encoder direction.'}


def write_overlay(report):
    mounted = report.get('fixture_mounting', {}).get('after', {}).get('all_passed', False)
    filename = 'full_body_robot.hips-mounted.xml' if mounted else 'full_body_robot.step-aligned.xml'
    root = ET.parse(MODEL / 'mujoco' / filename).getroot()
    root.set('model', 'Fixture-mounted hips linkage audit (static)' if mounted else 'STEP linkage audit (static, orange centerlines)')
    corrections = json.loads((MODEL / 'config/hips_mount_alignment.json').read_text()) if mounted else None
    world = root.find('worldbody')
    ET.SubElement(root.find('asset'), 'material', name='audit_orange', rgba='1 .4 .05 1', emission='1')
    ET.SubElement(root.find('asset'), 'material', name='audit_blue', rgba='.1 .6 1 1', emission='1')
    for geom in world.findall('.//geom[@type="mesh"]'):
        geom.set('rgba', '.84 .85 .86 .12')
    for name, rod in report['rods'].items():
        endpoints = [rod['endpoints'][side]['center_m'] for side in ('flange', 'platform')]
        correction = None
        if mounted and name.startswith('hips_'):
            correction = corrections['sides'][name.split('_')[1]]
            endpoints = [
                (np.asarray(correction[tag + '_transform'])[:3, :3] @ np.asarray(center) +
                 np.asarray(correction[tag + '_transform'])[:3, 3]).tolist()
                for tag, center in zip(('flange', 'platform'), endpoints)]
        for side, center in zip(('flange', 'platform'), endpoints):
            ET.SubElement(world, 'site', name=f'{name}_{side}_ball', pos=' '.join(map(str, center)),
                          size='.003', rgba='0.1 1 .3 1')
        ET.SubElement(world, 'geom', name=f'{name}_centerline', type='capsule',
                      fromto=' '.join(map(str, [*endpoints[0], *endpoints[1]])), size='.0014',
                      material='audit_orange', contype='0', conaffinity='0', mass='0')
        if len(rod['motor_axis_candidates']) == 1:
            candidate = rod['motor_axis_candidates'][0]
            center = np.asarray(candidate['point_on_axis_m'])
            axis = np.asarray(candidate['axis'])
            if correction is not None:
                transform = np.asarray(correction['flange_transform'])
                center = transform[:3, :3] @ center + transform[:3, 3]
                axis = transform[:3, :3] @ axis
            ET.SubElement(world, 'geom', name=f'{name}_motor_axis', type='capsule',
                          fromto=' '.join(map(str, [*(center - .02 * axis), *(center + .02 * axis)])),
                          size='.0009', material='audit_blue', contype='0', conaffinity='0', mass='0')
    ET.indent(root, space='  ')
    path = MODEL / 'mujoco/full_body_robot.linkage-audit.xml'
    path.write_text(ET.tostring(root, encoding='unicode') + '\n')
    return path


if __name__ == '__main__':
    report = audit()
    if (MODEL / 'config/hips_mount_alignment.json').exists():
        import hips_mounting
        calibration = json.loads((MODEL / 'config/hips_mount_alignment.json').read_text())
        if calibration['step_sha256'] != report['source_step_sha256']:
            raise ValueError('Hips installation calibration is stale')
        after = hips_mounting.validate(MODEL / 'mujoco/full_body_robot.hips-mounted.xml', calibration)
        if not after['all_passed']:
            raise ValueError('Hips fixtures do not meet installation criteria')
        report['fixture_mounting'] = {
            'after': after,
            'scope': 'Hips receiver holes and seat faces checked; other groups still have rod-end checks only.'}
    destination = MODEL / 'cad/assembly_audit/2026-11-13-step-linkages.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(destination)
    print(write_overlay(report))
