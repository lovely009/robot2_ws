#!/usr/bin/env python3
"""Reassemble floating hips hardware onto measured receiver holes; static only."""
from pathlib import Path
import hashlib
import importlib.util
import json
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import vtk
from vtk.util.numpy_support import vtk_to_numpy

import audit_step_linkages as audit

MODEL = Path(__file__).resolve().parents[1]
OUTPUT = MODEL / 'mujoco/full_body_robot.hips-mounted.xml'
CONFIG = MODEL / 'config/hips_mount_alignment.json'
ROLES = {
    'plus': {'flange_outer': 18, 'platform_outer': 2, 'platform_inner': 8, 'flange_inner': 4, 'crank': 3},
    'minus': {'flange_outer': 7, 'platform_outer': 6, 'platform_inner': 1, 'flange_inner': 5, 'crank': 0},
}


def point(transform, value):
    return transform[:3, :3] @ np.asarray(value) + transform[:3, 3]


def direction_frame(direction, normal):
    z = direction / np.linalg.norm(direction)
    x = normal - np.dot(normal, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def rigid(rotation, old_center, new_center):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = new_center - rotation @ old_center
    return transform


def load_geometry():
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    manifest = json.loads((MODEL / 'config/display_meshes.json').read_text())
    features = json.loads((MODEL / 'cad/assembly_audit/step-analytic-features.json').read_text())
    step = MODEL / 'cad/全身装配体（不含头部）.STEP'
    if not step.exists():
        step = MODEL.parent / '全身装配体（不含头部）.STEP'
    if hashlib.sha256(step.read_bytes()).hexdigest() != features['step_sha256'] or features['step_sha256'] != alignment['step_sha256']:
        raise ValueError('Repeat STEP feature extraction after changing the source')
    base = manifest['links']['base_link']
    source = MODEL.parent / 'Robot_body1/meshes' / base['source']
    if hashlib.sha256(source.read_bytes()).hexdigest() != base['source_sha256']:
        raise ValueError('Review hips source regions after changing the mesh')
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    reader.Update()
    regions = vtk.vtkPolyDataConnectivityFilter()
    regions.SetInputConnection(reader.GetOutputPort())
    regions.SetExtractionModeToAllRegions()
    regions.ColorRegionsOn()
    regions.Update()
    if regions.GetNumberOfExtractedRegions() != base['source_region_count']:
        raise ValueError('Hips source region count changed')
    vertices = vtk_to_numpy(regions.GetOutput().GetPoints().GetData()).astype(float)
    ids = vtk_to_numpy(regions.GetOutput().GetPointData().GetScalars()).astype(int)
    base_frame = np.asarray(alignment['links']['Base_link']['world_transform'])
    world = vertices @ base_frame[:3, :3].T + base_frame[:3, 3]
    receiver = alignment['links']['BN_hips_pitch_link']
    plate_path = MODEL.parent / 'Robot_body1/meshes' / receiver['source']
    if hashlib.sha256(plate_path.read_bytes()).hexdigest() != receiver['source_sha256']:
        raise ValueError('Receiver mesh changed')
    plate_reader = vtk.vtkSTLReader()
    plate_reader.SetFileName(str(plate_path))
    plate_reader.Update()
    plate = vtk_to_numpy(plate_reader.GetOutput().GetPoints().GetData()).astype(float)
    receiver_frame = np.asarray(receiver['world_transform'])
    plate = plate @ receiver_frame[:3, :3].T + receiver_frame[:3, 3]
    return alignment, base, features, regions, vertices, ids, world, plate


def inner_axis(ball, center, features):
    candidates = []
    for cylinder in features['cylinders']:
        if abs(cylinder['radius_m'] - .004) > 1e-8:
            continue
        axis, origin = np.asarray(cylinder['axis']), np.asarray(cylinder['origin_m'])
        axis = axis if axis[2] > 0 else -axis
        if np.linalg.norm(np.cross(center - origin, axis)) > audit.CENTER_TOL:
            continue
        support = audit.circle_support(ball, origin, axis, .004)
        if support is not None:
            candidates.append((int(support[0].sum()), axis))
    if not candidates:
        raise ValueError('No supported 8 mm inner-ring bore')
    axis = max(candidates, key=lambda item: item[0])[1]
    return axis, float(-np.min((ball - center) @ axis))


def receiver_hole(plate, source_center, features, side, require_rim=True):
    holes = []
    for cylinder in features['cylinders']:
        if abs(cylinder['radius_m'] - .0025) > 1e-8:
            continue
        origin, axis = np.asarray(cylinder['origin_m']), np.asarray(cylinder['axis'])
        axis = axis if axis[2] > 0 else -axis
        if axis[2] < .95:
            continue
        # A finite neighbourhood is essential: other surfaces can lie on the
        # same infinite cylinder axis elsewhere in this link.
        local = plate[np.linalg.norm(plate - origin, axis=1) < .012]
        bore = audit.circle_support(local, origin, axis, .0025)
        rim = audit.circle_support(local, origin, axis, .005)
        if bore is None or (require_rim and rim is None):
            continue
        bore_stations = (local[bore[0]] - origin) @ axis
        top_points = local[bore[0]][bore_stations > np.max(bore_stations) - .000001]
        top_circle = audit.circle_support(top_points, origin, axis, .0025)
        if top_circle is None:
            continue
        station = float(np.median((top_points - origin) @ axis))
        stations = (local[rim[0]] - origin) @ axis if rim is not None else bore_stations
        if require_rim and (np.ptp(stations) > .006 or abs(np.max(stations) - station) > audit.CENTER_TOL):
            continue
        seat = origin + station * axis
        if (seat[0] > 0) != (side == 'plus'):
            continue
        if any(np.linalg.norm(seat - old['seat_center_m']) < audit.CENTER_TOL for old in holes):
            continue
        holes.append({'seat_center_m': seat, 'axis': axis, 'axis_origin_m': origin,
                      'plate_thickness_m': float(np.ptp(stations)),
                      'bore_support': int(bore[0].sum()), 'seat_rim_support': int(rim[0].sum()) if rim is not None else 0})
    holes.sort(key=lambda hole: np.linalg.norm(source_center - hole['seat_center_m']))
    if not holes:
        raise ValueError('No supported receiver hole and finite seating rim')
    if len(holes) > 1:
        difference = np.linalg.norm(source_center - holes[1]['seat_center_m']) - np.linalg.norm(source_center - holes[0]['seat_center_m'])
        if difference < .001:
            raise ValueError('Ambiguous receiver holes')
    return holes[0]


def solve():
    alignment, base, features, _, _, ids, world, plate = load_geometry()
    raw = audit.audit()
    solutions = {}
    for side, roles in ROLES.items():
        rod = raw['rods'][f'hips_{side}_rod']
        start, end = (np.asarray(rod['endpoints'][tag]['center_m']) for tag in ('flange', 'platform'))
        inner = world[ids == roles['platform_inner']]
        old_axis, height = inner_axis(inner, end, features)
        flange_axis, flange_height = inner_axis(world[ids == roles['flange_inner']], start, features)
        flange_hole = receiver_hole(world[ids == roles['crank']], start, features, side, require_rim=False)
        if abs(height - .004) > audit.CENTER_TOL:
            raise ValueError('Review the inner-ring seating datum')
        hole = receiver_hole(plate, end, features, side)
        axis, seat = hole['axis'], hole['seat_center_m']
        target = seat + height * axis
        platform_rotation = Rotation.align_vectors([axis], [old_axis])[0].as_matrix()
        platform = rigid(platform_rotation, end, target)
        motor = rod['motor_axis_candidates']
        if len(motor) != 1:
            raise ValueError('Ambiguous motor axis')
        motor_axis = np.asarray(motor[0]['axis'])
        motor_axis = motor_axis if motor_axis[2] > 0 else -motor_axis
        origin = np.asarray(motor[0]['point_on_axis_m'])
        origin += np.dot(start - origin, motor_axis) * motor_axis
        radius = start - origin
        delta = target - origin
        length = np.linalg.norm(end - start)
        u, w = np.dot(delta, radius), np.dot(delta, np.cross(motor_axis, radius))
        cosine = (np.dot(delta, delta) + np.dot(radius, radius) - length**2) / (2 * np.hypot(u, w))
        if abs(cosine) > 1 + 1e-10:
            raise ValueError('No rigid-rod assembly solution')
        alpha, beta = np.arctan2(w, u), np.arccos(np.clip(cosine, -1., 1.))
        choices = []
        for angle in (alpha + beta, alpha - beta):
            rotation = Rotation.from_rotvec(motor_axis * angle).as_matrix()
            new_start = origin + rotation @ radius
            if (new_start[0] - origin[0] > 0) == (side == 'plus'):
                choices.append((angle, rotation, new_start))
        if len(choices) != 1:
            raise ValueError('Ambiguous outward crank branch')
        angle, flange_rotation, new_start = choices[0]
        flange = rigid(flange_rotation, origin, origin)
        outer_axis = np.asarray(rod['endpoints']['flange']['rod_end_cylinder']['axis'])
        outer_axis = outer_axis if outer_axis[2] > 0 else -outer_axis
        rotation = direction_frame(target - new_start, motor_axis + axis) @ direction_frame(end - start, outer_axis).T
        rod_pose = rigid(rotation, start, new_start)
        original_parts = base['components']
        prefix = f'hips_{side}'
        groups = {
            'flange': [i for i in original_parts[prefix + '_flange'] if i != roles['flange_outer']],
            'platform': [i for i in original_parts[prefix + '_platform'] if i != roles['platform_outer']],
            'rod': [*original_parts[prefix + '_rod'], roles['flange_outer'], roles['platform_outer']],
        }
        race_checks = {}
        for endpoint, race, center in (('flange', roles['flange_outer'], start),
                                       ('platform', roles['platform_outer'], end)):
            if np.count_nonzero(ids == race) != 992:
                raise ValueError('Review the spherical-bearing outer-race classification')
            race_points = world[ids == race]
            race_axis = np.asarray(rod['endpoints'][endpoint]['rod_end_cylinder']['axis'])
            support = audit.circle_support(race_points, center, race_axis, .008)
            width = float(np.ptp((race_points - center) @ race_axis))
            if support is None or abs(width - .005) > audit.CENTER_TOL:
                raise ValueError('Outer race does not fit the rod-end 16 mm bore and 5 mm width')
            race_checks[endpoint] = {'source_region': race, 'outer_radius_m': .008,
                                     'measured_width_m': width, 'cylinder_support': int(support[0].sum()),
                                     'max_angular_gap_deg': support[1], 'owner': 'rod'}
        solutions[side] = {'raw_flange_ball_m': start.tolist(), 'raw_platform_ball_m': end.tolist(),
                           'inner_source_axis': old_axis.tolist(), 'inner_seat_height_m': height,
                           'hole_axis': axis.tolist(), 'hole_axis_origin_m': hole['axis_origin_m'].tolist(),
                           'seat_center_m': seat.tolist(), 'target_ball_m': target.tolist(),
                           'new_flange_ball_m': new_start.tolist(), 'motor_angle_rad': float(angle),
                           'rod_length_m': float(length), 'receiver_thickness_m': hole['plate_thickness_m'],
                           'flange_transform': flange.tolist(), 'platform_transform': platform.tolist(),
                           'rod_transform': rod_pose.tolist(), 'regions': groups,
                           'rod_source_hole_axis': outer_axis.tolist(),
                           'motor_source_axis': motor_axis.tolist(),
                           'flange_inner_source_axis': flange_axis.tolist(),
                           'flange_inner_seat_height_m': flange_height,
                           'flange_seat_source_m': flange_hole['seat_center_m'].tolist(),
                           'flange_hole_source_axis': flange_hole['axis'].tolist(),
                           'outer_race_checks': race_checks}
    return {'mode': 'static fixture-mounted reconstruction; original STEP remains unchanged',
            'step_sha256': features['step_sha256'], 'base_mesh_sha256': base['source_sha256'],
            'base_frame_world': alignment['links']['Base_link']['world_transform'],
            'receiver_frame_world': alignment['links']['BN_hips_pitch_link']['world_transform'],
            'bearing_roles': ROLES, 'sides': solutions}


def expected_groups(base, side):
    roles = ROLES[side]
    source = base['components']
    prefix = f'hips_{side}'
    return {
        'flange': [i for i in source[prefix + '_flange'] if i != roles['flange_outer']],
        'platform': [i for i in source[prefix + '_platform'] if i != roles['platform_outer']],
        'rod': [*source[prefix + '_rod'], roles['flange_outer'], roles['platform_outer']],
    }


def mesh_builder():
    spec = importlib.util.spec_from_file_location('full_body_builder', MODEL / 'tools/build_mujoco_model.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper


def build(solution, output=OUTPUT, mesh_directory=None, config_path=CONFIG):
    alignment, base, _, regions, _, _, _, _ = load_geometry()
    helper = mesh_builder()
    root = ET.parse(MODEL / 'mujoco/full_body_robot.step-aligned.xml').getroot()
    root.set('model', 'STEP assembly with hips mounted (static)')
    root.find('statistic').set('center', '0 .055 .035')
    root.find('statistic').set('extent', '.23')
    root.find('visual/global').set('azimuth', '65')
    root.find('visual/global').set('elevation', '-30')
    for side in ROLES:
        if solution['sides'][side]['regions'] != expected_groups(base, side):
            raise ValueError('Hips bearing regions have incorrect owners')
    moved = [i for side in solution['sides'].values() for indices in side['regions'].values() for i in indices]
    original = [i for name, indices in base['components'].items() if name.startswith('hips_') for i in indices]
    if len(set(moved)) != len(moved) or set(moved) != set(original):
        raise ValueError('Hips split loses or duplicates source components')
    fixed = sorted(set(range(base['source_region_count'])) - set(moved))
    regions.SetExtractionModeToSpecifiedRegions()
    output = Path(output)
    output_dir = Path(mesh_directory) if mesh_directory is not None else MODEL / 'mujoco/visual/hips_mounted'
    if output != OUTPUT:
        root.find('compiler').set('meshdir', str(MODEL))

    def write(indices, name):
        regions.InitializeSpecifiedRegionList()
        for index in indices:
            regions.AddSpecifiedRegion(index)
        clean = vtk.vtkCleanPolyData()
        clean.SetInputConnection(regions.GetOutputPort())
        clean.Update()
        path = output_dir / f'{name}.STL'
        helper.write_display_mesh(clean.GetOutput(), path)
        return path.relative_to(MODEL).as_posix() if path.is_relative_to(MODEL) else str(path.resolve())

    base_body = root.find(".//body[@name='Base_link']")
    old_asset = base_body.find('geom').get('mesh')
    root.find(f"asset/mesh[@name='{old_asset}']").set('file', write(fixed, 'Base_link_fixed'))
    base_frame = np.asarray(solution['base_frame_world'])
    for side, entry in solution['sides'].items():
        for part, indices in entry['regions'].items():
            name = f'hips_{side}_{part}_mounted'
            parent_name = 'BN_hips_pitch_link' if part == 'platform' else 'Base_link'
            parent_frame = np.asarray(alignment['links'][parent_name]['world_transform'])
            pose = np.linalg.inv(parent_frame) @ np.asarray(entry[part + '_transform']) @ base_frame
            body = ET.SubElement(root.find(f".//body[@name='{parent_name}']"), 'body', name=name,
                                 **helper.pose_attributes(pose[:3, 3], Rotation.from_matrix(pose[:3, :3])))
            ET.SubElement(root.find('asset'), 'mesh', name=name, file=write(indices, name))
            ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, mass='0',
                          contype='0', conaffinity='0', rgba='.84 .85 .86 1')
    ET.indent(root, space='  ')
    output.write_text(ET.tostring(root, encoding='unicode') + '\n')
    if config_path is not None:
        Path(config_path).write_text(json.dumps(solution, ensure_ascii=False, indent=2) + '\n')
    return output


def verify_mesh(root, path, body_name, expected_vertices, mounted=False):
    """Check actual XML binding and STL content, including outer-race ownership."""
    body = root.find(f".//body[@name='{body_name}']")
    geoms = body.findall('geom')
    if len(geoms) != 1:
        raise ValueError(f'{body_name}: unexpected mesh binding')
    geom = geoms[0]
    if mounted and (geom.get('name') != body_name or geom.get('mesh') != body_name):
        raise ValueError(f'{body_name}: incorrect mesh binding')
    if any(key in geom.attrib for key in ('pos', 'quat', 'euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto')):
        raise ValueError(f'{body_name}: unexpected geom transform')
    asset = root.find(f"asset/mesh[@name='{geom.get('mesh')}']")
    if asset is None or not np.allclose(np.fromstring(asset.get('scale', '1 1 1'), sep=' '), 1):
        raise ValueError(f'{body_name}: missing or scaled mesh binding')
    directory = Path(root.find('compiler').get('meshdir', ''))
    if not directory.is_absolute():
        directory = Path(path).parent / directory
    mesh_path = directory / asset.get('file')
    if not mesh_path.is_file():
        raise ValueError(f'{body_name}: missing mesh file')
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh_path))
    reader.Update()
    if not reader.GetOutput().GetNumberOfPoints():
        raise ValueError(f'{body_name}: empty mesh')
    actual_vertices = vtk_to_numpy(reader.GetOutput().GetPoints().GetData()).astype(float)
    error = max(cKDTree(actual_vertices).query(expected_vertices)[0].max(),
                cKDTree(expected_vertices).query(actual_vertices)[0].max())
    if error > 1e-7:
        raise ValueError(f'{body_name}: mesh content differs from its source regions ({error} m)')


def fixed_reference_vertices(base, regions):
    """Rebuild only the independently selected fixed source regions.

    Base exceeds MuJoCo's 200k STL face limit. Compare the canonical display
    representation, rather than relaxing tolerance for its existing decimation.
    """
    moved = {i for side in ROLES for group in expected_groups(base, side).values() for i in group}
    regions.SetExtractionModeToSpecifiedRegions()
    regions.InitializeSpecifiedRegionList()
    for index in sorted(set(range(base['source_region_count'])) - moved):
        regions.AddSpecifiedRegion(index)
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(regions.GetOutputPort())
    clean.Update()
    with tempfile.TemporaryDirectory(prefix='hips-fixed-reference-') as directory:
        filename = Path(directory) / 'fixed.STL'
        mesh_builder().write_display_mesh(clean.GetOutput(), filename)
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(filename))
        reader.Update()
        return vtk_to_numpy(reader.GetOutput().GetPoints().GetData()).astype(float)


def validate(path, solution=None):
    """Measure fixtures anew from source geometry, independent of solver targets.

    ``solution`` is accepted for existing callers, but supplies no acceptance datum.
    The actual model meshes must also contain the correct source components.
    """
    alignment, base, features, regions, vertices, ids, world, plate = load_geometry()
    raw = audit.audit()
    root = ET.parse(path).getroot()
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.njnt or model.nu or model.neq:
        raise ValueError('Expected a static inspection model')
    base_frame = np.asarray(alignment['links']['Base_link']['world_transform'])
    receiver_frame = np.asarray(alignment['links']['BN_hips_pitch_link']['world_transform'])
    receiver = data.body('BN_hips_pitch_link')
    actual_receiver = rigid(receiver.xmat.reshape(3, 3), np.zeros(3), receiver.xpos)
    receiver_delta = actual_receiver @ np.linalg.inv(receiver_frame)
    receiver_vertices = (plate - receiver_frame[:3, 3]) @ receiver_frame[:3, :3]
    verify_mesh(root, path, 'BN_hips_pitch_link', receiver_vertices)
    part_names = [f'hips_{side}_{part}_mounted' for side in ROLES for part in ('flange', 'platform', 'rod')]
    present = sum(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0 for name in part_names)
    if present not in (0, 6):
        raise ValueError('Incomplete hips mounting bodies')
    if present:
        verify_mesh(root, path, 'Base_link', fixed_reference_vertices(base, regions))
    base_body = data.body('Base_link')
    base_delta = rigid(base_body.xmat.reshape(3, 3), np.zeros(3), base_body.xpos) @ np.linalg.inv(base_frame)
    results = {}
    for side, roles in ROLES.items():
        reference = raw['rods'][f'hips_{side}_rod']
        centers = [np.asarray(reference['endpoints'][tag]['center_m']) for tag in ('flange', 'platform')]
        source_axis, height = inner_axis(world[ids == roles['platform_inner']], centers[1], features)
        source_flange_axis, flange_height = inner_axis(world[ids == roles['flange_inner']], centers[0], features)
        hole = receiver_hole(plate, centers[1], features, side)
        flange_hole = receiver_hole(world[ids == roles['crank']], centers[0], features, side, require_rim=False)
        if present:
            for part, indices in expected_groups(base, side).items():
                verify_mesh(root, path, f'hips_{side}_{part}_mounted', vertices[np.isin(ids, indices)], mounted=True)
        def actual(part):
            name = f'hips_{side}_{part}_mounted'
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
                name = 'Base_link'
            body = data.body(name)
            return rigid(body.xmat.reshape(3, 3), np.zeros(3), body.xpos) @ np.linalg.inv(base_frame)
        platform, flange, rod = (actual(tag) for tag in ('platform', 'flange', 'rod'))
        ball = point(platform, centers[1])
        shaft_axis = platform[:3, :3] @ source_axis
        seat = point(receiver_delta, hole['seat_center_m'])
        hole_axis = receiver_delta[:3, :3] @ hole['axis']
        shaft_axis = shaft_axis if np.dot(shaft_axis, hole_axis) > 0 else -shaft_axis
        line_error = np.linalg.norm(np.cross(ball - seat, hole_axis))
        seat_error = abs(np.dot(ball - shaft_axis * height - seat, hole_axis))
        angle = np.arccos(np.clip(abs(np.dot(shaft_axis, hole_axis)), -1., 1.))
        motor_ball = point(flange, centers[0])
        motor_seat = point(flange, flange_hole['seat_center_m'])
        motor_shaft = flange[:3, :3] @ source_flange_axis
        motor_hole = flange[:3, :3] @ flange_hole['axis']
        motor_shaft = motor_shaft if np.dot(motor_shaft, motor_hole) > 0 else -motor_shaft
        motor_line_error = np.linalg.norm(np.cross(motor_ball - motor_seat, motor_hole))
        motor_seat_error = abs(np.dot(motor_ball - motor_shaft * flange_height - motor_seat, motor_hole))
        motor_angle = np.arccos(np.clip(abs(np.dot(motor_shaft, motor_hole)), -1., 1.))
        closure = [np.linalg.norm(point(rod, centers[0]) - point(flange, centers[0])),
                   np.linalg.norm(point(rod, centers[1]) - point(platform, centers[1]))]
        distance = np.linalg.norm(point(rod, centers[1]) - point(rod, centers[0]))
        source_length = np.linalg.norm(centers[1] - centers[0])
        rod_direction = point(rod, centers[1]) - point(rod, centers[0])
        rod_direction /= np.linalg.norm(rod_direction)
        source_rod_axis = np.asarray(reference['endpoints']['flange']['rod_end_cylinder']['axis'])
        source_rod_axis = source_rod_axis if source_rod_axis[2] > 0 else -source_rod_axis
        actual_rod_axis = rod[:3, :3] @ source_rod_axis
        average_axis = motor_shaft + hole_axis
        expected_rod_axis = average_axis - np.dot(average_axis, rod_direction) * rod_direction
        expected_rod_axis /= np.linalg.norm(expected_rod_axis)
        roll_error = np.arccos(np.clip(np.dot(actual_rod_axis, expected_rod_axis), -1., 1.))
        motor_source = reference['motor_axis_candidates'][0]
        source_motor_axis = np.asarray(motor_source['axis'])
        source_motor_origin = np.asarray(motor_source['point_on_axis_m'])
        fixed_motor_axis = base_delta[:3, :3] @ source_motor_axis
        moving_motor_axis = flange[:3, :3] @ source_motor_axis
        axis_error = np.linalg.norm(np.cross(point(flange, source_motor_origin) - point(base_delta, source_motor_origin), fixed_motor_axis))
        axis_angle = np.arccos(np.clip(abs(np.dot(fixed_motor_axis, moving_motor_axis)), -1., 1.))
        valid = (line_error < .000025 and seat_error < .000025 and angle < np.deg2rad(.02)
                 and motor_line_error < .000025 and motor_seat_error < .000025 and motor_angle < np.deg2rad(.02)
                 and max(closure) < .000025 and abs(distance-source_length) < 1e-8
                 and roll_error < np.deg2rad(.02) and axis_error < .000025 and axis_angle < np.deg2rad(.02))
        results[side] = {'mount_axis_error_m': float(line_error), 'seat_gap_m': float(seat_error),
                         'mount_axis_angle_deg': float(np.rad2deg(angle)),
                         'flange_mount_axis_error_m': float(motor_line_error),
                         'flange_seat_gap_m': float(motor_seat_error),
                         'flange_mount_axis_angle_deg': float(np.rad2deg(motor_angle)),
                         'independent_receiver_seat_source_m': hole['seat_center_m'].tolist(),
                         'receiver_rim_depth_m': hole['plate_thickness_m'],
                         'rod_roll_error_deg': float(np.rad2deg(roll_error)),
                         'motor_axis_error_m': float(axis_error),
                         'motor_axis_angle_deg': float(np.rad2deg(axis_angle)),
                         'rod_endpoint_errors_m': [float(e) for e in closure],
                         'rod_length_m': float(distance), 'passed': bool(valid)}
    return {'model': str(path), 'all_passed': all(v['passed'] for v in results.values()), 'sides': results}


if __name__ == '__main__':
    solution = solve()
    before = validate(MODEL / 'mujoco/full_body_robot.step-aligned.xml', solution)
    path = build(solution)
    after = validate(path, solution)
    if not after['all_passed']:
        raise ValueError(f'Mounted hips validation failed: {after}')
    report = {'scope': 'Static fixture/ball/race/rod reconstruction only; motion and contact acceptance remain pending',
              'before': before, 'after': after, 'calibration': solution}
    (MODEL / 'cad/assembly_audit/2026-11-12-hips-mounting.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(path)
    print(json.dumps(after, indent=2))
