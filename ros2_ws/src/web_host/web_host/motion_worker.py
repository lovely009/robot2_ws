"""Subprocess worker for BVH retargeting so conversion avoids GIL contention."""

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def run_job(directory: Path) -> None:
    import numpy as np
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    from bvh_broadcaster.bvh import BvhClip
    from bvh_broadcaster.retarget import ArmRetargeter, digest, load_motion
    from .shoulder_retarget import ShoulderRetargeter

    params = json.loads((directory / 'worker_input.json').read_text(encoding='utf-8'))
    generic = bool(params.get('generic'))
    mapping = params['mapping']
    up_axis = params['up_axis']
    scale = params['scale']
    links = params['robot_links']
    simulation = params.get('simulation')
    if simulation and isinstance(simulation.get('directory'), str):
        simulation = {**simulation, 'directory': Path(simulation['directory'])}

    clip = BvhClip.load(directory / 'source.bvh', preserve_names=generic)
    from .motion_source import prepare_source
    prepare_source(clip, up_axis)
    if generic:
        from bvh_broadcaster.generic_retarget import GenericRetargeter
        solver = GenericRetargeter(
            directory / 'robot.urdf',
            mapping,
            clip,
            up_axis=up_axis,
            scale=scale,
            match_directions=bool(simulation),
        )
        if simulation:
            from .motion_simulation import position_filter
            solver.position_filter = position_filter(simulation, solver.robot.independent)
            solver.mechanical_limits_enforced = False
            solver.report.update(
                mechanical_limits_enforced=False,
                simulation_limits_enforced=True,
                velocity_limit_rad_s=simulation['velocity_limit_rad_s'],
            )
    else:
        retargeter = ShoulderRetargeter if mapping.get('mode') == 'shoulders12' else ArmRetargeter
        solver = retargeter(directory / 'robot.urdf', mapping, clip, up_axis=up_axis)

    last_pct = -1

    def progress(frame, total):
        nonlocal last_pct
        pct = round(frame / total * 90)
        if pct != last_pct:
            last_pct = pct
            print(f'PROGRESS {pct}', flush=True)

    positions, directions, orientations = solver.convert(progress)
    if simulation:
        from .motion_simulation import validate_positions
        validate_positions(simulation, solver.names, positions)

    np.savez_compressed(
        directory / 'motion.npz',
        positions=positions,
        joint_names=solver.names,
        frame_time=clip.frame_time,
        preview_only=True,
        simulation_limits_enforced=bool(simulation),
        mechanical_limits_enforced=getattr(solver, 'mechanical_limits_enforced', True),
        urdf_sha256=digest(directory / 'robot.urdf'),
        direction_error_deg=directions,
        orientation_error_deg=orientations,
    )
    if generic:
        solver.validate_result(positions)
    else:
        load_motion(directory / 'motion.npz', directory / 'robot.urdf')

    link_ids = [solver.model.getFrameId(n, pin.FrameType.BODY) for n in links]
    children = {
        j.find('child').attrib['link']: j.find('parent').attrib['link']
        for j in ET.parse(directory / 'robot.urdf').getroot().findall('joint')
    }
    robot_parents = [links.index(children[n]) if n in children else -1 for n in links]
    indices = sorted(
        set(range(0, len(positions), max(1, (len(positions) + 899) // 900)))
        | {len(positions) - 1}
    )
    source_frames, robot_frames, root_quaternions = [], [], []
    for index in indices:
        ps, rs = [], []
        for joint, (p, quat) in zip(
            clip.joints,
            clip.local_transforms(index, scale=scale, up_axis=up_axis, apply_root_offset=False),
        ):
            r = Rotation.from_quat(quat).as_matrix()
            if joint.parent >= 0:
                p, r = ps[joint.parent] + rs[joint.parent] @ p, rs[joint.parent] @ r
            ps.append(p)
            rs.append(r)
        source_frames.append(np.round(ps, 5).tolist())
        if generic:
            root_rotation = solver.basis @ rs[solver.source_root] @ solver.basis.T
            root_quaternions.append(Rotation.from_matrix(root_rotation).as_quat().tolist())
        q = solver.configuration(positions[index]) if generic else positions[index]
        pin.framesForwardKinematics(solver.model, solver.data, q)
        robot_frames.append(np.round([solver.data.oMf[f].translation for f in link_ids], 5).tolist())

    preview = {
        'times': (np.array(indices) * clip.frame_time).tolist(),
        'duration': (len(positions) - 1) * clip.frame_time,
        'source': {
            'names': [j.name for j in clip.joints],
            'parents': [j.parent for j in clip.joints],
            'frames': source_frames,
        },
        'robot': {
            'names': links,
            'parents': robot_parents,
            'frames': robot_frames,
            'urdf': (directory / 'robot.urdf').read_bytes().decode('utf-8'),
            'ignore_limits': generic and not solver.mechanical_limits_enforced,
            'joint_names': solver.names,
            'joint_positions': np.round(positions[indices], 8).tolist(),
        },
        'source_to_robot': mapping['source_to_robot'],
    }
    if generic:
        preview['robot'].update(
            root_quaternions=root_quaternions,
            root_rotation_source=clip.joints[solver.source_root].name,
            root_rotation_frame='robot',
        )
    if simulation:
        from .motion_simulation import cad_preview
        cad = cad_preview(simulation, solver.names, positions[indices])
        if cad:
            preview['robot'].update(cad)
    (directory / 'preview.json').write_text(json.dumps(preview, allow_nan=False), encoding='utf-8')

    summary = {
        'preview_only': True,
        'frames': len(positions),
        'joint_names': solver.names,
        'frame_time': clip.frame_time,
        'mapping': mapping,
        'direction_mean_deg': np.mean(directions, axis=0).tolist(),
        'hand_orientation_mean_deg': np.mean(orientations, axis=0).tolist(),
        'nonconverged_solves': solver.nonconverged,
    }
    if generic:
        summary.update(solver.report)
    (directory / 'report.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8'
    )


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    directory = Path(args[0])
    try:
        run_job(directory)
        return 0
    except Exception as exc:
        (directory / 'worker_error.txt').write_text(str(exc), encoding='utf-8')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
