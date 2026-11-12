"""Fixture-hole, seating, mesh ownership and roll regressions for hips rods."""
import copy
import importlib.util
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'


@pytest.fixture(scope='module')
def hips():
    pytest.importorskip('mujoco')
    pytest.importorskip('vtk')
    sys.path.insert(0, str(MODEL / 'tools'))
    spec = importlib.util.spec_from_file_location('hips_mounting', MODEL / 'tools/hips_mounting.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def mounted(hips, tmp_path_factory):
    directory = tmp_path_factory.mktemp('hips-build')
    solution = hips.solve()
    path = hips.build(solution, output=directory / 'mounted.xml',
                      mesh_directory=directory / 'meshes', config_path=None)
    return solution, path


def variant(root, directory):
    root.find('compiler').set('meshdir', str(MODEL))
    path = directory / 'variant.xml'
    path.write_text(ET.tostring(root, encoding='unicode'))
    return path


def move_plus(root, solution, offset):
    for part in ('platform', 'rod', 'flange'):
        body = root.find(f".//body[@name='hips_plus_{part}_mounted']")
        pos = np.fromstring(body.get('pos'), sep=' ')
        parent = solution['receiver_frame_world'] if part == 'platform' else solution['base_frame_world']
        pos += np.asarray(parent)[:3, :3].T @ offset
        body.set('pos', ' '.join(map(str, pos)))


def test_hips_mounts_meet_hole_axis_and_receiver_face(hips, mounted):
    solution, path = mounted
    result = hips.validate(path, solution)
    assert result['all_passed'], result
    for side in result['sides'].values():
        assert side['rod_length_m'] == pytest.approx(.065, abs=1e-8)
        assert side['flange_mount_axis_error_m'] < .000025
        assert side['flange_seat_gap_m'] < .000025
    assert solution['sides']['plus']['regions']['rod'] == [17, 18, 2]
    assert solution['sides']['minus']['regions']['rod'] == [12, 7, 6]
    assert set(solution['sides']['plus']['regions']['platform']) == {8, 19}
    assert set(solution['sides']['minus']['regions']['platform']) == {1, 13}
    assert 16 in solution['sides']['plus']['regions']['flange']
    assert 9 in solution['sides']['minus']['regions']['flange']
    for entry in solution['sides'].values():
        for check in entry['outer_race_checks'].values():
            assert check['owner'] == 'rod'
            assert check['source_region'] in entry['regions']['rod']
            assert check['source_region'] not in entry['regions']['platform']
            assert check['measured_width_m'] == pytest.approx(.005, abs=.000025)


def test_raw_step_ball_match_is_not_fixture_installation(hips, mounted):
    result = hips.validate(MODEL / 'mujoco/full_body_robot.step-aligned.xml', mounted[0])
    assert not result['all_passed']
    assert all(s['mount_axis_error_m'] > .01 for s in result['sides'].values())


def test_moving_whole_linkage_still_fails_fixture_check(hips, mounted, tmp_path):
    solution, path = mounted
    root = ET.parse(path).getroot()
    move_plus(root, solution, [.001, 0, 0])
    result = hips.validate(variant(root, tmp_path), solution)
    assert not result['all_passed']
    assert max(result['sides']['plus']['rod_endpoint_errors_m']) < .000025
    assert result['sides']['plus']['mount_axis_error_m'] > .0005


def test_moving_solution_and_linkage_to_lower_shoulder_is_rejected(hips, mounted, tmp_path):
    solution, path = mounted
    root = ET.parse(path).getroot()
    solution = copy.deepcopy(solution)
    entry = solution['sides']['plus']
    offset = -.002 * np.asarray(entry['hole_axis'])
    move_plus(root, solution, offset)
    entry['seat_center_m'] = (np.asarray(entry['seat_center_m']) + offset).tolist()
    result = hips.validate(variant(root, tmp_path), solution)
    assert not result['all_passed']
    assert result['sides']['plus']['seat_gap_m'] > .0019


def test_swapped_rod_and_platform_mesh_bindings_are_rejected(hips, mounted, tmp_path):
    solution, path = mounted
    root = ET.parse(path).getroot()
    rod = root.find(".//body[@name='hips_plus_rod_mounted']/geom")
    platform = root.find(".//body[@name='hips_plus_platform_mounted']/geom")
    rod_mesh, platform_mesh = rod.get('mesh'), platform.get('mesh')
    rod.set('mesh', platform_mesh)
    platform.set('mesh', rod_mesh)
    with pytest.raises(ValueError, match='mesh binding'):
        hips.validate(variant(root, tmp_path), solution)


@pytest.mark.parametrize('degrees', [20, 180])
def test_rod_axial_roll_is_rejected_even_with_closed_ball_centers(hips, mounted, tmp_path, degrees):
    solution, path = mounted
    root = ET.parse(path).getroot()
    entry = solution['sides']['plus']
    start, end = (np.asarray(entry[k]) for k in ('new_flange_ball_m', 'target_ball_m'))
    roll = Rotation.from_rotvec((end - start) / np.linalg.norm(end - start) * np.deg2rad(degrees)).as_matrix()
    world = hips.rigid(roll, start, start) @ np.asarray(entry['rod_transform']) @ solution['base_frame_world']
    local = np.linalg.inv(solution['base_frame_world']) @ world
    body = root.find(".//body[@name='hips_plus_rod_mounted']")
    body.set('pos', ' '.join(map(str, local[:3, 3])))
    q = Rotation.from_matrix(local[:3, :3]).as_quat()
    body.set('quat', ' '.join(map(str, q[[3, 0, 1, 2]])))
    result = hips.validate(variant(root, tmp_path), solution)
    assert not result['all_passed']
    assert max(result['sides']['plus']['rod_endpoint_errors_m']) < .000025
    assert result['sides']['plus']['rod_roll_error_deg'] > degrees - 1


@pytest.mark.parametrize('part', ['rod', 'platform'])
def test_fixed_base_cannot_retain_moving_hips_components(hips, mounted, tmp_path, part):
    solution, path = mounted
    root = ET.parse(path).getroot()
    base_mesh = root.find(".//body[@name='Base_link']/geom").get('mesh')
    base_asset = root.find(f"asset/mesh[@name='{base_mesh}']")
    extra_asset = root.find(f"asset/mesh[@name='hips_plus_{part}_mounted']")
    append = hips.vtk.vtkAppendPolyData()
    for asset in (base_asset, extra_asset):
        reader = hips.vtk.vtkSTLReader()
        reader.SetFileName(asset.get('file'))
        reader.Update()
        append.AddInputData(reader.GetOutput())
    append.Update()
    mesh = tmp_path / 'base_with_duplicate_hips.STL'
    writer = hips.vtk.vtkSTLWriter()
    writer.SetInputData(append.GetOutput())
    writer.SetFileTypeToBinary()
    writer.SetFileName(str(mesh))
    assert writer.Write()
    base_asset.set('file', str(mesh))
    with pytest.raises(ValueError, match='Base_link: mesh content'):
        hips.validate(variant(root, tmp_path), solution)
