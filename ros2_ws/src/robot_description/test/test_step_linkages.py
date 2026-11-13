"""STEP rod-end concentricity and rejection of a displaced rod."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'


@pytest.fixture(scope='module')
def audit_module():
    pytest.importorskip('mujoco')
    pytest.importorskip('vtk')
    spec = importlib.util.spec_from_file_location('step_linkage_audit', MODEL / 'tools/audit_step_linkages.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_twenty_step_rod_ends_share_ball_centers(audit_module):
    report = audit_module.audit()
    assert len(report['rods']) == 10
    for rod in report['rods'].values():
        assert len(rod['endpoints']) == 2
        for endpoint in rod['endpoints'].values():
            hole = endpoint['rod_end_cylinder']
            assert hole['line_error_m'] < .000025
            assert hole['axial_center_error_m'] < .000025
            assert hole['max_angular_gap_deg'] <= 45
            assert hole['rod_support'] >= 8
        assert rod['motor_axis_candidates']
    assert report['rods']['left_shoulder_flange_linkage_rod']['center_distance_m'] == pytest.approx(.100684850, abs=1e-8)
    assert report['rods']['right_shoulder_flange_linkage_rod']['center_distance_m'] == pytest.approx(.100684850, abs=1e-8)
    plus = report['rods']['waist_plus_rod']['endpoints']['flange']['center_m']
    minus = report['rods']['waist_minus_rod']['endpoints']['flange']['center_m']
    assert np.linalg.norm(np.asarray(plus) - minus) == pytest.approx(.102, abs=1e-8)


def test_displaced_rod_does_not_pass_a_matching_ball_check(audit_module):
    features = json.loads((MODEL / 'cad/assembly_audit/step-analytic-features.json').read_text())
    parts = audit_module.load_parts()
    rod = parts['waist_plus_rod']['points']
    mate = parts['waist_plus_flange']['points']
    endpoint = audit_module.check_endpoint(rod, mate, features)
    axis = np.asarray(endpoint['rod_end_cylinder']['axis'])
    # Axial displacement leaves the infinite cylinder axis unchanged. The
    # section-center check must still reject the visibly separated rod.
    with pytest.raises(ValueError, match='common ball/rod-end center'):
        audit_module.check_endpoint(rod + .003 * axis, mate, features)
