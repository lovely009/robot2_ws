"""Small geometry-scanner regressions, using the system Coal/NumPy ABI."""

import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

if int(np.__version__.split(".")[0]) >= 2:
    pytest.skip("Run the Coal scanner tests with system Python/NumPy", allow_module_level=True)
pytest.importorskip("coal")

MODEL = Path(__file__).parents[1] / "models/full_body_robot"
spec = importlib.util.spec_from_file_location(
    "full_body_limit_scanner", MODEL / "tools/derive_simulation_limits.py")
scanner = importlib.util.module_from_spec(spec)
sys.path.insert(0, str(MODEL / 'tools'))
try:
    spec.loader.exec_module(scanner)
finally:
    sys.path.remove(str(MODEL / 'tools'))


def test_coupled_region_keeps_only_the_neutral_connected_component():
    first, second = np.arange(-2, 3), np.arange(-3, 4)
    mask = np.zeros((5, 7), dtype=bool)
    mask[1:4, 2:5] = True
    mask[0, 0] = True
    region = scanner.neutral_component(mask, first, second)
    assert region.sum() == 9
    assert region[2, 3] and not region[0, 0]
    mask[2, 3] = False
    with pytest.raises(ValueError, match='neutral'):
        scanner.neutral_component(mask, first, second)


def test_mating_exclusion_does_not_hide_other_components_of_the_same_link():
    vertices = "0 0 0 .1 0 0 0 .1 0 0 0 .1 1 0 0 1.1 0 0 1 .1 0 1 0 .1"
    faces = "0 2 1 0 1 3 0 3 2 1 2 3 4 6 5 4 5 7 4 7 6 5 6 7"
    model = scanner.mujoco.MjModel.from_xml_string(f"""<mujoco>
      <asset><mesh name="parts" vertex="{vertices}" face="{faces}"/></asset>
      <worldbody>
        <geom name="fixed" type="mesh" mesh="parts"/>
        <body name="moving"><joint type="slide" axis="1 0 0"/>
          <geom name="rotating" type="mesh" mesh="parts"/></body>
      </worldbody></mujoco>""")
    data = scanner.mujoco.MjData(model)
    scanner.mujoco.mj_kinematics(model, data)
    collisions = scanner.SurfaceCollisions(model, data)
    assert not collisions.exclusions
    assert len(collisions.unclassified_collisions) == 2
    assert collisions.check(data) is not None
    data.qpos[:] = 1
    scanner.mujoco.mj_kinematics(model, data)
    assert collisions.check(data) is not None


def test_adjacent_body_and_external_obstacles_both_remain_checked():
    model = scanner.mujoco.MjModel.from_xml_string('''<mujoco>
      <asset><mesh name="part" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"
        face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset>
      <worldbody>
        <geom name="housing" type="mesh" mesh="part" pos=".25 0 0"/>
        <body name="moving"><joint name="assembly_axis" type="slide" axis="1 0 0"/>
          <geom name="rotor" type="mesh" mesh="part"/></body>
        <body name="external" pos="1 0 0"><geom name="obstacle" type="mesh" mesh="part"/></body>
      </worldbody></mujoco>''')
    data = scanner.mujoco.MjData(model)
    scanner.mujoco.mj_kinematics(model, data)
    strict = scanner.SurfaceCollisions(model, data)
    data.qpos[:] = .25
    scanner.mujoco.mj_kinematics(model, data)
    assert strict.check(data) is not None
    data.qpos[:] = 1
    scanner.mujoco.mj_kinematics(model, data)
    assert 'obstacle' in strict.check(data)


def test_scanned_limits_are_bound_to_current_exact_cad():
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    config = yaml.safe_load((MODEL / 'config/simulation_limits.yaml').read_text())
    assert config['collision_validation'].startswith('sampled:')
    assert report['geometry_fingerprint'] == config['geometry_sha256'] == scanner.source_fingerprint()
    assert not report['adjacent_assembly_exclusions']
    assert not report['unclassified_collisions']
    assert report['accepted_exemptions']
    assert len(report['collision_mesh_sha256']) == 68
    for name, digest in report['collision_mesh_sha256'].items():
        assert hashlib.sha256((MODEL / 'mujoco/collision/native' / name).read_bytes()).hexdigest() == digest
    assert config['source_sha256'] == hashlib.sha256((MODEL / 'urdf/full_body_robot.urdf').read_bytes()).hexdigest()


def test_new_scanned_asymmetric_bounds_are_used_without_legacy_symmetrizing():
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    config = yaml.safe_load((MODEL / 'config/simulation_limits.yaml').read_text())
    assert len(report['joint_limits']) == len(config['joint_limits']) == 37
    assert len(report['actuator_limits']) == len(config['actuator_limits']) == 37
    for name, bounds in report['joint_limits'].items():
        assert [config['joint_limits'][name][key] for key in ('lower', 'upper')] == bounds
    for name, bounds in report['actuator_limits'].items():
        assert [config['actuator_limits'][name][key] for key in ('lower', 'upper')] == bounds
    assert any(abs(a+b) > 1e-4 for a,b in report['joint_limits'].values())


def test_native_coupled_domains_match_the_display_model():
    from robot_transmission.coupled_limits import CoupledLimits
    config = yaml.safe_load((MODEL / 'config/simulation_limits.yaml').read_text())
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    root = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    embedded = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    assert embedded['coupled_limits'] == config['coupled_limits']
    assert embedded['joint_limits'] == {name: [bound['lower'], bound['upper']]
                                       for name, bound in config['joint_limits'].items()}
    assert set(config['coupled_limits']) == set(topology['differential_groups'])
    for name, workspace in config['coupled_limits'].items():
        assert workspace['joints'] == topology['differential_groups'][name]['joints']
        assert len({len(row) for row in workspace['valid_rows']}) == 1
    CoupledLimits(config['coupled_limits'], config['joint_limits']).validate(
        dict.fromkeys(config['joint_limits'], 0.))


def test_mate_requires_shared_step_surface_and_finite_extent():
    angles = np.linspace(0., 2*np.pi, 40, endpoint=False)
    ring = np.column_stack((.01*np.cos(angles), .01*np.sin(angles), np.zeros(40)))
    points = np.vstack((ring+[0, 0, -.002], ring+[0, 0, .002]))
    feature = (np.zeros(3), np.array([0., 0., 1.]), .01, 'independent STEP fixture')
    volumes = scanner.cad_mating.shared_surfaces(points, points, np.array([[.01, 0, 0]]), [feature], [])
    assert volumes
    assert any(scanner.cad_mating.inside(np.array([[.01, 0, 0]]), v).all() for v in volumes)
    for volume in volumes:
        assert not scanner.cad_mating.inside(np.array([[.01, 0, .1], [.02, 0, 0]]), volume).any()
    assert not scanner.cad_mating.shared_surfaces(points, points*2, np.array([[.01, 0, 0]]), [feature], [])


def test_motor_pocket_accepts_a_step_verified_d_shaped_housing():
    model=scanner.mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <geom name="stator" type="box" size=".02 .02 .01"/>
      <body><joint name="motor" axis="0 0 1"/><geom name="rotor" type="box" size=".01 .01 .01"/></body>
      </worldbody><actuator><position joint="motor"/></actuator></mujoco>""")
    data=scanner.mujoco.MjData(model);scanner.mujoco.mj_forward(model,data)
    angles=np.linspace(np.deg2rad(35),np.deg2rad(325),50)
    ring=np.column_stack((.027*np.cos(angles),.027*np.sin(angles),np.zeros(len(angles))))
    points=np.vstack((ring+[0,0,-.01],ring+[0,0,.01]))
    faces=np.array([face for i in range(len(angles)-1) for face in [(i,i+1,i+len(angles)),(i+1,i+len(angles)+1,i+len(angles))]])
    parts={0:[{'tag':'housing','digest':'independent fixture','triangles':faces}],1:[{'tag':'rotor','digest':'rotor fixture','triangles':faces}]}
    world={0:[points],1:[points*.8]}
    features=[(np.zeros(3),np.array([0.,0.,1.]),.027,'independent STEP cylinder')]
    result=scanner.cad_mating.motor_interiors(model,data,parts,world,features)
    assert len(result)==1
    volume=next(iter(result.values()))[0]
    assert volume['maximum_angular_gap_deg']==pytest.approx(70.)
    assert scanner.cad_mating.inside(np.array([[.02,0,0]]),volume).all()
    assert not scanner.cad_mating.inside(np.array([[.04,0,0],[0,0,.05]]),volume).any()
    offaxis=[(np.array([.001,0,0]),np.array([0.,0.,1.]),.027,'wrong axis')]
    assert not scanner.cad_mating.motor_interiors(model,data,parts,world,offaxis)
    # A hinge origin can slide along its axis without changing any physical pocket.
    data.xanchor[model.joint('motor').id] += [0.,0.,.5]
    shifted=scanner.cad_mating.motor_interiors(model,data,parts,world,features)
    shifted_volume=next(iter(shifted.values()))[0]
    np.testing.assert_allclose(shifted_volume['origin'],volume['origin'],atol=1e-12)
    assert shifted_volume['half_length']==pytest.approx(volume['half_length'])


def test_contact_allowance_checks_entire_component_and_current_world_frame():
    # Two colliding lobes belong to ONE component; only the first is an allowed mate.
    vertices='0 0 0 .1 0 0 0 .1 0 0 0 .1 1 0 0 1.1 0 0 1 .1 0 1 0 .1'
    faces='0 2 1 0 1 3 0 3 2 1 2 3 4 6 5 4 5 7 4 7 6 5 6 7 0 4 1'
    model=scanner.mujoco.MjModel.from_xml_string(f"""<mujoco><asset><mesh name="part" vertex="{vertices}" face="{faces}"/></asset>
      <worldbody><geom name="a" type="mesh" mesh="part"/>
      <body><joint type="slide" axis="0 0 1"/><geom name="b" type="mesh" mesh="part"/></body></worldbody></mujoco>""")
    data=scanner.mujoco.MjData(model);scanner.mujoco.mj_forward(model,data)
    collisions=scanner.SurfaceCollisions(model,data)
    key=(0,0,1,0)
    rotation=data.geom_xmat[0].reshape(3,3);position=data.geom_xpos[0]
    volume={'kind':'sphere','origin':np.zeros(3),'radius':.2,'a_origin':rotation.T@(-position)}
    collisions.mates[key]={'volumes':[volume]}
    transforms={i:scanner.coal.Transform3s(data.geom_xmat[i].reshape(3,3),data.geom_xpos[i]) for i in (0,1)}
    assert collisions.pair_hit(key,transforms,(0,))  # The second lobe remains blocked.
    volume['radius']=2.
    assert not collisions.pair_hit(key,transforms,(0,))
    # Moving both parts together must move the mate volume too.
    shifted={i:scanner.coal.Transform3s(data.geom_xmat[i].reshape(3,3),data.geom_xpos[i]+[3,2,1]) for i in (0,1)}
    assert not collisions.pair_hit(key,shifted,(0,))
    from types import SimpleNamespace
    collisions.collide=lambda *args: True
    collisions.result=SimpleNamespace(getContacts=lambda:[])
    assert collisions.pair_hit(key,transforms,(0,))
    budgets=[]
    def saturated(*args):
        budgets.append(collisions.request.num_max_contacts)
        return True
    collisions.collide=saturated
    witness=SimpleNamespace(pos=np.zeros(3));external=SimpleNamespace(pos=np.array([3.,0.,0.]))
    collisions.result=SimpleNamespace(getContacts=lambda:[witness]*32 if collisions.request.num_max_contacts==32 else [external])
    assert collisions.pair_hit(key,transforms,(0,))
    assert budgets==[32,128]  # A full allowed prefix cannot hide the next contact.
    budgets.clear()
    collisions.result=SimpleNamespace(getContacts=lambda:[witness]*collisions.request.num_max_contacts)
    assert collisions.pair_hit(key,transforms,(0,))
    assert budgets[-1]>=1048576 and budgets[-2]<1048576  # Saturation is blocked rather than silently accepted.


def test_cylinder_material_extrema_include_a_coarse_face_with_all_vertices_outside():
    points=np.array([[-2.,-2.,1.],[2.,-2.,3.],[0.,3.,2.]])
    bounds=scanner.cad_mating.cylinder_material_bounds(points,np.array([[0,1,2]]),np.zeros(3),np.array([0.,0.,1.]),.5)
    np.testing.assert_allclose(bounds,[1.75,2.25],atol=1e-12)
    vertical=np.array([[2.,0.,-1.],[-2.,0.,-1.],[2.,0.,1.]])
    bounds=scanner.cad_mating.cylinder_material_bounds(vertical,np.array([[0,1,2]]),np.zeros(3),np.array([0.,0.,1.]),.5)
    np.testing.assert_allclose(bounds,[-1.,.25],atol=1e-12)


def test_disconnected_axial_material_is_not_filled_by_a_projection_hull():
    first=np.array([[0.,0.,0.],[.1,0.,1.],[0.,.1,1.]])
    points=np.vstack((first,first+[0.,0.,3.]))
    intervals=scanner.cad_mating.cylinder_material_intervals(points,np.array([[0,1,2],[3,4,5]]),np.zeros(3),np.array([0.,0.,1.]),1.)
    np.testing.assert_allclose(intervals,[[0.,1.],[3.,4.]])


def test_planar_allowance_uses_both_actual_triangle_patches():
    triangle=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]])
    volume={'kind':'plane','origin':np.zeros(3),'axis':np.array([0.,0.,1.]),'u':np.array([1.,0.,0.]),
            'v':np.array([0.,1.,0.]),'half_u':2.,'half_v':2.,'thickness':.0001,
            'patch_a':triangle,'patch_b':triangle+[.5,0.,0.]}
    result=scanner.cad_mating.inside(np.array([[.6,.1,0.],[.1,.1,0.],[1.3,.1,0.],[.6,.1,.002]]),volume)
    assert result.tolist()==[True,False,False,False]
    disconnected=np.array([[[0.,0.],[1.,0.],[0.,1.]],[[2.,2.],[3.,2.],[2.,3.]]])
    assert scanner.cad_mating.patch_contains(np.array([[.1,.1],[2.1,2.1],[1.5,1.5]]),disconnected,1e-5).tolist()==[True,True,False]


def test_scan_generation_ranges_masks_and_policy_are_bound(tmp_path):
    import copy
    artifacts=scanner.scan_artifacts
    for filename in artifacts.POLICY_PATHS:
        path=tmp_path/filename;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'policy')
    (tmp_path/'mujoco').mkdir()
    hashes=artifacts.policy_hashes(tmp_path)
    report={'policy_inputs_sha256':hashes,'scan_script_sha256':hashes['tools/derive_simulation_limits.py'],
            'mating_script_sha256':hashes['tools/cad_mating.py'],'joint_limits':{'j':[-.2,.3]},
            'actuator_limits':{'m':[-.1,.4]},'differential_groups':{'g':{'coupled_workspace':{'valid_rows':['1']}}}}
    raw=json.dumps(report).encode();(tmp_path/'mujoco/geometry_limits.json').write_bytes(raw)
    config={'scan_report_sha256':hashlib.sha256(raw).hexdigest(),'joint_limits':{'j':{'lower':-.2,'upper':.3}},
            'actuator_limits':{'m':{'lower':-.1,'upper':.4}},'coupled_limits':{'g':{'valid_rows':['1']}}}
    assert artifacts.validate(tmp_path,config)==report
    for key in ('joint_limits','actuator_limits','coupled_limits'):
        broken=copy.deepcopy(config);broken[key]={}
        with pytest.raises(ValueError,match='mismatch'):artifacts.validate(tmp_path,broken)
    broken=copy.deepcopy(config);broken['scan_report_sha256']='older-generation'
    with pytest.raises(ValueError,match='generation mismatch'):artifacts.validate(tmp_path,broken)
    (tmp_path/'tools/cad_mating.py').write_bytes(b'new policy')
    with pytest.raises(ValueError,match='policy'):artifacts.validate(tmp_path,config)


def test_verified_finite_interior_also_applies_to_contacts_absent_at_neutral():
    model=scanner.mujoco.MjModel.from_xml_string("""<mujoco><asset>
      <mesh name="cube" vertex="-.05 -.05 -.05 .05 -.05 -.05 .05 .05 -.05 -.05 .05 -.05 -.05 -.05 .05 .05 -.05 .05 .05 .05 .05 -.05 .05 .05"
        face="0 2 1 0 3 2 4 5 6 4 6 7 0 1 5 0 5 4 1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7"/></asset>
      <worldbody><geom name="a" type="mesh" mesh="cube" pos="1 0 0"/>
      <body><joint type="hinge" axis="0 0 1"/><geom name="b" type="mesh" mesh="cube" pos="0 1 0"/></body></worldbody></mujoco>""")
    data=scanner.mujoco.MjData(model);scanner.mujoco.mj_forward(model,data)
    collisions=scanner.SurfaceCollisions(model,data)
    assert not collisions.candidate_neutral_collisions
    pair=frozenset(int(model.geom_bodyid[g]) for g in (0,1))
    collisions.motor_interiors[pair]=[{'kind':'sphere','origin':np.zeros(3),'radius':1.2,'evidence':'fixture certified finite interior'}]
    data.qpos[:]=-np.pi/2;scanner.mujoco.mj_forward(model,data)
    assert collisions.check(data) is None
    assert collisions.mates
    for entry in collisions.mates.values():
        for volume in entry['volumes']:volume['radius']=.9
    assert collisions.check(data) is not None  # Contacts outside the same finite zone stay blocked.


def test_axial_interface_fit_accepts_only_the_documented_tolerance():
    intersections=scanner.cad_mating.interval_intersections
    tiny=list(intersections([[0.,.01]],[[.010000003,.02]]))
    assert len(tiny)==1 and tiny[0][0]==tiny[0][1]
    assert tiny[0][2]==pytest.approx(3e-9)
    assert not list(intersections([[0.,.01]],[[.0101,.02]]))
