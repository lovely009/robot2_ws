"""全身 CAD 预览件的命名、参数保真和资源检查。"""

import hashlib
from itertools import product
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE = Path(__file__).parents[1]
MODEL = PACKAGE / "models" / "full_body_robot"
SOURCE = MODEL / "cad"
ORIGINAL_NAME = "全身装配（不含头部）"
ORIGINAL_URDF = SOURCE / "urdf" / f"{ORIGINAL_NAME}.urdf"
RUNTIME_URDF = MODEL / "urdf" / f"{MODEL.name}.urdf"
MAPPING_URDF = MODEL / "urdf" / f"{MODEL.name}.mapping.urdf"

JOINT_NAMES = {
    "Joint_Link1": "BN_Spine_yaw",
    "Joint_Link3": "BN_Spine_pitch",
    "Joint_Link2": "BN_Spine_roll",
    "Joint_Link4": "BN_hips_roll",
    "Joint_Link5": "BN_hips_pitch",
}
for side, suffix in (("Left", ""), ("Right", "r")):
    groups = (
        ("Shoulder", "3", 1, ("axis_1", "axis_2")),
        ("UpperArm", "3", 3, ("axis_1", "axis_2", "axis_3")),
        ("LowerArm", "3", 6, ("axis_1", "axis_2")),
        ("Hand", "3", 8, ("axis_2", "axis_1") if side == "Left" else ("axis_1", "axis_2")),
        ("UpperLeg", "5", 1, ("axis_1", "axis_2", "axis_3")),
        ("LowerLeg", "5", 4, ("axis_1", "axis_2")),
        ("Foot", "5", 6, ("pitch", "roll")),
    )
    for node, branch, start, axes in groups:
        for index, axis in enumerate(axes, start):
            JOINT_NAMES[f"Joint_link{branch}.{index}{suffix}"] = f"BN_{side}{node}_{axis}"

NAME_ALIASES = __import__('json').loads((MODEL / 'config/joint_names.json').read_text())['joints']
JOINT_NAMES = {old: NAME_ALIASES.get(name, name) for old, name in JOINT_NAMES.items()}
LINK_NAMES = {"base_link": "Base_link"}
LINK_NAMES.update({original: f"{name}_link" for original, name in JOINT_NAMES.items()})


def _xml_shape(element):
    return (element.tag, element.attrib, (element.text or "").strip(),
            [_xml_shape(child) for child in element])


def test_original_urdf_is_unchanged():
    assert hashlib.sha256(ORIGINAL_URDF.read_bytes()).hexdigest() == (
        "76d4278667151ffd7f26b4e21ad75b3032ada97c346c76b808d1b83139c11920")


def test_native_visuals_cover_sources_and_unique_component_ownership():
    import json
    metadata = json.loads((MODEL / 'config/native_assembly.json').read_text())
    original = ET.parse(RUNTIME_URDF).getroot()
    assert set(metadata['ownership']) == {link.get('name') for link in original.findall('link')}
    mjcf = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    visible = mjcf.findall('.//geom[@type="mesh"]')
    assert len(visible) == 68
    for geom in visible:
        assert geom.get('mass') == '0'
        assert geom.get('contype') == geom.get('conaffinity') == '0'
    for name, entry in metadata['ownership'].items():
        source = MODEL.parent / 'robot_body/meshes' / entry['source']
        assert hashlib.sha256(source.read_bytes()).hexdigest() == entry['source_sha256']
        assigned = entry['fixed_regions'] + [i for part in entry['parts'].values() for i in part]
        assigned += [i for rotor in entry['rotor_regions'].values() for i in rotor]
        assert sorted(assigned) == list(range(len(assigned))), name
    display = ET.parse(MODEL / 'urdf/full_body_robot.standalone.urdf').getroot()
    assert len(display.findall('.//visual')) == 68
    assert all('/visual/native/' in mesh.get('filename') for mesh in display.findall('.//mesh'))

def test_display_and_collision_models_have_identical_physics():
    import numpy as np
    mujoco = pytest.importorskip('mujoco')
    display = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/full_body_robot.xml'))
    physics = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/full_body_robot.physics.xml'))
    for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'jnt_pos', 'jnt_axis',
                  'jnt_range', 'dof_damping', 'actuator_gainprm', 'actuator_biasprm',
                  'actuator_ctrlrange', 'eq_data', 'qpos0'):
        np.testing.assert_array_equal(getattr(display, field), getattr(physics, field), err_msg=field)


def test_runtime_inertias_preserve_native_export_world_com_and_tensor():
    import json
    import numpy as np
    from scipy.spatial.transform import Rotation
    source = ET.parse(MODEL.parent / 'robot_body/urdf/cnm.urdf').getroot()
    original = {link.get('name').strip(): link for link in source.findall('link')}
    native = json.loads((MODEL / 'config/step_alignment.json').read_text())['links']
    runtime = ET.parse(RUNTIME_URDF).getroot()
    frames = {'Base_link': np.eye(4)}
    for joint in runtime.findall('joint'):
        origin = joint.find('origin');local = np.eye(4)
        local[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
        local[:3,3] = np.fromstring(origin.get('xyz'),sep=' ')
        frames[joint.find('child').get('link')] = frames[joint.find('parent').get('link')] @ local
    def inertia(link, frame):
        item=link.find('inertial');origin=item.find('origin')
        center=frame[:3,:3]@np.fromstring(origin.get('xyz'),sep=' ')+frame[:3,3]
        rotation=frame[:3,:3]@Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix()
        values=item.find('inertia');xx,yy,zz,xy,xz,yz=[float(values.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
        tensor=rotation@np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])@rotation.T
        return center,tensor,float(item.find('mass').get('value'))
    for link in runtime.findall('link'):
        name=link.get('name');actual=inertia(link,frames[name])
        expected_frame=np.asarray(native[name]['world_transform'])
        mounting=json.loads((MODEL/'config/arm_mount_alignment.json').read_text())
        if name in mounting['affected_links']:
            expected_frame=np.asarray(mounting['world_transform'])@expected_frame
        expected=inertia(original[name],expected_frame)
        np.testing.assert_allclose(actual[0],expected[0],atol=1e-10)
        np.testing.assert_allclose(actual[1],expected[1],atol=1e-10)
        assert actual[2]==expected[2]
    assert _xml_shape(runtime)==_xml_shape(ET.parse(MAPPING_URDF).getroot())

def test_mapping_urdf_uses_same_joint_names_by_child_link():
    expected = {f"{name}_link": name for name in JOINT_NAMES.values()}
    actual = {joint.find("child").get("link"): joint.get("name")
              for joint in ET.parse(MAPPING_URDF).getroot().findall("joint")}
    assert actual == expected


@pytest.mark.parametrize(
    "model_name, joint_prefix, expected_rod_lengths, same_direction, opposite_direction",
    [
        ("waist", "BN_Spine", [0.042, 0.042], [-6.14, 0], [0.03, -4.99]),
        ("hips", "BN_Hips", [0.065, 0.065], [-3.85, -0.33], [0, 1.86]),
        ("left_shoulder", "BN_LeftShoulder", [0.106906], [5, 2.20], [5, -3.09]),
        ("right_shoulder", "BN_RightShoulder", [0.106906], [5, -2.20], [5, 3.09]),
        ("left_wrist", "BN_LeftHand", [0.101500, 0.066006], [-4.77, -0.01], [-0.11, -2.42]),
        ("right_wrist", "BN_RightHand", [0.101500, 0.066006], [0, 4.78], [2.41, 0.11]),
    ],
)
def test_mujoco_closes_rods_and_resolves_differential_modes(
    model_name, joint_prefix, expected_rod_lengths, same_direction, opposite_direction,
):
    mujoco = pytest.importorskip("mujoco")
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(MODEL / "mujoco" / f"{model_name}_differential.xml"))
    data = mujoco.MjData(model)
    assert model.nu == 2 and model.neq == len(expected_rod_lengths)
    assert model.nv == model.nu + model.neq
    input_addresses = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    mujoco.mj_forward(model, data)
    rod_lengths = data.ten_length.copy()
    np.testing.assert_allclose(rod_lengths, expected_rod_lengths, atol=1e-6)
    for index in range(model.nkey):
        mujoco.mj_resetDataKeyframe(model, data, index)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qpos[input_addresses], data.ctrl, atol=1e-6)
        np.testing.assert_allclose(data.ten_length, rod_lengths, atol=1e-6)
    for plus, minus in product(range(-10, 11, 5), repeat=2):
        mujoco.mj_resetData(model, data)
        target = np.deg2rad([plus, minus])
        for fraction in np.linspace(0, 1, 251):
            data.ctrl[:] = target * fraction
            mujoco.mj_step(model, data)
            np.testing.assert_allclose(data.ten_length, rod_lengths, atol=2e-5)
        mujoco.mj_step(model, data, nstep=500)
        np.testing.assert_allclose(data.qpos[input_addresses], target, atol=1e-5)
        np.testing.assert_allclose(data.ten_length, rod_lengths, atol=1e-6)
        assert not any(warning.number for warning in data.warning)
        angles = np.rad2deg([data.joint(__import__('yaml').safe_load((MODEL/'config/motor_topology.yaml').read_text())['differential_groups'][model_name]['joints'][0]).qpos[0],
                            data.joint(__import__('yaml').safe_load((MODEL/'config/motor_topology.yaml').read_text())['differential_groups'][model_name]['joints'][1]).qpos[0]])
        if (plus, minus) == (5, 5):
            np.testing.assert_allclose(angles, same_direction, atol=0.05)
        if (plus, minus) == (5, -5):
            np.testing.assert_allclose(angles, opposite_direction, atol=0.05)


def test_full_body_mujoco_combines_direct_drives_and_closed_loops():
    mujoco = pytest.importorskip("mujoco")
    import numpy as np
    from scipy.spatial.transform import Rotation
    import yaml

    model = mujoco.MjModel.from_xml_path(str(MODEL / "mujoco/full_body_robot.xml"))
    assert (model.nmesh, model.nu, model.nv, model.neq) == (68, 37, 67, 10)
    standalone = ET.parse(MODEL / "urdf/full_body_robot.standalone.urdf").getroot()
    assert {joint.get("name") for joint in standalone.findall("joint")} == {
        model.joint(index).name for index in range(model.njnt)
    }
    controlled = ET.parse(MODEL / "urdf/full_body_robot.urdf").getroot().findall("joint")
    assert [joint.get("name") for joint in standalone.findall("joint")
            if joint.get("simulation_auxiliary") != "true"] == [
        joint.get("name") for joint in controlled
    ]
    for mesh in standalone.iter("mesh"):
        path = mesh.get("filename").split(f"/{MODEL.name}/", 1)[1]
        assert (MODEL / path).is_file()
    assert not standalone.findall("simulation_rod")
    assert (model.tendon_rgba[:, 3] == 0).all()
    assert len(standalone.findall("joint")) == 67
    assert len(standalone.findall(".//geometry")) == len(standalone.findall(".//geometry/mesh"))
    topology = yaml.safe_load((MODEL / "config/motor_topology.yaml").read_text())
    limits = yaml.safe_load((MODEL / "config/simulation_limits.yaml").read_text())
    assert set(limits["joint_limits"]) == {joint.get("name") for joint in controlled}
    for joint in controlled:
        name = joint.get("name")
        bound = limits["joint_limits"][name]
        exported = standalone.find(f"joint[@name='{name}']")
        assert exported.get("type") == "revolute"
        assert float(exported.find("limit").get("lower")) == bound["lower"]
        assert float(exported.find("limit").get("upper")) == bound["upper"]
        np.testing.assert_allclose(model.joint(name).range, [bound["lower"], bound["upper"]], atol=1e-9)
    addresses = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    direct = [(model.actuator(f"motor_{number:02d}").id, motor["joint"],
               float(np.clip(np.deg2rad(3 * (-1)**number),
                             limits["joint_limits"][motor["joint"]]["lower"],
                             limits["joint_limits"][motor["joint"]]["upper"])))
              for number, motor in topology["motors"].items() if "joint" in motor]
    for index, joint, _ in direct:
        assert addresses[index] == model.jnt_qposadr[model.joint(joint).id]

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nv - data.nefc == 37
    lengths = data.ten_length.copy()
    moving_links = [f"{group}_{side}" for group, side in product(("waist", "hips"), ("plus", "minus"))]
    moving_links += [f"{side}_shoulder_flange_linkage" for side in ("left", "right")]
    moving_links += [f"{side}_wrist_flange_{part}" for side, part in
                     product(("left", "right"), ("proximal", "distal"))]
    def flange_rotation(name):
        geom = data.geom(f"{name}_flange_visual")
        parent = model.body_parentid[model.geom_bodyid[geom.id]]
        return data.xmat[parent].reshape(3, 3).T @ geom.xmat.reshape(3, 3)

    flange_rotations = {name: flange_rotation(name).copy() for name in moving_links}
    for side, sign in (("plus", 1), ("minus", -1)):
        mount = data.site(f"hips_{side}_platform_ball").xpos
        np.testing.assert_allclose(mount, [sign * 0.07152, 0.05493050, 0.03120560], atol=1e-6)
        crank = data.site(f"hips_{side}_motor_ball").xpos - data.body(f"hips_{side}_flange").xpos
        assert sign * crank[0] > 0.029
    for joint in ET.parse(RUNTIME_URDF).getroot().findall("joint"):
        parent = data.body(joint.find("parent").get("link"))
        child = data.body(joint.find("child").get("link"))
        origin = joint.find("origin")
        parent_rotation = parent.xmat.reshape(3, 3)
        np.testing.assert_allclose(child.xpos, parent.xpos + parent_rotation @ np.fromstring(
            origin.get("xyz"), sep=" "), atol=1e-10)
        np.testing.assert_allclose(child.xmat.reshape(3, 3), parent_rotation @ Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy"), sep=" ")).as_matrix(), atol=1e-10)

    for label in ("Six groups same direction", "Six groups opposite direction"):
        key = model.key(label)
        mujoco.mj_resetDataKeyframe(model, data, key.id)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.ten_length, lengths, atol=1e-6)
        frames = {"Base_link": (np.zeros(3), Rotation.identity())}
        for joint in standalone.findall("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            position, rotation = frames[parent]
            position = position + rotation.apply(np.fromstring(origin.get("xyz"), sep=" "))
            rotation = rotation * Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
            rotation = rotation * Rotation.from_rotvec(
                np.fromstring(joint.find("axis").get("xyz"), sep=" ") * data.joint(joint.get("name")).qpos[0])
            frames[child] = position, rotation
        for index in range(1, model.nbody):
            body = data.body(index)
            position, rotation = frames[body.name]
            np.testing.assert_allclose(body.xpos, position, atol=1e-8)
            np.testing.assert_allclose(body.xmat.reshape(3, 3), rotation.as_matrix(), atol=1e-8)
        for name in moving_links:
            np.testing.assert_allclose(data.body(f"{name}_rod").xpos,
                                       data.site(f"{name}_motor_ball").xpos, atol=1e-10)
            np.testing.assert_allclose(data.site(f"{name}_rod_tip").xpos,
                                       data.site(f"{name}_platform_ball").xpos, atol=1e-6)
            for part in ('rod', 'flange', 'platform'):
                visible_geom = model.geom(f'{name}_{part}_visual')
                assert visible_geom.rgba[3] > 0
                mesh_file = ET.parse(MODEL / 'mujoco/full_body_robot.xml').find(
                    f"asset/mesh[@name='{name}_{part}_visual']").get('file')
                assert '/native/' in mesh_file
            assert model.geom_bodyid[model.geom(f'{name}_rod_visual').id] == model.body(f'{name}_rod').id
            assert model.geom_bodyid[model.geom(f'{name}_flange_visual').id] == model.site_bodyid[
                model.site(f'{name}_motor_ball').id]
            assert model.geom_bodyid[model.geom(f'{name}_platform_visual').id] == model.site_bodyid[
                model.site(f'{name}_platform_ball').id]
            geom = data.geom(f"{name}_flange_visual")
            joint = model.body_jntadr[model.geom_bodyid[geom.id]]
            angle = key.qpos[model.jnt_qposadr[joint]]
            assert np.linalg.norm(flange_rotation(name) - flange_rotations[name]) == pytest.approx(
                2 * np.sqrt(2) * abs(np.sin(angle / 2)), abs=1e-8)
        target, expected = key.ctrl.copy(), key.qpos.copy()
        for index, _, angle in direct:
            target[index] = expected[addresses[index]] = angle
        mujoco.mj_resetData(model, data)
        for fraction in np.linspace(0, 1, 401):
            data.ctrl[:] = target * fraction
            mujoco.mj_step(model, data)
            np.testing.assert_allclose(data.ten_length, lengths, atol=3e-5)
            equality = data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY
            assert np.max(np.abs(data.efc_pos[:data.nefc][equality])) < 3e-5
        mujoco.mj_step(model, data, nstep=2200)
        np.testing.assert_allclose(data.qpos, expected, atol=np.deg2rad(0.02))
        assert not any(warning.number for warning in data.warning)


def test_native_rods_retain_exact_source_component_geometry():
    import json
    import numpy as np
    from scipy.spatial import cKDTree
    vtk=pytest.importorskip('vtk')
    from vtk.util.numpy_support import vtk_to_numpy
    metadata=json.loads((MODEL/'config/native_assembly.json').read_text())
    count=0
    for source in metadata['ownership'].values():
        if not source['parts']:continue
        reader=vtk.vtkSTLReader();reader.SetFileName(str(MODEL.parent/'robot_body/meshes'/source['source']));reader.Update()
        regions=vtk.vtkPolyDataConnectivityFilter();regions.SetInputConnection(reader.GetOutputPort())
        regions.SetExtractionModeToAllRegions();regions.ColorRegionsOn();regions.Update()
        points=vtk_to_numpy(regions.GetOutput().GetPoints().GetData()).astype(float)
        ids=vtk_to_numpy(regions.GetOutput().GetPointData().GetScalars()).astype(int)
        for name,indices in source['parts'].items():
            if not name.endswith('_rod'):continue
            reader=vtk.vtkSTLReader();reader.SetFileName(str(MODEL/'mujoco/visual/native'/f'{name}.STL'));reader.Update()
            actual=vtk_to_numpy(reader.GetOutput().GetPoints().GetData()).astype(float)
            expected=points[np.isin(ids,indices)]
            assert max(cKDTree(expected).query(actual)[0].max(),cKDTree(actual).query(expected)[0].max())<1e-7,name
            count+=1
    assert count==10

@pytest.mark.parametrize('fault', ['region_count', 'duplicate', 'missing'])
def test_display_split_rejects_invalid_component_maps(tmp_path, fault):
    import importlib.util
    pytest.importorskip('mujoco')
    vtk = pytest.importorskip('vtk')
    spec = importlib.util.spec_from_file_location('full_body_builder', MODEL / 'tools/build_mujoco_model.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    points, faces = vtk.vtkPoints(), vtk.vtkCellArray()
    for region in range(7):
        start = points.GetNumberOfPoints()
        for x, y in ((0, 0), (1, 0), (0, 1)):
            points.InsertNextPoint(region * 3 + x, y, 0)
        faces.InsertNextCell(3)
        for index in range(3):
            faces.InsertCellPoint(start + index)
    mesh = vtk.vtkPolyData()
    mesh.SetPoints(points)
    mesh.SetPolys(faces)
    names = list(builder.CAD_CUTS['base_link.STL'][2])
    entry = {'source_region_count': 7, 'components': {name: [i] for i, name in enumerate(names)}}
    if fault == 'region_count':
        entry['source_region_count'] = 8
    elif fault == 'duplicate':
        entry['components'][names[1]] = [0]
    else:
        entry['components'][names[1]] = []
    with pytest.raises(ValueError):
        builder.split_registered_display(mesh, Path('base_link.STL'), tmp_path / 'base.STL',
                                         ET.Element('mujoco'), entry, tmp_path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("path", [RUNTIME_URDF, MAPPING_URDF], ids=lambda path: path.name)
def test_joint_tree_contains_all_38_links_and_37_joints(path):
    root = ET.parse(path).getroot()
    links, joints = root.findall("link"), root.findall("joint")
    names = {link.get("name") for link in links}
    assert len(links) == len(names) == 38
    assert names == set(LINK_NAMES.values())
    assert len(joints) == len({joint.get("name") for joint in joints}) == 37
    assert all(joint.get("type") == "revolute" for joint in joints)
    joint_names = [joint.get("name") for joint in joints]
    assert joint_names[:3] == ["BN_Spine_yaw", "BN_Spine_pitch", "BN_Spine_roll"]
    left_wrist = joint_names.index("BN_LeftHand_roll")
    assert joint_names[left_wrist:left_wrist + 2] == ["BN_LeftHand_roll", "BN_LeftHand_pitch"]
    children = {name: [] for name in names}
    child_links = set()
    for joint in joints:
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        assert parent in names and child in names
        assert child not in child_links
        assert parent == "Base_link" or parent in child_links
        child_links.add(child)
        children[parent].append(child)
    assert names - child_links == {"Base_link"}
    visited, pending = set(), ["Base_link"]
    while pending:
        link = pending.pop()
        assert link not in visited
        visited.add(link)
        pending.extend(children[link])
    assert visited == names


def test_mesh_copies_match_source_and_all_references_resolve():
    sources = {path.name for path in (SOURCE / "meshes").glob("*.STL")}
    assert sources == {path.name for path in (MODEL / "meshes").glob("*.STL")}
    assert len(sources) == 38
    for filename in sources:
        assert hashlib.sha256((SOURCE / "meshes" / filename).read_bytes()).digest() == (
            hashlib.sha256((MODEL / "meshes" / filename).read_bytes()).digest())
    prefix = f"package://robot_description/models/{MODEL.name}/"
    meshes = list(ET.parse(RUNTIME_URDF).getroot().iter("mesh"))
    assert len(meshes) == 76
    for mesh in meshes:
        filename = mesh.get("filename")
        assert filename.startswith(prefix)
        assert (MODEL / filename[len(prefix):]).is_file()


def test_preview_is_enumerable_without_invented_motor_mapping(monkeypatch):
    monkeypatch.syspath_prepend(str(PACKAGE.parent / "web_host"))
    from web_host.models import (
        enumerate_arm_models,
        has_joint_mapping_source,
        model_transmissions,
    )

    assert MODEL.name in enumerate_arm_models(MODEL.parent)
    assert not has_joint_mapping_source(MODEL.name, MODEL.parent)
    metadata = model_transmissions(MODEL.name, MODEL.parent)
    assert metadata['mapping_kind'] == 'topology'
    assert metadata['hardware_ready'] is False
    assert metadata['direct_joints'] == {}
