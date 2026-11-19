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

JOINT_METADATA = __import__('json').loads((MODEL / 'config/joint_names.json').read_text())
NAME_ALIASES = JOINT_METADATA['joints']
COORDINATES = JOINT_METADATA['native_joint_coordinates']
NATIVE_NAMES = {name: native for native, (name, _) in COORDINATES.items()}
JOINT_NAMES = {old: NAME_ALIASES.get(name, COORDINATES.get(name, (name, 1))[0])
               for old, name in JOINT_NAMES.items()}
LINK_NAMES = {"base_link": "Base_link"}
LINK_NAMES.update({original: f"{NATIVE_NAMES[name]}_link" for original, name in JOINT_NAMES.items()})
CAD_TO_MODEL = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]


def _xml_shape(element):
    return (element.tag, element.attrib, (element.text or "").strip(),
            [_xml_shape(child) for child in element])


def test_original_urdf_is_unchanged():
    assert hashlib.sha256(ORIGINAL_URDF.read_bytes()).hexdigest() == (
        "76d4278667151ffd7f26b4e21ad75b3032ada97c346c76b808d1b83139c11920")







@pytest.mark.parametrize('runtime_name', [
    'full_body_robot.urdf', 'full_body_robot.mapping.urdf',
    'full_body_robot.standalone.urdf',
])
def test_runtime_inertias_preserve_native_export_world_com_and_tensor(runtime_name):
    import json
    import numpy as np
    from scipy.spatial.transform import Rotation
    source = ET.parse(MODEL.parent / 'robot_body/urdf/cnm.urdf').getroot()
    original = {link.get('name').strip(): link for link in source.findall('link')}
    arm_source = ET.parse(
        MODEL.parent / '全身实验版_仅双臂质量/urdf/全身实验版_仅双臂质量.urdf'
    ).getroot()
    arm_links = {
        link.get('name').strip(): link for link in arm_source.findall('link')
        if any(part in link.get('name') for part in ('Shoulder', 'Arm', 'Hand'))
    }
    assert len(arm_links) == 18
    original.update(arm_links)
    native = json.loads((MODEL / 'config/step_alignment.json').read_text())['links']
    runtime = ET.parse(MODEL / 'urdf' / runtime_name).getroot()
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
        name=link.get('name')
        if name not in original:
            continue
        actual=inertia(link,frames[name])
        expected_frame=np.asarray(native[name]['world_transform'])
        mounting=json.loads((MODEL/'config/arm_mount_alignment.json').read_text())
        if name in mounting['affected_links']:
            expected_frame=np.asarray(mounting['world_transform'])@expected_frame
        expected_frame = expected_frame.copy()
        expected_frame[:3] = np.asarray(CAD_TO_MODEL) @ expected_frame[:3]
        expected=inertia(original[name],expected_frame)
        np.testing.assert_allclose(actual[0],expected[0],atol=1e-10)
        np.testing.assert_allclose(actual[1],expected[1],atol=1e-10)
        assert actual[2]==expected[2]
    if runtime_name == RUNTIME_URDF.name:
        assert _xml_shape(runtime)==_xml_shape(ET.parse(MAPPING_URDF).getroot())



def test_mapping_urdf_uses_same_joint_names_by_child_link():
    expected = {LINK_NAMES[old]: name for old, name in JOINT_NAMES.items()}
    actual = {joint.find("child").get("link"): joint.get("name")
              for joint in ET.parse(MAPPING_URDF).getroot().findall("joint")}
    assert actual == expected






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
    assert joint_names[:3] == ["waist_yaw", "waist_pitch", "waist_roll"]
    left_wrist = joint_names.index("left_wrist_pitch")
    assert joint_names[left_wrist:left_wrist + 2] == ["left_wrist_pitch", "left_wrist_roll"]
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


def test_complete_display_skeleton_and_component_ownership():
    import json
    ownership = json.loads((MODEL / 'config/native_assembly.json').read_text())['ownership']
    main = ET.parse(RUNTIME_URDF).getroot()
    assert set(ownership) == {link.get('name') for link in main.findall('link')}
    for entry in ownership.values():
        assigned = entry['fixed_regions'] + [i for part in entry['parts'].values() for i in part]
        assigned += [i for rotor in entry['rotor_regions'].values() for i in rotor]
        assert sorted(assigned) == list(range(len(assigned)))
        source = MODEL.parent / 'robot_body/meshes' / entry['source']
        assert hashlib.sha256(source.read_bytes()).hexdigest() == entry['source_sha256']
    display = ET.parse(MODEL / 'urdf/full_body_robot.standalone.urdf').getroot()
    assert len(display.findall('joint')) == 67
    assert len(display.findall('.//visual')) == 68
    assert all('/visual/native/' in mesh.get('filename') for mesh in display.iter('mesh'))
    controlled = [joint.get('name') for joint in display.findall('joint')
                  if joint.get('simulation_auxiliary') != 'true']
    assert controlled == [joint.get('name') for joint in main.findall('joint')]
