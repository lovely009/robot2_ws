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
    "Joint_Link3": "BN_Spine_axis_1",
    "Joint_Link2": "BN_Spine_axis_2",
    "Joint_Link4": "BN_Hips_axis_1",
    "Joint_Link5": "BN_Hips_axis_2",
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

LINK_NAMES = {"base_link": "base_link"}
LINK_NAMES.update({original: f"{name}_link" for original, name in JOINT_NAMES.items()})


def _xml_shape(element):
    return (element.tag, element.attrib, (element.text or "").strip(),
            [_xml_shape(child) for child in element])


def test_original_urdf_is_unchanged():
    assert hashlib.sha256(ORIGINAL_URDF.read_bytes()).hexdigest() == (
        "76d4278667151ffd7f26b4e21ad75b3032ada97c346c76b808d1b83139c11920")


def test_runtime_urdf_matches_cad_with_waist_and_left_wrist_corrections():
    original = ET.parse(ORIGINAL_URDF).getroot()
    original.set("name", MODEL.name)
    for mesh in original.iter("mesh"):
        mesh.set("filename", mesh.get("filename").replace(
            f"package://{ORIGINAL_NAME}/", f"package://robot_description/models/{MODEL.name}/"))
    for link in original.findall("link"):
        link.set("name", LINK_NAMES[link.get("name")])
    for joint in original.findall("joint"):
        joint.set("name", JOINT_NAMES[joint.get("name")])
        for tag in ("parent", "child"):
            reference = joint.find(tag)
            reference.set("link", LINK_NAMES[reference.get("link")])

    corrected_parents = {
        "BN_Spine_axis_2": "BN_Spine_axis_1_link",
        "BN_LeftHand_axis_1": "BN_LeftLowerArm_axis_2_link",
        "BN_LeftHand_axis_2": "BN_LeftHand_axis_1_link",
    }
    mapping = ET.parse(MAPPING_URDF).getroot()
    for name, parent in corrected_parents.items():
        joint = original.find(f"joint[@name='{name}']")
        joint.find("parent").set("link", parent)
        origin = joint.find("origin")
        origin.attrib = mapping.find(f"joint[@name='{name}']/origin").attrib.copy()

    runtime = ET.parse(RUNTIME_URDF).getroot()
    for root in (runtime, original):
        root[:] = sorted(root, key=lambda element: (element.tag, element.get("name")))
    assert _xml_shape(runtime) == _xml_shape(original)


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
        angles = np.rad2deg([data.joint(f"{joint_prefix}_axis_1").qpos[0],
                            data.joint(f"{joint_prefix}_axis_2").qpos[0]])
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
    addresses = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    direct = [(model.actuator(f"motor_{number:02d}").id, motor["joint"], np.deg2rad(3 * (-1)**number))
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
        np.testing.assert_allclose(mount, [sign * 0.07152, 0.064277, 0.025742], atol=1e-6)
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
        frames = {"base_link": (np.zeros(3), Rotation.identity())}
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
            assert np.linalg.norm(flange_rotation(name) - flange_rotations[name]) > 0.05
        target, expected = key.ctrl.copy(), key.qpos.copy()
        for index, _, angle in direct:
            target[index] = expected[addresses[index]] = angle
        mujoco.mj_resetData(model, data)
        for fraction in np.linspace(0, 1, 401):
            data.ctrl[:] = target * fraction
            mujoco.mj_step(model, data)
            np.testing.assert_allclose(data.ten_length, lengths, atol=3e-5)
            assert np.max(np.abs(data.efc_pos[:data.nefc])) < 3e-5
        mujoco.mj_step(model, data, nstep=2200)
        np.testing.assert_allclose(data.qpos, expected, atol=np.deg2rad(0.02))
        assert not any(warning.number for warning in data.warning)


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
    assert joint_names[:3] == ["BN_Spine_yaw", "BN_Spine_axis_1", "BN_Spine_axis_2"]
    left_wrist = joint_names.index("BN_LeftHand_axis_1")
    assert joint_names[left_wrist:left_wrist + 2] == ["BN_LeftHand_axis_1", "BN_LeftHand_axis_2"]
    children = {name: [] for name in names}
    child_links = set()
    for joint in joints:
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        assert parent in names and child in names
        assert child not in child_links
        assert parent == "base_link" or parent in child_links
        child_links.add(child)
        children[parent].append(child)
    assert names - child_links == {"base_link"}
    visited, pending = set(), ["base_link"]
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
    with pytest.raises(ValueError, match="motor_driver.yaml"):
        model_transmissions(MODEL.name, MODEL.parent)
