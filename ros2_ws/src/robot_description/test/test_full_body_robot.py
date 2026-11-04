"""全身 CAD 预览件保真：仅适配模型名和网格 URI，不补未经确认的物理参数。"""

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE = Path(__file__).parents[1]
MODEL = PACKAGE / "models" / "full_body_robot"
SOURCE = MODEL / "cad"
ORIGINAL_NAME = "全身装配（不含头部）"
ORIGINAL_URDF = SOURCE / "urdf" / f"{ORIGINAL_NAME}.urdf"
RUNTIME_URDF = MODEL / "urdf" / f"{MODEL.name}.urdf"


def _xml_shape(element):
    return (element.tag, element.attrib, (element.text or "").strip(),
            [_xml_shape(child) for child in element])


def test_original_urdf_is_unchanged():
    assert hashlib.sha256(ORIGINAL_URDF.read_bytes()).hexdigest() == (
        "76d4278667151ffd7f26b4e21ad75b3032ada97c346c76b808d1b83139c11920")


def test_runtime_urdf_preserves_all_physical_parameters():
    original = ET.parse(ORIGINAL_URDF).getroot()
    original.set("name", MODEL.name)
    for mesh in original.iter("mesh"):
        mesh.set("filename", mesh.get("filename").replace(
            f"package://{ORIGINAL_NAME}/", f"package://robot_description/models/{MODEL.name}/"))

    assert _xml_shape(ET.parse(RUNTIME_URDF).getroot()) == _xml_shape(original)


def test_joint_tree_contains_all_38_links_and_37_joints():
    root = ET.parse(RUNTIME_URDF).getroot()
    links, joints = root.findall("link"), root.findall("joint")
    names = {link.get("name") for link in links}
    assert len(links) == len(names) == 38
    assert len(joints) == len({joint.get("name") for joint in joints}) == 37
    assert all(joint.get("type") == "revolute" for joint in joints)
    children = {name: [] for name in names}
    child_links = set()
    for joint in joints:
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        assert parent in names and child in names
        assert child not in child_links
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
