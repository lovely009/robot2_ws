#!/usr/bin/env python3
"""生成 STEP 原生装配整机；保留旧 CAD 工具供分件与源几何审计。"""

from collections import defaultdict
from copy import deepcopy
import hashlib
import json
import os
from tempfile import TemporaryDirectory
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import vtk
import yaml


MODEL = Path(__file__).resolve().parents[1]
OUTPUT = MODEL / "mujoco/full_body_robot.xml"
STANDALONE = MODEL / "urdf/full_body_robot.standalone.urdf"
STL_FACE_LIMIT = 200_000
DISPLAY_FACE_TARGET = 120_000
LIMITS = MODEL / "config/simulation_limits.yaml"
LOCKED_SOLVER_RANGE = 1e-9

# Region numbers refer to the original CAD STL exports, before display decimation.
CAD_CUTS = {
    "Joint_Link1.STL": (
        "d7080437a8c4732a4b025528b3cef5cc112d06b117f2645affc4678dbc1895a6", 171, {
            "waist_plus_flange": [156, 40, 41, 42, 43, 77, 164, 167],
            "waist_plus_rod": [78],
            "waist_plus_platform": [79, 158, 168],
            "waist_minus_flange": [162, 118, 119, 120, 121, 161, 163, 169],
            "waist_minus_rod": [159],
            "waist_minus_platform": [1, 157, 165],
        },
    ),
    "base_link.STL": (
        "db4c7dee5e2d7c9c817446a6d2766f488c9e7ec0459b50c8ef49746a49b5004f", 276, {
            "hips_plus_flange": [227, 230, 232, 239, 250, 251, 252, 259, 260, 262, 266, 268],
            "hips_plus_rod": [236],
            "hips_plus_platform": [229, 238, 240],
            "hips_minus_flange": [226, 234, 235, 237, 247, 248, 253, 254, 255, 256, 257, 258],
            "hips_minus_rod": [231],
            "hips_minus_platform": [228, 233, 241],
        },
    ),
    "Joint_Link2.STL": (
        "64aaebe00ccfa39178f0155d0130fb5f92207c6b10ae417424b57a421d67a7ef", 59, {
            "left_shoulder_flange_linkage_flange": [3, 4, 6],
            "left_shoulder_flange_linkage_rod": [5, 8, 12],
            "left_shoulder_flange_linkage_platform": [7, 13],
            "right_shoulder_flange_linkage_flange": [48, 49, 50],
            "right_shoulder_flange_linkage_rod": [42, 45, 47],
            "right_shoulder_flange_linkage_platform": [44, 46],
        },
    ),
    "Joint_link3.7.STL": (
        "a60250a2d6bb339de1e71a58c8a9f77c434a8e5c1418c02bc7eb4f1ec3d324a9", 40, {
            "left_wrist_flange_proximal_flange": [2, 38],
            "left_wrist_flange_proximal_rod": [17],
            "left_wrist_flange_proximal_platform": [19],
            "left_wrist_flange_distal_flange": [0, 39],
            "left_wrist_flange_distal_rod": [18],
            "left_wrist_flange_distal_platform": [20],
        },
    ),
    "Joint_link3.7r.STL": (
        "7a5b6e0b7d5baa2e030599df8aebd1236a17c7341b778b136e281149368c87c9", 41, {
            "right_wrist_flange_proximal_flange": [2, 39],
            "right_wrist_flange_proximal_rod": [18],
            "right_wrist_flange_proximal_platform": [20],
            "right_wrist_flange_distal_flange": [0, 40],
            "right_wrist_flange_distal_rod": [19],
            "right_wrist_flange_distal_platform": [21],
        },
    ),
}

# The wrist STL joins the recessed rotor and stationary end cover into one region.
WRIST_ROTORS = {
    "Joint_link3.7.STL": ("left_wrist", {"proximal": 6, "distal": 25}),
    "Joint_link3.7r.STL": ("right_wrist", {"proximal": 6, "distal": 26}),
}
WRIST_ROTOR_RADIUS = 0.00855

# Source STL placement of the hips pieces, before aligning them to the crossbeam holes.
HIPS_CAD_BALLS = {
    "plus": ((0.075922944, 0.023653986, 0.028), (0.10532156, 0.081621874, 0.027332679),
             (-0.01128057, 0.05793355, 0.99825671)),
    "minus": ((-0.063494767, 0.022800067, 0.028), (-0.073238621, 0.08660591, 0.020327214),
              (0.16205043, 0.13625783, 0.97732976)),
}
HIPS_CROSSBEAM_NORMAL = (2.19096428e-7, 0.0361379527, 0.999346811)


def numbers(values):
    return " ".join(format(float(value), ".14g") for value in values)


def pose(origin):
    return (np.fromstring(origin.get("xyz"), sep=" "),
            Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")))


def pose_attributes(position, rotation):
    return {"pos": numbers(position), "quat": numbers(rotation.as_quat()[[3, 0, 1, 2]])}


def urdf_rpy(rotation):
    """Keep near-gimbal rotations exact when expressing MJCF frames in URDF."""
    matrix = rotation.as_matrix()
    cosine = np.hypot(matrix[0, 0], matrix[1, 0])
    pitch = np.arctan2(-matrix[2, 0], cosine)
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0]) if cosine > 1e-12 else 0.
    sine_yaw, cosine_yaw = np.sin(yaw), np.cos(yaw)
    roll = np.arctan2(sine_yaw * matrix[0, 2] - cosine_yaw * matrix[1, 2],
                      -sine_yaw * matrix[0, 1] + cosine_yaw * matrix[1, 1])
    return np.array([roll, pitch, yaw])


def write_display_mesh(polydata, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    faces = polydata.GetNumberOfPolys()
    if faces >= STL_FACE_LIMIT:
        decimator = vtk.vtkQuadricDecimation()
        decimator.SetInputData(polydata)
        decimator.SetTargetReduction(1 - DISPLAY_FACE_TARGET / faces)
        decimator.Update()
        polydata = decimator.GetOutput()
        faces = polydata.GetNumberOfPolys()
    if not 0 < faces < STL_FACE_LIMIT:
        raise ValueError(f"{output.name}: invalid display mesh ({faces} faces)")
    writer = vtk.vtkSTLWriter()
    writer.SetInputData(polydata)
    writer.SetFileTypeToBinary()
    writer.SetFileName(str(output))
    if not writer.Write():
        raise OSError(f"Could not write {output}")


def split_wrist_rotor(polydata, body):
    cylinder = vtk.vtkCylinder()
    cylinder.SetCenter(*np.fromstring(body.get("pos"), sep=" "))
    cylinder.SetAxis(*np.fromstring(body.find("joint").get("axis"), sep=" "))
    cylinder.SetRadius(WRIST_ROTOR_RADIUS)
    clip = vtk.vtkClipPolyData()
    clip.SetInputData(polydata)
    clip.SetClipFunction(cylinder)
    clip.InsideOutOn()
    clip.GenerateClippedOutputOn()
    clip.Update()
    for piece in (clip.GetOutput(), clip.GetClippedOutput()):
        if not piece.GetNumberOfPolys():
            raise ValueError(f"{body.get('name')}: wrist rotor split did not intersect the mesh")
    return clip.GetOutput(), clip.GetClippedOutput()


def display_mesh(source):
    with source.open("rb") as stream:
        stream.seek(80)
        count, = struct.unpack("<I", stream.read(4))
    cut = CAD_CUTS.get(source.name)
    if not cut and count < STL_FACE_LIMIT:
        return source
    output = OUTPUT.parent / "visual" / source.name
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    reader.Update()
    if cut is None:
        write_display_mesh(reader.GetOutput(), output)
        return output

    digest, region_count, parts = cut
    if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
        raise ValueError(f"{source.name}: review CAD regions after changing the source mesh")
    regions = vtk.vtkPolyDataConnectivityFilter()
    regions.SetInputConnection(reader.GetOutputPort())
    regions.SetExtractionModeToAllRegions()
    regions.Update()
    if regions.GetNumberOfExtractedRegions() != region_count:
        raise ValueError(f"{source.name}: unexpected CAD region count")
    regions.SetExtractionModeToSpecifiedRegions()
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(regions.GetOutputPort())

    def extract(indices):
        regions.InitializeSpecifiedRegionList()
        for index in indices:
            regions.AddSpecifiedRegion(index)
        clean.Update()
        return clean.GetOutput()

    extra = defaultdict(list)
    moving = {index for indices in parts.values() for index in indices}
    if source.name in WRIST_ROTORS:
        group, rotors = WRIST_ROTORS[source.name]
        module = ET.parse(OUTPUT.parent / f"{group}_differential.xml").getroot()
        for side, index in rotors.items():
            body = module.find(f"worldbody/body[@name='flange_{side}_body']")
            rotor, cover = split_wrist_rotor(extract([index]), body)
            moving.add(index)
            extra[f"{group}_flange_{side}_flange"].append(rotor)
            extra[source.stem].append(cover)
    fixed = sorted(set(range(region_count)) - moving)
    for name, indices in [(source.stem, fixed), *parts.items()]:
        mesh = extract(indices)
        if extra[name]:
            append = vtk.vtkAppendPolyData()
            for piece in (mesh, *extra[name]):
                append.AddInputData(piece)
            append.Update()
            mesh = append.GetOutput()
        write_display_mesh(mesh, output.with_name(f"{name}.STL"))
    return output


def add_visual_and_inertia(body, link, assets):
    inertial = link.find("inertial")
    center, rotation = pose(inertial.find("origin"))
    tensor = inertial.find("inertia").attrib
    xx, yy, zz, xy, xz, yz = (float(tensor[key]) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
    inertia = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    frame = rotation.as_matrix()
    inertia = frame @ inertia @ frame.T
    components = inertia[(0, 1, 2, 0, 0, 1), (0, 1, 2, 1, 2, 2)]
    ET.SubElement(body, "inertial", pos=numbers(center), mass=inertial.find("mass").get("value"),
                  fullinertia=numbers(components))
    visual = link.find("visual")
    source = MODEL / "meshes" / Path(visual.find("geometry/mesh").get("filename")).name
    mesh = display_mesh(source)
    name = link.get("name")
    ET.SubElement(assets, "mesh", name=name, file=mesh.relative_to(MODEL).as_posix())
    ET.SubElement(body, "geom", name=f"{name}_visual", type="mesh", mesh=name, mass="0", group="1",
                  rgba=visual.find("material/color").get("rgba"), **pose_attributes(*pose(visual.find("origin"))))


def add_cad_geom(body, assets, name, position, rotation):
    ET.SubElement(assets, "mesh", name=name, file=f"mujoco/visual/{name}.STL")
    ET.SubElement(body, "geom", name=f"{name}_visual", mesh=name, type="mesh", group="1", mass="0",
                  rgba="0.76 0.78 0.81 1", **pose_attributes(position, rotation))


def rod_frame(direction, transverse_reference):
    z = direction / np.linalg.norm(direction)
    x = transverse_reference - np.dot(transverse_reference, z) * z
    x /= np.linalg.norm(x)
    return Rotation.from_matrix(np.column_stack((x, np.cross(z, x), z)))


def add_cad_linkage(name, model_data, lower, upper, flange, platform, platform_pose, site_names, assets, equalities):
    side = lower.removesuffix("_motor_ball")
    prefix = f"{name}_{side}"
    start = model_data.site(lower).xpos
    end = model_data.site(upper).xpos
    length = np.linalg.norm(end - start)
    orientation = rod_frame(end - start, np.array([1., 0., 0.]))
    flange_position = np.fromstring(flange.get("pos"), sep=" ")
    if name == "hips":
        old_start, old_end, old_normal = map(np.asarray, HIPS_CAD_BALLS[side])
        flange_turn, _ = Rotation.align_vectors([start - flange_position], [old_start - flange_position])
        platform_turn, _ = Rotation.align_vectors([HIPS_CROSSBEAM_NORMAL], [old_normal])
        vertical = np.array([0., 0., 1.])
        rod_turn = rod_frame(end - start, vertical) * rod_frame(old_end - old_start, vertical).inv()
    elif name == "waist":
        # Fit the 102 mm waist CAD parts to the nominal 100 mm linkage spacing.
        shift = np.array([-np.sign(start[0]) * 0.001, 0, 0])
        old_start, old_end = start - shift, end - shift
        flange_turn = platform_turn = rod_turn = Rotation.identity()
    else:
        old_start, old_end = start, end
        flange_turn = platform_turn = rod_turn = Rotation.identity()
    add_cad_geom(flange, assets, f"{prefix}_flange", start - flange_position - flange_turn.apply(old_start),
                 flange_turn)
    platform_position, platform_rotation = platform_pose
    add_cad_geom(platform, assets, f"{prefix}_platform",
                 platform_position + platform_rotation.apply(end - platform_turn.apply(old_end)),
                 platform_rotation * platform_turn)

    rod_name = f"{prefix}_rod"
    rod = ET.SubElement(flange, "body", name=rod_name, **pose_attributes(start - flange_position, orientation))
    for suffix, axis in (("x", "1 0 0"), ("y", "0 1 0")):
        ET.SubElement(rod, "joint", name=f"{rod_name}_{suffix}", axis=axis, damping="0.001", armature="0.00005")
    transverse_inertia = 0.01 * length**2 / 12
    ET.SubElement(rod, "inertial", pos=numbers((0, 0, length / 2)), mass="0.01",
                  diaginertia=numbers((transverse_inertia, transverse_inertia, 1e-7)))
    mesh_rotation = orientation.inv() * rod_turn
    add_cad_geom(rod, assets, rod_name, -mesh_rotation.apply(old_start), mesh_rotation)
    tip = f"{rod_name}_tip"
    ET.SubElement(rod, "site", name=tip, pos=numbers((0, 0, length)), rgba="0 0 0 0")
    ET.SubElement(equalities, "connect", name=f"{rod_name}_end", site1=tip, site2=site_names[upper],
                  solref="0.006 1", solimp="0.999 0.9999 0.001")
    return rod_name, lower, upper, orientation


def add_mechanism(name, group, joints, bodies, frames, assets, tendons, equalities):
    path = OUTPUT.parent / f"{name}_differential.xml"
    module = ET.parse(path).getroot()
    reference = joints[group["joints"][0]].find("parent").get("link")
    output = joints[group["joints"][-1]].find("child").get("link")
    ref_position, ref_rotation = frames[reference]
    out_position, out_rotation = frames[output]
    platform_pose = (out_rotation.inv().apply(ref_position - out_position),
                     out_rotation.inv() * ref_rotation)

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site_names = {}
    flanges = {}
    for source_body in module.findall("worldbody/body"):
        joint = source_body.find("joint")
        if joint.get("name") in joints:
            continue
        flange = deepcopy(source_body)
        flange.set("name", f"{name}_{flange.get('name')}")
        flange.find("joint").set("name", f"{name}_{joint.get('name')}")
        for site in flange.findall("site"):
            original = site.get("name")
            site_names[original] = f"{name}_{original}"
            site.set("name", site_names[original])
            flanges[original] = flange
            site.set("rgba", "0 0 0 0")
        for geom in flange.findall("geom"):
            geom.set("group", "2")
            geom.set("rgba", "0 0 0 0")
        bodies[reference].append(flange)

    rods = []
    for spatial in module.findall("tendon/spatial"):
        rod = deepcopy(spatial)
        rod.set("name", f"{name}_{spatial.get('name')}")
        for endpoint in rod.findall("site"):
            original = endpoint.get("site")
            if original not in site_names:
                position = ref_position + ref_rotation.apply(data.site(original).xpos)
                local = out_rotation.inv().apply(position - out_position)
                source_site = module.find(f".//body/site[@name='{original}']")
                site_names[original] = f"{name}_{original}"
                attributes = {
                    **source_site.attrib, "name": site_names[original], "pos": numbers(local),
                }
                attributes["rgba"] = "0 0 0 0"
                ET.SubElement(bodies[output], "site", attributes)
            endpoint.set("site", site_names[original])
        lower, upper = (site.get("site") for site in spatial.findall("site"))
        rods.append(add_cad_linkage(name, data, lower, upper, flanges[lower], bodies[output],
                                    platform_pose, site_names, assets, equalities))
        rod.set("rgba", "0 0 0 0")
        tendons.append(rod)

    controls = []
    for source in module.findall("actuator/position"):
        joint = source.get("joint")
        controls.append({
            "name": f"{name}_{source.get('name')}",
            "joint": joint if joint in joints else f"{name}_{joint}",
        })
    if len(controls) != len(group["motor_numbers"]):
        raise ValueError(f"{name}: unexpected actuator count")
    return controls, (model, rods)


def add_keyframes(root, world, actuators, modules):
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(keys, "key", name="Neutral")
    for label, local_key in (
        ("Six groups same direction", "Same direction +5 deg"),
        ("Six groups opposite direction", "Opposite direction +5 / -5 deg"),
    ):
        positions = {joint.get("name"): 0.0 for joint in world.iter("joint")}
        inputs = {actuator.get("name"): 0.0 for actuator in actuators}
        for name, (module, rods) in modules.items():
            key = module.key(local_key)
            for index in range(module.njnt):
                joint = module.joint(index).name
                full_name = joint if joint in positions else f"{name}_{joint}"
                positions[full_name] = key.qpos[module.jnt_qposadr[index]]
            for index in range(module.nu):
                inputs[f"{name}_{module.actuator(index).name}"] = key.ctrl[index]
            if rods:
                data = mujoco.MjData(module)
                mujoco.mj_resetDataKeyframe(module, data, key.id)
                mujoco.mj_forward(module, data)
                for rod, lower, upper, orientation in rods:
                    anchor = data.site(lower)
                    flange_rotation = data.xmat[module.site_bodyid[anchor.id]].reshape(3, 3)
                    direction = orientation.inv().apply(flange_rotation.T @ (data.site(upper).xpos - anchor.xpos))
                    direction /= np.linalg.norm(direction)
                    positions[f"{rod}_x"] = np.arctan2(-direction[1], direction[2])
                    positions[f"{rod}_y"] = np.arcsin(np.clip(direction[0], -1.0, 1.0))
        ET.SubElement(keys, "key", name=label, qpos=numbers(positions.values()), ctrl=numbers(inputs.values()))


def write_standalone_urdf(mjcf, controlled_joints, limits):
    """发布与仿真一致的 TF 和 CAD 外观，包括六组差动运动件。"""
    robot = ET.Element("robot", name=MODEL.name)
    robot.append(ET.Comment(" Generated from the whole-body MJCF for simulation visualization. "))
    files = {mesh.get("name"): mesh.get("file") for mesh in mjcf.findall("asset/mesh")}
    uri = f"package://robot_description/models/{MODEL.name}/"

    def add_origin(element, source):
        quaternion = np.fromstring(source.get("quat", "1 0 0 0"), sep=" ")
        rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
        ET.SubElement(element, "origin", xyz=source.get("pos", "0 0 0"),
                      rpy=numbers(urdf_rpy(rotation)))

    def add_visual(link, name, origin, rgba, shape, **dimensions):
        visual = ET.SubElement(link, "visual", name=name)
        add_origin(visual, origin)
        ET.SubElement(ET.SubElement(visual, "geometry"), shape, **dimensions)
        material = ET.SubElement(visual, "material", name=f"{name}_color")
        ET.SubElement(material, "color", rgba=rgba)

    def add_body(body, parent):
        name = body.get("name")
        joints = body.findall("joint")
        for index, source in enumerate(joints):
            child = name if index == len(joints) - 1 else f"{name}_axis_frame_{index + 1}"
            if child != name:
                ET.SubElement(robot, "link", name=child)
            joint = ET.SubElement(robot, "joint", name=source.get("name"), type="continuous")
            if source.get("name") not in controlled_joints:
                joint.set("simulation_auxiliary", "true")
            else:
                joint.set("type", "revolute")
                bound = limits["joint_limits"][source.get("name")]
                ET.SubElement(joint, "limit", lower=str(bound["lower"]), upper=str(bound["upper"]),
                              effort="0", velocity="0")
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=child)
            add_origin(joint, body if index == 0 else ET.Element("origin"))
            ET.SubElement(joint, "axis", xyz=source.get("axis"))
            parent = child
        link = ET.SubElement(robot, "link", name=name)
        for geom in body.findall("geom[@type='mesh']"):
            if float(geom.get("rgba", "1 1 1 1").split()[-1]) == 0:
                continue
            add_visual(link, geom.get("name"), geom, geom.get("rgba"), "mesh",
                       filename=uri + files[geom.get("mesh")])
        for child in body.findall("body"):
            add_body(child, name)

    add_body(mjcf.find("worldbody/body"), None)
    ET.indent(robot, space="  ")
    STANDALONE.write_text(ET.tostring(robot, encoding="unicode") + "\n")


def split_registered_display(mesh, reference, output, root, entry, final_directory):
    """Split hash-guarded, reviewed source regions onto the existing mechanism bodies."""
    regions = vtk.vtkPolyDataConnectivityFilter()
    regions.SetInputData(mesh)
    regions.SetExtractionModeToAllRegions()
    regions.Update()
    count = regions.GetNumberOfExtractedRegions()
    if count != entry["source_region_count"]:
        raise ValueError(f"{reference.name}: review CAD component ownership after changing regions")
    parts = entry["components"]
    if set(parts) != set(CAD_CUTS[reference.name][2]):
        raise ValueError(f"{reference.name}: incomplete moving component map")
    moving = [index for indices in parts.values() for index in indices]
    moving += [index for indices in entry.get("rotor_regions", {}).values() for index in indices]
    if len(moving) != len(set(moving)) or any(i < 0 or i >= count for i in moving):
        raise ValueError(f"{reference.name}: duplicate or invalid moving region")
    if any(not indices for indices in parts.values()):
        raise ValueError(f"{reference.name}: missing moving component")
    regions.SetExtractionModeToSpecifiedRegions()

    def extract(indices):
        regions.InitializeSpecifiedRegionList()
        for index in indices:
            regions.AddSpecifiedRegion(index)
        clean = vtk.vtkCleanPolyData()
        clean.SetInputConnection(regions.GetOutputPort())
        clean.Update()
        result = vtk.vtkPolyData()
        result.DeepCopy(clean.GetOutput())
        return result

    extra = defaultdict(list)
    if reference.name in WRIST_ROTORS:
        group, _ = WRIST_ROTORS[reference.name]
        module = ET.parse(OUTPUT.parent / f"{group}_differential.xml").getroot()
        if set(entry["rotor_regions"]) != {"proximal", "distal"}:
            raise ValueError(f"{reference.name}: incomplete wrist rotor map")
        for side, indices in entry["rotor_regions"].items():
            body = module.find(f"worldbody/body[@name='flange_{side}_body']")
            rotor, cover = split_wrist_rotor(extract(indices), body)
            extra[f"{group}_flange_{side}_flange"].append(rotor)
            extra[reference.stem].append(cover)
    fixed = sorted(set(range(count)) - set(moving))
    static = None
    for name, indices in [(reference.stem, fixed), *parts.items()]:
        append = vtk.vtkAppendPolyData()
        for piece in (extract(indices), *extra[name]):
            append.AddInputData(piece)
        append.Update()
        if name == reference.stem:
            static = vtk.vtkPolyData()
            static.DeepCopy(append.GetOutput())
        else:
            path = output.with_name(f"{name}.STL")
            write_display_mesh(append.GetOutput(), path)
            final_path = final_directory / path.name
            root.find(f"asset/mesh[@name='{name}']").set("file", final_path.relative_to(MODEL).as_posix())
            root.find(f".//geom[@name='{name}_visual']").set("rgba", "0.76 0.78 0.81 1")
    return static


def apply_display_meshes(root):
    """Attach reviewed CAD visuals without importing its physics or joint definitions."""
    manifest = MODEL / "config/display_meshes.json"
    if not manifest.is_file():
        return
    metadata = json.loads(manifest.read_text())
    entries = metadata["links"]
    source_package = metadata["source"]
    for entry in entries.values():
        for path, expected in (
            (MODEL.parent / source_package / "meshes" / entry["source"], entry["source_sha256"]),
            (MODEL / "meshes" / entry["reference"], entry["reference_sha256"]),
        ):
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError(f"{path.name}: review display registration and component ownership")
    bodies = {body.get("name"): body for body in root.findall(".//body")}
    assets = root.find("asset")
    originals = {name: deepcopy(bodies[name].find(f"geom[@name='{name}_visual']")) for name in entries}
    for geom in root.findall(".//geom[@type='mesh']"):
        rgba = geom.get("rgba", "1 1 1 1").split()
        geom.set("rgba", " ".join([*rgba[:3], "0"]))
    final_directory = MODEL / "mujoco/visual" / source_package
    final_directory.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".display-", dir=final_directory.parent) as staging:
        for name, entry in entries.items():
            source = MODEL.parent / source_package / "meshes" / entry["source"]
            reference = MODEL / "meshes" / entry["reference"]
            reader = vtk.vtkSTLReader()
            reader.SetFileName(str(source))
            reader.Update()
            matrix = vtk.vtkMatrix4x4()
            for row in range(4):
                for column in range(4):
                    matrix.SetElement(row, column, entry["new_to_old"][row][column])
            transform = vtk.vtkTransform()
            transform.SetMatrix(matrix)
            mapped = vtk.vtkTransformPolyDataFilter()
            mapped.SetInputConnection(reader.GetOutputPort())
            mapped.SetTransform(transform)
            mapped.Update()
            output = Path(staging) / f"{name}.STL"
            mesh = mapped.GetOutput()
            if reference.name in CAD_CUTS:
                mesh = split_registered_display(mesh, reference, output, root, entry, final_directory)
            write_display_mesh(mesh, output)
            asset = f"{name}_robot_body"
            ET.SubElement(assets, "mesh", name=asset, file=(final_directory / output.name).relative_to(MODEL).as_posix())
            visual = originals[name]
            visual.attrib.update(name=asset, mesh=asset, mass="0", contype="0", conaffinity="0", rgba="1 1 1 1")
            bodies[name].append(visual)
        # No generated files are replaced unless every split succeeds.
        final_directory.mkdir(parents=True, exist_ok=True)
        for path in Path(staging).iterdir():
            os.replace(path, final_directory / path.name)


def build_legacy():
    source = ET.parse(MODEL / "urdf/full_body_robot.urdf").getroot()
    joints = {joint.get("name"): joint for joint in source.findall("joint")}
    links = {link.get("name"): link for link in source.findall("link")}
    children = defaultdict(list)
    for joint in joints.values():
        children[joint.find("parent").get("link")].append(joint)

    topology = yaml.safe_load((MODEL / "config/motor_topology.yaml").read_text())
    limits = yaml.safe_load(LIMITS.read_text())
    if set(limits["joint_limits"]) != set(joints):
        raise ValueError("Simulation limits must cover every physical joint")
    motors, groups = topology["motors"], topology["differential_groups"]
    controls = {number: [{"name": f"motor_{number:02d}", "joint": motor["joint"]}]
                for number, motor in motors.items() if "joint" in motor}
    assigned = [actuator["joint"] for group in controls.values() for actuator in group]
    assigned.extend(joint for group in groups.values() for joint in group["joints"])
    if len(assigned) != len(joints) or set(assigned) != set(joints):
        raise ValueError("Topology must cover every URDF joint exactly once")

    root = ET.Element("mujoco", model="Full body differential preview")
    root.append(ET.Comment(" Generated by tools/build_mujoco_model.py. Geometry debugging only. "))
    ET.SubElement(root, "compiler", angle="radian", meshdir="..", fusestatic="false")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 0", integrator="implicitfast")
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "text",
                  name="robot_sim_control_mode", data="viewer_position")
    ET.SubElement(root, "statistic", center="0 0 -0.25", extent="1.42")
    ET.SubElement(ET.SubElement(root, "visual"), "global", azimuth="120", elevation="-14")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", type="hinge", damping="0.1", armature="0.01")
    ET.SubElement(defaults, "geom", contype="0", conaffinity="0", density="850")
    ET.SubElement(defaults, "site", size="0.003", group="2")
    ET.SubElement(defaults, "equality", solref="0.01 1", solimp="0.99 0.999 0.001")
    ET.SubElement(defaults, "position", kp="12", kv="1.2", ctrlrange=numbers((-np.pi, np.pi)))

    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 1 1.8", dir="0 -0.45 -1")
    ET.SubElement(world, "light", pos="0 -1 0.8", dir="0 0.55 -1")
    bodies = {"base_link": ET.SubElement(world, "body", name="base_link")}
    frames = {"base_link": (np.zeros(3), Rotation.identity())}

    def add_children(parent):
        add_visual_and_inertia(bodies[parent], links[parent], assets)
        parent_position, parent_rotation = frames[parent]
        for joint in children[parent]:
            child = joint.find("child").get("link")
            position, rotation = pose(joint.find("origin"))
            frames[child] = (parent_position + parent_rotation.apply(position), parent_rotation * rotation)
            body = ET.SubElement(bodies[parent], "body", name=child, **pose_attributes(position, rotation))
            bound = limits["joint_limits"][joint.get("name")]
            lower, upper = bound["lower"], bound["upper"]
            if lower == upper:
                lower, upper = -LOCKED_SOLVER_RANGE, LOCKED_SOLVER_RANGE
            ET.SubElement(body, "joint", name=joint.get("name"), axis=joint.find("axis").get("xyz"),
                          limited="true", range=numbers((lower, upper)))
            bodies[child] = body
            add_children(child)

    add_children("base_link")
    tendons, equalities = (ET.SubElement(root, tag) for tag in ("tendon", "equality"))
    modules = {}
    flanges = {}
    for name, group in groups.items():
        group_controls, modules[name] = add_mechanism(name, group, joints, bodies, frames, assets, tendons, equalities)
        controls[group["motor_numbers"][0]] = group_controls
        for control in group_controls:
            if control["joint"] not in joints:
                side = control["joint"].removeprefix(f"{name}_flange_").removesuffix("_x")
                rod = (f"{name}_{side}_rod" if name in ("waist", "hips")
                       else f"{control['joint']}_rod")
                flanges[control["name"]] = rod
    ET.SubElement(custom, "text", name="robot_sim_joint_position", data=json.dumps({
        "model": MODEL.name,
        "joints": list(joints),
        "flanges": flanges,
        "joint_limits": {name: [bound["lower"], bound["upper"]]
                         for name, bound in limits["joint_limits"].items()},
        "coupled_limits": limits.get("coupled_limits", {}),
    }, separators=(",", ":")))
    actuators = ET.SubElement(root, "actuator")
    for number in sorted(controls):
        for control in controls[number]:
            bound = limits["actuator_limits"][control["name"]]
            lower, upper = bound["lower"], bound["upper"]
            if lower == upper:
                lower, upper = -LOCKED_SOLVER_RANGE, LOCKED_SOLVER_RANGE
            ET.SubElement(actuators, "position", **control, ctrlrange=numbers((lower, upper)))
    add_keyframes(root, world, actuators, modules)
    ET.indent(root, space="  ")
    compiled = deepcopy(root)
    compiled.find("compiler").set("meshdir", str(MODEL))
    model = mujoco.MjModel.from_xml_string(ET.tostring(compiled, encoding="unicode"))
    data = mujoco.MjData(model)
    sys.path.insert(0, str(MODEL.parents[2] / "robot_sim/scripts"))
    from mujoco_joint_position import JointPositionTargets
    position_config = json.loads(custom.find("text[@name='robot_sim_joint_position']").get("data"))
    controller = JointPositionTargets(model, position_config)
    names = list(controller.joints)
    for key in root.findall("keyframe/key")[1:]:
        original = model.key(key.get("name")).qpos
        target = np.array([original[controller.joints[name]] for name in names])
        for group in groups.values():
            indices = [names.index(name) for name in group["joints"]]
            scale = 1.0
            for index in indices:
                lower, upper = controller.limits[names[index]]
                if target[index] < lower:
                    scale = min(scale, lower / target[index])
                if target[index] > upper:
                    scale = min(scale, upper / target[index])
            target[indices] *= scale * 0.9
            # Imported examples must also fit the group's sampled two-axis region.
            for fraction in (1., .5, .25, 0.):
                trial = dict.fromkeys(names, 0.)
                trial.update({names[index]: float(target[index] * fraction) for index in indices})
                try:
                    controller._validate_coupled(trial)
                except ValueError:
                    continue
                target[indices] *= fraction
                break
        controller.set_configuration(data, target)
        key.set("qpos", numbers(data.qpos))
        key.set("ctrl", numbers(data.ctrl))
    ET.indent(root, space="  ")
    compiled = deepcopy(root)
    compiled.find("compiler").set("meshdir", str(MODEL))
    model = mujoco.MjModel.from_xml_string(ET.tostring(compiled, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.nu != len(motors) or model.nv - data.nefc != len(joints):
        raise ValueError("Incorrect number of actuators or independent joint coordinates")
    # Keep the original geometry-scanner input independent of display-only additions.
    (MODEL / "mujoco/full_body_robot.physics.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")
    apply_display_meshes(root)
    ET.indent(root, space="  ")
    pending = OUTPUT.with_suffix(".xml.tmp")
    pending.write_text(ET.tostring(root, encoding="unicode") + "\n")
    pending.replace(OUTPUT)
    write_standalone_urdf(root, joints, limits)
    print(f"{OUTPUT}: {len(links)} URDF links, {model.nu} inputs, {model.neq} closed-loop constraints")


def build():
    """Use native assembly coordinates; the legacy builder is reference only."""
    from build_native_model import build as native_build
    return native_build()


if __name__ == "__main__":
    build()
