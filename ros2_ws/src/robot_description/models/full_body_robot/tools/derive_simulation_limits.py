#!/usr/bin/python3
"""Scan the assembled CAD surfaces and linkage workspace for simulation limits.

Run with system Python and ROS sourced. Coal uses the system NumPy ABI.
MuJoCo can also be loaded from the existing ros2_control environment.
"""

import argparse
from collections import Counter, defaultdict
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import coal
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.ndimage import binary_erosion, label

try:
    import mujoco
except ModuleNotFoundError:
    sys.path.append(str(Path.home() / ".ros/ros2_control/.venv/lib" /
                        f"python{sys.version_info.major}.{sys.version_info.minor}/site-packages"))
    import mujoco

import yaml

MODEL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODEL.parents[2] / "robot_sim/scripts"))
from mujoco_joint_position import JointPositionTargets

STEP = math.radians(1)
MARGIN = math.radians(2)
CAP = math.pi
CLEARANCE = 0.0001
MATING_CLEARANCE = 0.0005
TANGENT_COS_MAX = 0.995
REFINED_JOINTS = tuple(
    f"BN_{side}{part}_axis_{axis}"
    for part, axes in (("UpperLeg", (1, 2, 3)), ("Hand", (1, 2)))
    for side in ("Left", "Right") for axis in axes
)
WRIST_FLANGES = tuple(f"{side}_wrist_flange_{part}"
                     for side in ("left", "right") for part in ("proximal", "distal"))
ASSEMBLY_JOINTS = REFINED_JOINTS + WRIST_FLANGES + tuple(name + "_rod_x" for name in WRIST_FLANGES)


def load_model():
    root = ET.parse(MODEL / "mujoco/full_body_robot.xml").getroot()
    root.find("compiler").set("meshdir", str(MODEL))
    for joint in root.findall(".//joint"):
        joint.set("limited", "false")
        joint.attrib.pop("range", None)
    root.find("default/position").set("ctrlrange", f"{-CAP} {CAP}")
    for actuator in root.findall("actuator/position"):
        actuator.set("ctrlrange", f"{-CAP} {CAP}")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    config = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get("data"))
    config["differential_limits"] = {
        joint: CAP for group in yaml.safe_load((MODEL / "config/motor_topology.yaml").read_text())[
            "differential_groups"].values() for joint in group["joints"]
    }
    config.pop("joint_limits", None)
    config.pop("coupled_limits", None)
    return model, JointPositionTargets(model, config)


class SurfaceCollisions:
    """Keep CAD components separate so a mating bearing cannot mask a whole link."""

    def __init__(self, model, data, assembly_joints=()):
        self.model = model
        self.parts = defaultdict(list)
        self.skipped = []
        self.request = coal.CollisionRequest()
        self.request.security_margin = CLEARANCE
        self.result = coal.CollisionResult()
        for geom_id in range(model.ngeom):
            if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = model.geom_dataid[geom_id]
            va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
            vertices = model.mesh_vert[va:va + vn].astype(float)
            faces = model.mesh_face[fa:fa + fn]
            edges = np.concatenate((faces[:, :2], faces[:, 1:], faces[:, [2, 0]]))
            graph = coo_matrix((np.ones(len(edges)), edges.T), shape=(vn, vn)).tocsr()
            count, labels = connected_components(graph, directed=False)
            face_labels = labels[faces[:, 0]]
            for component in range(count):
                selected = faces[face_labels == component]
                ids, inverse = np.unique(selected, return_inverse=True)
                points = vertices[ids]
                tag = f"{model.geom(geom_id).name}#{component}"
                if len(selected) < 4 or np.max(np.ptp(points, axis=0)) < 0.0002:
                    self.skipped.append(tag)
                    continue
                triangles = inverse.reshape(-1, 3)
                geometry = coal.BVHModelOBBRSS()
                geometry.beginModel(len(triangles), len(points))
                vector = coal.StdVec_Vec3s()
                indices = coal.StdVec_Triangle()
                for point in points:
                    vector.append(point)
                for a, b, c in triangles:
                    indices.append(coal.Triangle(int(a), int(b), int(c)))
                geometry.addSubModel(vector, indices)
                geometry.endModel()
                lower, upper = points.min(axis=0), points.max(axis=0)
                self.parts[geom_id].append({
                    "shape": geometry, "tag": tag,
                    "centre": (lower + upper) / 2, "half": (upper - lower) / 2,
                })
        assembly_pairs = set()
        self.assembly_exclusions = []
        for name in assembly_joints:
            child = int(model.joint(name).bodyid[0])
            parent = int(model.body_parentid[child])
            assembly_pairs.add(frozenset((parent, child)))
            self.assembly_exclusions.append([model.body(parent).name, model.body(child).name])
        self.geom_ids = np.array(list(self.parts))
        self.pairs = np.array([
            (a, b) for a, b in itertools.combinations(self.geom_ids, 2)
            if model.geom_bodyid[a] != model.geom_bodyid[b]
            and frozenset((int(model.geom_bodyid[a]), int(model.geom_bodyid[b]))) not in assembly_pairs
        ], dtype=int).reshape(-1, 2)
        self.bounds = {}
        for geom, parts in self.parts.items():
            self.bounds[geom] = (
                np.array([part["centre"] for part in parts]),
                np.array([part["half"] for part in parts]),
            )
        self.neutral_rotation, self.neutral_position = self.relative_poses(data)
        self.exclusions = set()
        self.exclusion_names = []
        self.check(data, baseline=True)

    def relative_poses(self, data):
        a, b = self.pairs.T
        inverse = data.geom_xmat[a].reshape(-1, 3, 3).transpose(0, 2, 1)
        return (inverse @ data.geom_xmat[b].reshape(-1, 3, 3),
                np.einsum("nij,nj->ni", inverse, data.geom_xpos[b] - data.geom_xpos[a]))

    def check(self, data, baseline=False):
        clearance = MATING_CLEARANCE if baseline else CLEARANCE
        self.request.security_margin = clearance
        rotations = data.geom_xmat.reshape(-1, 3, 3)
        boxes = {}
        transforms = {}
        for geom, (centre, half) in self.bounds.items():
            position, rotation = data.geom_xpos[geom], rotations[geom]
            world_centre = centre @ rotation.T + position
            extent = half @ np.abs(rotation).T
            boxes[geom] = (world_centre - extent - clearance, world_centre + extent + clearance)
            transforms[geom] = coal.Transform3s(rotation, position)
        relative_rotation, relative_position = self.relative_poses(data)
        changed = (np.max(np.abs(relative_rotation - self.neutral_rotation), axis=(1, 2)) > 1e-8
                   ) | (np.max(np.abs(relative_position - self.neutral_position), axis=1) > 1e-8)
        for pair_index, (a, b) in enumerate(self.pairs):
            if not baseline and not changed[pair_index]:
                continue
            lo_a, hi_a = boxes[a]
            lo_b, hi_b = boxes[b]
            overlap = np.all(hi_a[:, None] >= lo_b, axis=2) & np.all(hi_b >= lo_a[:, None], axis=2)
            for ia, ib in np.argwhere(overlap):
                key = (int(a), int(ia), int(b), int(ib))
                if key in self.exclusions:
                    continue
                self.result.clear()
                first, second = self.parts[a][ia], self.parts[b][ib]
                coal.collide(first["shape"], transforms[a], second["shape"], transforms[b],
                             self.request, self.result)
                if self.result.isCollision():
                    if not baseline:
                        return f"{first['tag']} / {second['tag']}"
                    self.exclusions.add(key)
                    self.exclusion_names.append([first["tag"], second["tag"]])
        return None


class Workspace:
    def __init__(self):
        self.model, self.controller = load_model()
        self.data = mujoco.MjData(self.model)
        self.names = list(self.controller.joints)
        self.controller.limits = dict.fromkeys(self.names, (-CAP, CAP))
        self.pose(np.zeros(len(self.names)))
        # These selected motor/bearing/ball-joint assemblies allow internal CAD overlap.
        self.collisions = SurfaceCollisions(self.model, self.data, ASSEMBLY_JOINTS)

    def pose(self, positions):
        self.controller.set_configuration(self.data, positions)

    def valid(self, positions):
        try:
            self.pose(positions)
        except ValueError as error:
            return str(error)
        for joint, ball, platform, length in self.controller.rods.values():
            pose = self.controller.pose
            axis = pose.xaxis[joint]
            radius = pose.site_xpos[ball] - pose.xanchor[joint]
            radial = radius - axis * np.dot(axis, radius)
            offset = pose.site_xpos[platform] - pose.xanchor[joint] - (radius - radial)
            amplitude = np.linalg.norm(radial) * np.linalg.norm(offset - axis * np.dot(axis, offset))
            cosine = (np.dot(offset, offset) + np.dot(radial, radial) - length**2) / (2 * amplitude)
            if abs(cosine) >= TANGENT_COS_MAX:
                return f"linkage tangency at {self.model.joint(joint).name}"
        return self.collisions.check(self.data)


def neutral_component(mask, first, second):
    """Keep poses connected to neutral by changing one joint at a time."""
    origin_row = int(np.flatnonzero(first == 0)[0])
    origin_col = int(np.flatnonzero(second == 0)[0])
    components, _ = label(mask)
    component = components[origin_row, origin_col]
    if not component:
        raise ValueError("The neutral pose has no workspace with the required margin")
    return components == component


def coupled_ranges(workspace, bounds, group_names=None):
    topology = yaml.safe_load((MODEL / "config/motor_topology.yaml").read_text())
    groups = {}
    actuators = {}
    grouped = {name for group in topology["differential_groups"].values() for name in group["joints"]}
    for number, motor in topology["motors"].items():
        if "joint" in motor and motor["joint"] not in grouped:
            actuators[workspace.model.actuator(number - 1).name] = bounds[motor["joint"]]
    for name, group in topology["differential_groups"].items():
        if group_names is not None and name not in group_names:
            continue
        joints = group["joints"]
        first, second = (np.arange(round(math.degrees(bounds[joint][0])),
                                   round(math.degrees(bounds[joint][1])) + 1) for joint in joints)
        mask = np.zeros((len(first), len(second)), dtype=bool)
        inputs = np.full((len(first), len(second), 2), np.nan)
        reasons = Counter()
        slots = np.array(group["motor_numbers"]) - 1
        for row, a in enumerate(first):
            for col, b in enumerate(second):
                q = np.zeros(len(workspace.names))
                q[[workspace.names.index(joint) for joint in joints]] = np.deg2rad([a, b])
                reason = workspace.valid(q)
                mask[row, col] = reason is None
                if reason:
                    reasons[reason] += 1
                else:
                    inputs[row, col] = workspace.data.ctrl[slots]
        interior = binary_erosion(mask, structure=np.ones((5, 5)), border_value=1)
        selected_mask = neutral_component(interior, first, second)
        row, col = np.nonzero(selected_mask)
        low, high, left, right = int(row.min()), int(row.max()), int(col.min()), int(col.max())
        coupled_workspace = {
            "joints": joints,
            "lower": np.deg2rad([first[0], second[0]]).tolist(),
            "step": STEP,
            "valid_rows": ["".join("1" if value else "0" for value in row) for row in selected_mask],
        }
        selected = inputs[selected_mask]
        if not np.isfinite(selected).all():
            raise ValueError(f"{name}: non-finite motor input in the selected workspace")
        rectangle = [[int(first[low]), int(first[high])], [int(second[left]), int(second[right])]]
        for joint, interval in zip(joints, rectangle):
            bounds[joint] = np.deg2rad(interval).tolist()
        for index, slot in enumerate(slots):
            actuators[workspace.model.actuator(int(slot)).name] = [
                float(max(-CAP, selected[:, index].min() - STEP)),
                float(min(CAP, selected[:, index].max() + STEP))]
        groups[name] = {
            "joints": joints, "grid_points": int(mask.size),
            "valid_points": int(mask.sum()), "envelope_deg": rectangle,
            "invalid_reasons": dict(reasons),
            "coupled_workspace": coupled_workspace,
        }
        print("coupled", name, rectangle, f"{mask.sum()}/{mask.size} valid", flush=True)
    return groups, actuators


def source_fingerprint():
    root = ET.parse(MODEL / "mujoco/full_body_robot.xml").getroot()
    files = sorted({asset.get("file") for asset in root.findall("asset/mesh")})
    hashes = {name: hashlib.sha256((MODEL / name).read_bytes()).hexdigest() for name in files}
    for tag in ("custom", "actuator", "keyframe"):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    for joint in root.findall(".//joint"):
        joint.attrib.pop("limited", None)
        joint.attrib.pop("range", None)
    root.find("default").remove(root.find("default/position"))
    hashes["kinematics"] = hashlib.sha256(ET.tostring(root)).hexdigest()
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()


def save_result(workspace, report, group_names=None):
    bounds = {name: list(limits) for name, limits in report["joint_limits"].items()}
    groups, actuators = coupled_ranges(workspace, bounds, group_names)
    report.update(
        source_sha256=source_fingerprint(), joint_limits=bounds,
        differential_groups=report.get("differential_groups", {}) | groups,
        actuator_limits=report.get("actuator_limits", {}) | actuators,
        differential_actuator_margin_deg=1,
        neutral_mating_clearance_m=MATING_CLEARANCE,
        adjacent_assembly_exclusions=workspace.collisions.assembly_exclusions,
        motor_scan_range_deg=[-180, 180],
        motor_scan_range_source="User-confirmed output-shaft simulation search range; physical per-motor travel is uncalibrated.",
        limitations=[
            "Bounds scan one joint or one differential group with other output joints at zero.",
            "Zero-pose component pairs within 0.5 mm are treated as mating parts and excluded.",
            "Listed adjacent thigh/wrist assemblies allow internal CAD overlap without a depth bound.",
            "Triangle surfaces and one-degree sampling do not certify solid containment or all combined poses.",
            "Scan caps at 180 degrees are search bounds, not measured mechanical end stops.",
            "The CAD source has no calibrated physical limits, encoder signs or cable restrictions.",
        ],
    )
    output = MODEL / "mujoco/geometry_limits.json"
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    config = {
        "source": "mujoco/geometry_limits.json",
        "source_sha256": report["source_sha256"],
        "joint_limits": {name: {"lower": bounds[0], "upper": bounds[1]} for name, bounds in bounds.items()},
        "actuator_limits": {name: {"lower": limits[0], "upper": limits[1]}
                            for name, limits in report["actuator_limits"].items()},
        "coupled_limits": {name: group["coupled_workspace"] for name, group in report["differential_groups"].items()
                           if "coupled_workspace" in group},
    }
    (MODEL / "config/simulation_limits.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    print(output, flush=True)


def derive(refine=False, coupled=False):
    started = time.monotonic()
    workspace = Workspace()
    print(f"CAD components {sum(map(len, workspace.collisions.parts.values()))}, "
          f"neutral mating exclusions {len(workspace.collisions.exclusions)}", flush=True)
    report = {}
    if refine or coupled:
        report = json.loads((MODEL / "mujoco/geometry_limits.json").read_text())
        if report["source_sha256"] != source_fingerprint():
            raise ValueError("Geometry changed; rescan all joints before refining selected ranges")
    bounds, reasons = report.get("joint_limits", {}), report.get("boundary_reasons", {})
    selected = REFINED_JOINTS if refine else workspace.names
    if coupled:
        topology = yaml.safe_load((MODEL / "config/motor_topology.yaml").read_text())
        selected = [name for group in topology["differential_groups"].values() for name in group["joints"]]
    for name in selected:
        index = workspace.names.index(name)
        limits, hits = [], []
        for sign in (-1, 1):
            last, hit = CAP, "scan_cap_180_deg"
            for angle in np.arange(STEP, CAP + STEP / 2, STEP):
                q = np.zeros(len(workspace.names))
                q[index] = sign * angle
                obstacle = workspace.valid(q)
                if obstacle:
                    last, hit = angle - STEP, obstacle
                    break
            limits.append(sign * max(0, last - MARGIN))
            hits.append(hit)
        bounds[name], reasons[name] = [round(float(v), 12) for v in limits], hits
        print(name, np.rad2deg(limits).round(2).tolist(), hits, flush=True)
    report.update({
        "method": "CAD triangle surfaces, disconnected components, neutral mating exclusions",
        "step_deg": 1, "margin_deg": 2, "cap_deg": 180,
        "clearance_m": CLEARANCE, "tangent_cos_max": TANGENT_COS_MAX,
        "neutral_mating_clearance_m": MATING_CLEARANCE,
        "joint_limits": {name: bounds[name] for name in workspace.names},
        "boundary_reasons": reasons,
        "neutral_exclusions": workspace.collisions.exclusion_names,
        "skipped_degenerate_components": workspace.collisions.skipped,
        "elapsed_seconds": round(time.monotonic() - started, 2),
        "revised_joint_names": list(selected),
    })
    path = Path("/tmp/full-body-geometry-limits-initial.json")
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(path, flush=True)
    save_result(workspace, report, ("left_wrist", "right_wrist") if refine else None)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--refine-thigh-wrist", action="store_true",
                        help="Rescan the ten thigh/wrist outputs while preserving other computed limits")
    selection.add_argument("--refine-coupled", action="store_true",
                           help="Rescan all six differential groups while preserving direct-joint limits")
    options = parser.parse_args()
    derive(options.refine_thigh_wrist, options.refine_coupled)
