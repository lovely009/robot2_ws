#!/usr/bin/python3
"""Scan the assembled CAD surfaces and linkage workspace for simulation limits.

Run with system Python and ROS sourced. Coal uses the system NumPy ABI.
MuJoCo can also be loaded from the existing ros2_control environment.
"""

import argparse
from collections import Counter, defaultdict
import itertools
import hashlib
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
from assembly_fingerprint import geometry_fingerprint
import cad_mating
import scan_artifacts

MODEL = Path(__file__).resolve().parents[1]
PHYSICS_MODEL = MODEL / "mujoco/full_body_robot.physics.xml"
sys.path.insert(0, str(MODEL.parents[2] / "robot_sim/scripts"))
from mujoco_joint_position import JointPositionTargets

STEP = math.radians(1)
MARGIN = math.radians(2)
CAP = math.pi
CLEARANCE = 0.0001
MATING_CLEARANCE = 0.0005
TANGENT_COS_MAX = 0.995
REFINED_JOINTS = tuple(
    f"BN_{side}UpperLeg_{axis}" for side in ("Left", "Right") for axis in (1, 2, 3)
) + tuple(f"BN_{side}Hand_{axis}" for side in ("Left", "Right") for axis in ("roll", "pitch"))
WRIST_FLANGES = tuple(f"{side}_wrist_flange_{part}"
                     for side in ("left", "right") for part in ("proximal", "distal"))
ASSEMBLY_JOINTS = REFINED_JOINTS + WRIST_FLANGES + tuple(name + "_rod_x" for name in WRIST_FLANGES)


def validate_sources():
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    for path, expected in [(MODEL.parent / alignment['source_urdf'], alignment['urdf_sha256']),
                           (MODEL.parent / '全身不带头.STEP', alignment['step_sha256'])]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Source changed since assembly calibration: {path}')
    return alignment


def load_model():
    root = ET.parse(PHYSICS_MODEL if PHYSICS_MODEL.is_file() else MODEL / "mujoco/full_body_robot.xml").getroot()
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


def triangle_shape(points, triangles):
    if not len(triangles):
        return None
    shape = coal.BVHModelOBBRSS()
    shape.beginModel(len(triangles), len(points))
    vector, indices = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
    for point in points:
        vector.append(point)
    for a, b, c in triangles:
        indices.append(coal.Triangle(int(a), int(b), int(c)))
    shape.addSubModel(vector, indices)
    shape.endModel()
    return shape


class SurfaceCollisions:
    """Check every component pair; exempt only proven, bounded mating faces."""

    def __init__(self, model, data, raw_meshes=None, mate_features=None):
        import hashlib
        from scipy.spatial.transform import Rotation
        self.model = model
        self.parts = defaultdict(list)
        self.skipped = []
        self.collision_mesh_hashes = {}
        self.request = coal.CollisionRequest()
        self.request.security_margin = CLEARANCE
        self.request.num_max_contacts = 32
        self.result = coal.CollisionResult()
        self.mate_features = mate_features or ([], [])
        for geom_id in range(model.ngeom):
            if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = model.geom_dataid[geom_id]
            if raw_meshes is not None:
                import vtk
                from vtk.util.numpy_support import vtk_to_numpy
                path = raw_meshes[model.mesh(mesh_id).name]
                self.collision_mesh_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
                reader = vtk.vtkSTLReader(); reader.SetFileName(str(path)); reader.Update()
                poly = reader.GetOutput()
                vertices = vtk_to_numpy(poly.GetPoints().GetData()).astype(float)
                faces = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
                rotation = Rotation.from_quat(model.mesh_quat[mesh_id][[1, 2, 3, 0]]).as_matrix()
                vertices = (vertices - model.mesh_pos[mesh_id]) @ rotation
            else:
                va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
                fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
                vertices = model.mesh_vert[va:va + vn].astype(float)
                faces = model.mesh_face[fa:fa + fn]
            edges = np.concatenate((faces[:, :2], faces[:, 1:], faces[:, [2, 0]]))
            graph = coo_matrix((np.ones(len(edges)), edges.T), shape=(len(vertices), len(vertices))).tocsr()
            count, labels = connected_components(graph, directed=False)
            face_labels = labels[faces[:, 0]]
            for component in range(count):
                selected = faces[face_labels == component]
                ids, inverse = np.unique(selected, return_inverse=True)
                points, triangles = vertices[ids], inverse.reshape(-1, 3)
                tag = f"{model.geom(geom_id).name}#{component}"
                xyz = points[triangles]
                area = np.linalg.norm(np.cross(xyz[:, 1] - xyz[:, 0], xyz[:, 2] - xyz[:, 0]), axis=1)
                valid = area > 0.
                if not valid.all():
                    self.skipped.append({'tag': tag, 'zero_area_triangles': int((~valid).sum()),
                                         'reason': 'Mathematically zero-area triangles only'})
                    triangles = triangles[valid]
                if not len(triangles):
                    continue
                lower, upper = points.min(axis=0), points.max(axis=0)
                self.parts[geom_id].append({'shape': triangle_shape(points, triangles), 'tag': tag,
                    'points': points, 'triangles': triangles,
                    'digest': cad_mating.component_digest(points, triangles),
                    'centre': (lower + upper) / 2, 'half': (upper - lower) / 2})
        self.assembly_exclusions = []
        self.geom_ids = np.array(list(self.parts))
        self.pairs = np.array([(a, b) for a, b in itertools.combinations(self.geom_ids, 2)
                              if model.geom_bodyid[a] != model.geom_bodyid[b]], dtype=int).reshape(-1, 2)
        self.bounds = {geom: (np.array([p['centre'] for p in parts]), np.array([p['half'] for p in parts]))
                       for geom, parts in self.parts.items()}
        self.neutral_rotation, self.neutral_position = self.relative_poses(data)
        self.exclusions, self.exclusion_names = set(), []
        self.mates, self.candidate_neutral_collisions, self.unclassified_collisions = {}, [], []
        self.unclassified_details=[]
        self.neutral_world = {geom: [p['points'] @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
                                    for p in parts] for geom, parts in self.parts.items()}
        self.neutral_frames = {geom: (data.geom_xmat[geom].reshape(3, 3).copy(), data.geom_xpos[geom].copy())
                               for geom in self.parts}
        axes=json.loads((MODEL/'cad/assembly_audit/2026-11-13-new-body-joint-axes.json').read_text())
        verified=[r['joint'] for r in axes if any(c.get('parent') and c.get('child') for c in r['candidates'])] if raw_meshes is not None else []
        self.interior_bound_keys=set()
        self.motor_interiors = cad_mating.motor_interiors(model, data, self.parts, self.neutral_world, self.mate_features[0], verified)
        self.bearing_interiors=cad_mating.bearing_interiors(model,self.parts,self.neutral_world,self.mate_features[2]) if len(self.mate_features)>2 else {}
        self.check(data, baseline=True)

    def relative_poses(self, data):
        a, b = self.pairs.T
        inverse = data.geom_xmat[a].reshape(-1, 3, 3).transpose(0, 2, 1)
        return inverse @ data.geom_xmat[b].reshape(-1, 3, 3), np.einsum('nij,nj->ni', inverse, data.geom_xpos[b] - data.geom_xpos[a])

    def collide(self, first, ta, second, tb):
        if first is None or second is None:
            return False
        self.result.clear()
        coal.collide(first, ta, second, tb, self.request, self.result)
        return self.result.isCollision()

    def active_mates(self, key, rotations, positions):
        a, _, b, _ = key
        active = []
        for i, volume in enumerate(self.mates[key]['volumes']):
            points = [rotations[g] @ volume[f'{tag}_origin'] + positions[g] for g, tag in [(a, 'a'), (b, 'b')]]
            if volume['kind']!='plane' and np.linalg.norm(points[0] - points[1]) > 2 * cad_mating.FIT:
                continue
            if volume['kind'] in ('cylinder','plane'):
                axes = [rotations[g] @ volume[f'{tag}_axis'] for g, tag in [(a, 'a'), (b, 'b')]]
                if abs(np.dot(*axes)) < np.cos(np.deg2rad(.02)):
                    continue
                if volume['kind']=='plane' and abs(np.dot(points[0]-points[1],axes[0]))>2*cad_mating.FIT:
                    continue
            active.append(i)
        return tuple(active)

    def pair_hit(self, key, transforms, active):
        a, ia, b, ib = key
        first, second = self.parts[a][ia], self.parts[b][ib]
        self.pending_contacts = None
        budget = 32
        try:
            while True:
                self.request.num_max_contacts = budget
                hit = self.collide(first['shape'], transforms[a], second['shape'], transforms[b])
                if not hit or not active:
                    return hit
                contacts = np.array([contact.pos for contact in self.result.getContacts()])
                if not len(contacts):return True
                allowed = np.zeros(len(contacts), dtype=bool)
                # Evaluate volumes in the current world frame, not their neutral placement.
                rotation = transforms[a].getRotation(); position = transforms[a].getTranslation()
                for i in active:
                    original = self.mates[key]['volumes'][i]
                    volume = dict(original)
                    volume['origin'] = rotation @ original['a_origin'] + position
                    if original['kind'] in ('cylinder','plane'):
                        volume['axis'] = rotation @ original['a_axis']
                    if original['kind']=='plane':
                        volume['u']=rotation@original['a_u'];volume['v']=rotation@original['a_v']
                        volume['patch_a']=original['a_patch']@rotation.T+position
                        rotation_b=transforms[b].getRotation();position_b=transforms[b].getTranslation()
                        volume['patch_b']=original['b_patch']@rotation_b.T+position_b
                        volume['origin_b']=rotation_b@original['b_origin']+position_b
                        volume['axis_b']=rotation_b@original['b_axis']
                    allowed |= cad_mating.inside(contacts, volume)
                if not allowed.all():
                    self.pending_contacts = contacts[~allowed]
                    return True
                if len(contacts) < budget:
                    return False
                if budget >= 1048576:
                    # Never accept a truncated contact list as proof of clearance.
                    self.pending_contacts = contacts
                    return True
                budget *= 4
        finally:
            self.request.num_max_contacts = 32

    def bind_volumes(self,key,volumes):
        a,_,b,_=key
        entry=self.mates.setdefault(key,{'volumes':[]})
        for original in volumes:
            volume=dict(original)
            for geom,tag in [(a,'a'),(b,'b')]:
                rotation,position=self.neutral_frames[geom]
                volume[f'{tag}_origin']=rotation.T@(volume['origin']-position)
                if volume['kind'] in ('cylinder','plane'):volume[f'{tag}_axis']=rotation.T@volume['axis']
                if volume['kind']=='plane':
                    volume[f'{tag}_u']=rotation.T@volume['u'];volume[f'{tag}_v']=rotation.T@volume['v']
                    volume[f'{tag}_patch']=(volume['patch_'+tag]-position)@rotation
            entry['volumes'].append(volume)

    def bind_interior_pair(self,key):
        a,_,b,_=key
        pair=frozenset((int(self.model.geom_bodyid[a]),int(self.model.geom_bodyid[b])))
        volumes=self.motor_interiors.get(pair,[])+self.bearing_interiors.get(pair,[])
        if volumes:self.bind_volumes(key,volumes)
        self.interior_bound_keys.add(key)

    def interior_catalog(self):
        return [{'bodies':[self.model.body(i).name for i in sorted(pair)],
                 'volumes':[{key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in volume.items()}
                            for volume in volumes]}
                for source in (self.motor_interiors,self.bearing_interiors) for pair,volumes in source.items()]

    def classify(self, key, transforms):
        a, ia, b, ib = key
        self.mates[key] = {'volumes': []}
        seen = set()
        for _ in range(6):
            contacts = self.pending_contacts if self.pending_contacts is not None else np.array([c.pos for c in self.result.getContacts()])
            if not len(contacts):break
            previous = len(self.mates[key]['volumes'])
            found = cad_mating.shared_surfaces(self.neutral_world[a][ia], self.neutral_world[b][ib], contacts, *self.mate_features, triangles=(self.parts[a][ia]['triangles'],self.parts[b][ib]['triangles']))
            pair=frozenset((int(self.model.geom_bodyid[a]),int(self.model.geom_bodyid[b])))
            found += [dict(v) for v in self.motor_interiors.get(pair,[])+self.bearing_interiors.get(pair,[]) if cad_mating.inside(contacts,v).any()]
            for volume in found:
                identity = (volume['kind'], tuple(np.round(volume['origin'], 7)), round(volume.get('radius',volume.get('thickness',0)), 7), round(volume.get('half_length', 0.), 7))
                if identity in seen:
                    continue
                seen.add(identity)
                self.bind_volumes(key,[volume])
            active = tuple(range(len(self.mates[key]['volumes'])))
            if active and not self.pair_hit(key, transforms, active):
                return True
            if len(self.mates[key]['volumes']) == previous:
                break
        self.last_unclassified_details={'components':[self.parts[a][ia]['tag'],self.parts[b][ib]['tag']],
            'remaining_contacts':contacts[:16].tolist(), 'features_tried':[v['evidence'] for v in self.mates[key]['volumes']]}
        del self.mates[key]
        return False

    def check(self, data, baseline=False):
        rotations, positions = data.geom_xmat.reshape(-1, 3, 3), data.geom_xpos
        boxes, transforms = {}, {}
        for geom, (centre, half) in self.bounds.items():
            world = centre @ rotations[geom].T + positions[geom]
            extent = half @ np.abs(rotations[geom]).T
            boxes[geom] = world - extent - CLEARANCE, world + extent + CLEARANCE
            transforms[geom] = coal.Transform3s(rotations[geom], positions[geom])
        relative_rotation, relative_position = self.relative_poses(data)
        changed = ((np.max(abs(relative_rotation - self.neutral_rotation), axis=(1, 2)) > 1e-8)
                   | (np.max(abs(relative_position - self.neutral_position), axis=1) > 1e-8))
        for pair_index, (a, b) in enumerate(self.pairs):
            if not baseline and not changed[pair_index] and not self.unclassified_collisions:
                continue
            lo_a, hi_a = boxes[a]; lo_b, hi_b = boxes[b]
            overlap = np.all(hi_a[:, None] >= lo_b, axis=2) & np.all(hi_b >= lo_a[:, None], axis=2)
            for ia, ib in np.argwhere(overlap):
                key = (int(a), int(ia), int(b), int(ib))
                active = self.active_mates(key, rotations, positions) if key in self.mates else ()
                if not self.pair_hit(key, transforms, active):
                    continue
                if not baseline and key not in self.interior_bound_keys:
                    # Geometry-certified interiors apply to newly encountered pairs too.
                    # Neutral contact itself never grants a new allowance.
                    self.bind_interior_pair(key)
                    active=self.active_mates(key,rotations,positions) if key in self.mates else ()
                    if active and not self.pair_hit(key,transforms,active):continue
                names = [self.parts[a][ia]['tag'], self.parts[b][ib]['tag']]
                if not baseline:
                    return ' / '.join(names)
                self.candidate_neutral_collisions.append(names)
                if not self.classify(key, transforms):
                    self.unclassified_collisions.append(names)
                    self.unclassified_details.append(self.last_unclassified_details)
                if len(self.candidate_neutral_collisions) % 25 == 0:
                    print('neutral contacts', len(self.candidate_neutral_collisions),
                          'classified', len(self.mates), 'unclassified', len(self.unclassified_collisions), flush=True)
        return None

    def exemption_records(self):
        result = []
        for (a, ia, b, ib), entry in self.mates.items():
            result.append({'components': [self.parts[a][ia]['tag'], self.parts[b][ib]['tag']],
                'component_digests': [self.parts[a][ia]['digest'], self.parts[b][ib]['digest']],
                'surfaces': [{key: value.tolist() if isinstance(value, np.ndarray) else value
                              for key, value in v.items() if not key.startswith(('a_', 'b_')) and key not in ('patch_a','patch_b')} for v in entry['volumes']]})
        return result


class Workspace:
    def __init__(self):
        self.source_evidence=validate_sources()
        self.policy_evidence=scan_artifacts.policy_hashes(MODEL)
        self.geometry_evidence=source_fingerprint()
        self.model, self.controller = load_model()
        self.data = mujoco.MjData(self.model)
        self.names = list(self.controller.joints)
        self.controller.limits = dict.fromkeys(self.names, (-CAP, CAP))
        self.pose(np.zeros(len(self.names)))
        # These selected motor/bearing/ball-joint assemblies allow internal CAD overlap.
        root = ET.parse(PHYSICS_MODEL).getroot()
        raw_meshes = {mesh.get('name'): MODEL / mesh.get('file').replace('visual/native/', 'collision/native/')
                      for mesh in root.findall('asset/mesh')}
        self.collisions = SurfaceCollisions(self.model, self.data, raw_meshes=raw_meshes,
                                           mate_features=cad_mating.features(MODEL, self.model, self.data))
        diagnostics = {'candidate_neutral_collisions': self.collisions.candidate_neutral_collisions,
                       'accepted_exemptions': self.collisions.exemption_records(),
                       'unclassified_collisions': self.collisions.unclassified_collisions,
                       'unclassified_details':self.collisions.unclassified_details}
        Path('/tmp/full-body-neutral-mates.json').write_text(json.dumps(diagnostics, indent=2) + '\n')
        if self.collisions.unclassified_collisions:
            raise ValueError(f"Unclassified neutral CAD contacts: {len(self.collisions.unclassified_collisions)}; see /tmp/full-body-neutral-mates.json")

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
    root = ET.parse(PHYSICS_MODEL if PHYSICS_MODEL.is_file() else MODEL / "mujoco/full_body_robot.xml").getroot()
    return geometry_fingerprint(MODEL, root)


def save_result(workspace, report, group_names=None):
    if (scan_artifacts.policy_hashes(MODEL)!=workspace.policy_evidence
            or source_fingerprint()!=workspace.geometry_evidence
            or validate_sources()!=workspace.source_evidence):
        raise ValueError('Scan inputs changed during execution; discard this generation')
    bounds = {name: list(limits) for name, limits in report["joint_limits"].items()}
    groups, actuators = coupled_ranges(workspace, bounds, group_names)
    alignment=validate_sources()
    for filename, expected in workspace.collisions.collision_mesh_hashes.items():
        if hashlib.sha256((MODEL/'mujoco/collision/native'/filename).read_bytes()).hexdigest()!=expected:
            raise ValueError('Collider changed during scanning: '+filename)
    report.update(
        geometry_fingerprint=source_fingerprint(), source_urdf_sha256=alignment['urdf_sha256'],
        source_step_sha256=alignment['step_sha256'],
        scan_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        mating_script_sha256=hashlib.sha256(Path(cad_mating.__file__).read_bytes()).hexdigest(),
        policy_inputs_sha256=scan_artifacts.policy_hashes(MODEL),
        scope='simulation_only', scan_mode='all_37_single_axis + 6_group_2d',
        hardware_validation='not_calibrated',
        collision_mesh_sha256=workspace.collisions.collision_mesh_hashes,
        exemption_policy='STEP-supported bounded mating faces; shared centers/axes checked at every pose',
        candidate_neutral_collisions=workspace.collisions.candidate_neutral_collisions,
        accepted_exemptions=workspace.collisions.exemption_records(),
        finite_interior_catalog=workspace.collisions.interior_catalog(),
        interior_pair_policy="Certified finite motor/bearing volumes also apply to new component contacts; both current centers/axes and every contact must pass",
        unclassified_collisions=workspace.collisions.unclassified_collisions, joint_limits=bounds,
        differential_groups=report.get("differential_groups", {}) | groups,
        actuator_limits=report.get("actuator_limits", {}) | actuators,
        differential_actuator_margin_deg=1,
        neutral_mating_clearance_m=MATING_CLEARANCE,
        adjacent_assembly_exclusions=workspace.collisions.assembly_exclusions,
        motor_scan_range_deg=[-180, 180],
        motor_scan_range_source="User-confirmed output-shaft simulation search range; physical per-motor travel is uncalibrated.",
        limitations=[
            "Bounds scan one joint or one differential group with other output joints at zero.",
            "Only STEP-supported bounded mating faces are masked; neutral contact alone grants no exemption.",
            "No complete body pair is excluded; exact unsimplified STL components are checked.",
            "Triangle surfaces and one-degree sampling do not certify solid containment or all combined poses.",
            "Scan caps at 180 degrees are search bounds, not measured mechanical end stops.",
            "The CAD source has no calibrated physical limits, encoder signs or cable restrictions.",
        ],
    )
    if len(bounds)!=37 or len(report['actuator_limits'])!=37 or workspace.collisions.unclassified_collisions:
        raise ValueError('Incomplete or unclassified geometry limits cannot be installed')
    from robot_transmission.coupled_limits import CoupledLimits
    CoupledLimits({name:g['coupled_workspace'] for name,g in report['differential_groups'].items()}, bounds).validate(dict.fromkeys(bounds,0.))
    output = MODEL / "mujoco/geometry_limits.json"
    temporary=output.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(output)
    config = {
        "source": "mujoco/geometry_limits.json",
        "scan_report_sha256":hashlib.sha256(output.read_bytes()).hexdigest(),
        "source_sha256": hashlib.sha256((MODEL/'urdf/full_body_robot.urdf').read_bytes()).hexdigest(),
        "geometry_sha256": report["geometry_fingerprint"],
        "source_urdf_sha256": report["source_urdf_sha256"],
        "source_step_sha256": report["source_step_sha256"],
        "collision_validation": "sampled: exact CAD surfaces at 1 degree with 2 degree margin; other groups neutral; hardware and combined poses uncalibrated",
        "preview_limit_policy": "Asymmetric CAD scan intervals with neutral-connected coupled masks; search caps are not measured hard stops",
        "joint_limits": {name: {"lower": bounds[0], "upper": bounds[1]} for name, bounds in bounds.items()},
        "actuator_limits": {name: {"lower": limits[0], "upper": limits[1]}
                            for name, limits in report["actuator_limits"].items()},
        "coupled_limits": {name: group["coupled_workspace"] for name, group in report["differential_groups"].items()
                           if "coupled_workspace" in group},
    }
    destination=MODEL/"config/simulation_limits.yaml"
    temporary=destination.with_suffix(".yaml.tmp")
    temporary.write_text(yaml.safe_dump(config, sort_keys=False))
    temporary.replace(destination)
    print(output, flush=True)


def derive(refine=False, coupled=False):
    started = time.monotonic()
    workspace = Workspace()
    print(f"CAD components {sum(map(len, workspace.collisions.parts.values()))}, "
          f"neutral mating exclusions {len(workspace.collisions.exclusions)}", flush=True)
    report = {}
    if refine or coupled:
        report = json.loads((MODEL / "mujoco/geometry_limits.json").read_text())
        if report["geometry_fingerprint"] != source_fingerprint():
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
        "method": "Exact unsimplified STL triangle surfaces; STEP-supported bounded mating faces",
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
