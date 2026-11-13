"""Bound mating surfaces by shared analytic STEP features, never whole bodies."""
import hashlib
import json
import gzip
from pathlib import Path

import numpy as np

FIT = .000025
PADDING = .00011


def features(directory, model, data):
    source = directory / 'cad/assembly_audit/step-analytic-features.json'
    records = json.loads(source.read_text())
    alignment = json.loads((directory / 'config/step_alignment.json').read_text())
    if records['step_sha256'] != alignment['step_sha256']:
        raise ValueError('Mating features belong to another STEP assembly')
    cylinders = [(np.asarray(c['origin_m']), np.asarray(c['axis']), c['radius_m'], f'STEP cylinder {i}')
                 for i, c in enumerate(records['cylinders'])]
    spheres = [(np.asarray(s['center_m']), s['radius_m'], f'STEP sphere {i}')
               for i, s in enumerate(records['spheres'])]
    mounting = json.loads((directory / 'config/arm_mount_alignment.json').read_text())
    delta = np.asarray(mounting['world_transform'])
    cylinders += [(delta[:3, :3] @ o + delta[:3, 3], delta[:3, :3] @ a, r, tag + ' after arm mounting')
                  for o, a, r, tag in cylinders[:]]
    spheres += [(delta[:3, :3] @ o + delta[:3, 3], r, tag + ' after arm mounting')
                for o, r, tag in spheres[:]]
    rods = json.loads((directory / 'cad/assembly_audit/2026-11-13-step-linkages.json').read_text())
    assembly = json.loads((directory / 'config/native_assembly.json').read_text())
    bearings = []
    for name, rod in rods['rods'].items():
        prefix = name.removesuffix('_rod')
        pose = np.asarray(assembly['coupled_visual_world_transforms'][name + '_visual'])
        original = np.asarray(alignment['links'][rod['source_body']]['world_transform'])
        installed = pose @ np.linalg.inv(original)
        for end, site in [('flange', '_motor_ball'), ('platform', '_platform_ball')]:
            endpoint = rod['endpoints'][end]
            center = data.site(prefix + site).xpos.copy()
            spheres.append((center, endpoint['ball_radius_m'],
                            f'Verified installed {name}/{end} ball'))
            eye = endpoint['rod_end_cylinder']
            bearings.append({'origin': center, 'ball_radius': endpoint['ball_radius_m'],
                'eye_radius': eye['radius_m'], 'axis': installed[:3, :3] @ eye['axis'],
                'half_length': eye['axial_span_m'] / 2,
                'body_pair': [int(model.body(name).id),
                              int(model.site_bodyid[model.site(prefix+site).id])],
                'evidence': f'Linkage audit {name}/{end}: spherical stud and cylindrical bearing receiver'})
    raw = json.loads(gzip.decompress((directory / 'cad/assembly_audit/step-planes.json.gz').read_bytes()))
    if raw['step_sha256'] != alignment['step_sha256']:
        raise ValueError('Mating planes belong to another STEP assembly')
    planes = [(np.asarray(p['normal']), p['offset_m'], f'STEP plane {i}') for i, p in enumerate(raw['planes'])]
    planes += [(delta[:3, :3] @ n, float((delta[:3, :3] @ n) @ (delta[:3, :3] @ (n*d) + delta[:3, 3])), tag+' after arm mounting')
               for n, d, tag in planes[:]]
    normals = np.asarray([p[0] for p in planes])
    selected = np.max(abs(normals @ data.xaxis.T), axis=1) > np.cos(np.deg2rad(.02))
    planes = [p for p, include in zip(planes, selected) if include]
    print('STEP mating planes parallel to actual joint axes', len(planes), flush=True)
    return cylinders, spheres, bearings, planes


def shared_surfaces(first, second, contacts, cylinders, spheres, bearings=(), planes=(), triangles=(None, None)):
    """Return finite mate volumes supported on both exact component meshes."""
    volumes = []
    seen = set()
    if planes and triangles[0] is not None:
        normals = np.asarray([p[0] for p in planes]); offsets = np.asarray([p[1] for p in planes])
        close = np.min(abs(contacts[:8] @ normals.T - offsets), axis=0) < PADDING
        for (normal, distance, evidence), near in zip(planes, close):
            if not near:
                continue
            selected = [];patches=[];patch_ids=[]
            for points, faces in zip((first, second), triangles):
                mask = abs(points @ normal - distance) < FIT
                ids=np.flatnonzero(mask[faces].all(axis=1))
                if not len(ids):break
                patch=points[faces[ids]]
                selected.append(patch.reshape(-1,3));patches.append(patch);patch_ids.append(ids.tolist())
            if len(selected) != 2:
                continue
            u = np.cross(normal, [1., 0., 0.] if abs(normal[0]) < .8 else [0., 1., 0.]); u /= np.linalg.norm(u)
            v = np.cross(normal, u)
            lows=[np.min(p@np.column_stack((u,v)),axis=0) for p in selected]
            highs=[np.max(p@np.column_stack((u,v)),axis=0) for p in selected]
            if np.any(np.minimum(*highs)-np.maximum(*lows)<-PADDING):continue
            low,high=lows[0],highs[0]
            center = normal*distance + np.column_stack((u, v)) @ ((low+high)/2)
            identity = ('plane', *np.round(normal, 6), *np.round(center, 7), *np.round(high-low, 7))
            if identity in seen:
                continue
            seen.add(identity)
            volumes.append({'kind':'plane', 'origin':center, 'axis':normal, 'u':u, 'v':v,
                'half_u':float((high[0]-low[0])/2+PADDING), 'half_v':float((high[1]-low[1])/2+PADDING),
                'thickness':PADDING, 'patch_a':patches[0],'patch_b':patches[1],
                'patch_a_triangle_ids':patch_ids[0],'patch_b_triangle_ids':patch_ids[1],
                'patch_a_sha256':hashlib.sha256(patches[0].astype('<f8').tobytes()).hexdigest(),
                'patch_b_sha256':hashlib.sha256(patches[1].astype('<f8').tobytes()).hexdigest(),
                'evidence':evidence+'; exact finite triangle patches on both components'})
    for bearing in bearings:
        origin, axis = bearing['origin'], bearing['axis']
        radius = np.hypot(bearing['eye_radius'], bearing['half_length']) + PADDING
        if np.min(np.linalg.norm(contacts - origin, axis=1)) > radius:
            continue
        ball = [np.count_nonzero(abs(np.linalg.norm(p - origin, axis=1) - bearing['ball_radius']) < FIT)
                for p in (first, second)]
        eye = []
        for points in (first, second):
            offset = points - origin
            eye.append(np.count_nonzero((abs(np.linalg.norm(np.cross(offset, axis), axis=1) - bearing['eye_radius']) < FIT)
                                        & (abs(offset @ axis) <= bearing['half_length'] + FIT)))
        if not ((ball[0] >= 20 and eye[1] >= 12) or (ball[1] >= 20 and eye[0] >= 12)):
            continue
        volumes.append({'kind': 'sphere', 'origin': origin, 'radius': float(radius),
            'evidence': bearing['evidence'], 'interface': 'finite spherical bearing assembly',
            'ball_support': ball, 'receiver_support': eye})
    if spheres:
        origins = np.array([s[0] for s in spheres])
        radii = np.array([s[1] for s in spheres])
        candidates = np.min(np.linalg.norm(contacts[:8, None] - origins, axis=2), axis=0) <= radii + PADDING
        spheres = [s for s, selected in zip(spheres, candidates) if selected]
    if cylinders:
        origins = np.array([c[0] for c in cylinders]); axes = np.array([c[1] for c in cylinders])
        radii = np.array([c[2] for c in cylinders])
        candidates = np.min(np.linalg.norm(np.cross(contacts[:8, None] - origins, axes), axis=2), axis=0) <= radii + PADDING
        cylinders = [c for c, selected in zip(cylinders, candidates) if selected]
    for origin, radius, evidence in spheres:
        if radius < .001 or np.min(np.linalg.norm(contacts - origin, axis=1)) > radius + PADDING:
            continue
        support = [p[abs(np.linalg.norm(p - origin, axis=1) - radius) < FIT] for p in (first, second)]
        if any(len(p) < 20 or np.min(np.ptp(p, axis=0)) < radius * .15 for p in support):
            continue
        key = ('sphere', *np.round(origin, 7), round(radius, 7))
        if key not in seen:
            seen.add(key)
            volumes.append({'kind': 'sphere', 'origin': origin, 'radius': radius + PADDING, 'evidence': evidence})
    for origin, axis, radius, evidence in cylinders:
        if radius < .001:
            continue
        radial = np.linalg.norm(np.cross(contacts - origin, axis), axis=1)
        if np.min(radial) > radius + PADDING:
            continue
        supported = []
        for points in (first, second):
            offsets = points - origin
            selected = abs(np.linalg.norm(np.cross(offsets, axis), axis=1) - radius) < FIT
            offsets = offsets[selected]
            if len(offsets) < 12:
                break
            u = np.cross(axis, [1., 0., 0.] if abs(axis[0]) < .8 else [0., 1., 0.])
            u /= np.linalg.norm(u)
            v = np.cross(axis, u)
            angles = np.sort(np.arctan2(offsets @ v, offsets @ u))
            if np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])) > np.pi / 3:
                break
            # Chamfers and end faces belong to the same finite coaxial pocket.
            # Bound its depth by actual component material inside the fitted radius.
            material = points - origin
            material = material[np.linalg.norm(np.cross(material, axis), axis=1) <= radius + FIT]
            stations = material @ axis
            supported.append((stations.min(), stations.max()))
        if len(supported) != 2:
            continue
        low, high = max(p[0] for p in supported), min(p[1] for p in supported)
        if high < low - FIT:
            continue
        middle = origin + axis * ((low + high) / 2)
        half = max(0., high - low) / 2 + PADDING
        key = ('cylinder', *np.round(middle, 7), *np.round(axis, 6), round(radius, 7), round(half, 7))
        if key not in seen:
            seen.add(key)
            volumes.append({'kind': 'cylinder', 'origin': middle, 'axis': axis,
                            'radius': radius + PADDING, 'half_length': half, 'evidence': evidence,
                            'axial_extent_source': 'Intersection of exact component material inside the shared STEP cylinder radius'})
    return volumes


def patch_contains(points, triangles, tolerance):
    result=np.zeros(len(points),dtype=bool)
    for start in range(0,len(points),128):
        query=points[start:start+128]
        edge1,edge2=triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]
        delta=query[:,None]-triangles[None,:,0]
        det=edge1[:,0]*edge2[:,1]-edge1[:,1]*edge2[:,0]
        valid=abs(det)>1e-25;safe=np.where(valid,det,1.)
        a=(delta[:,:,0]*edge2[:,1]-delta[:,:,1]*edge2[:,0])/safe
        b=(edge1[:,0]*delta[:,:,1]-edge1[:,1]*delta[:,:,0])/safe
        inside=((a>=-1e-12)&(b>=-1e-12)&(a+b<=1+1e-12)&valid).any(axis=1)
        for first,second in [(0,1),(1,2),(2,0)]:
            edge=triangles[:,second]-triangles[:,first]
            difference=query[:,None]-triangles[None,:,first]
            length=np.sum(edge*edge,axis=1)
            fraction=np.clip(np.sum(difference*edge,axis=2)/np.maximum(length,1e-30),0,1)
            distance=np.sum((difference-fraction[:,:,None]*edge)**2,axis=2)
            inside|=(distance<=tolerance*tolerance).any(axis=1)
        result[start:start+len(query)]=inside
    return result


def inside(points, volume):
    offset = points - volume['origin']
    if volume['kind'] == 'sphere':
        return np.linalg.norm(offset, axis=-1) <= volume['radius']
    if volume['kind'] == 'plane':
        mask=((abs(offset @ volume['axis'])<=volume['thickness'])
              &(abs(offset@volume['u'])<=volume['half_u'])&(abs(offset@volume['v'])<=volume['half_v']))
        mask &= abs((points-volume.get('origin_b',volume['origin']))@volume.get('axis_b',volume['axis']))<=volume['thickness']
        if mask.any():
            basis=np.column_stack((volume['u'],volume['v']))
            selected=points[mask]@basis
            allowed=patch_contains(selected,volume['patch_a']@basis,PADDING)&patch_contains(selected,volume['patch_b']@basis,PADDING)
            mask[np.flatnonzero(mask)]=allowed
        return mask
    stations = offset @ volume['axis']
    return ((abs(stations) <= volume['half_length'])
            & (np.linalg.norm(np.cross(offset, volume['axis']), axis=-1) <= volume['radius']))


def component_digest(points, triangles):
    digest = hashlib.sha256()
    digest.update(np.asarray(points, dtype='<f8').tobytes())
    digest.update(np.asarray(triangles, dtype='<i8').tobytes())
    return digest.hexdigest()


def cylinder_material_intervals(points, triangles, anchor, axis, radius):
    """Exact axial extrema of triangle surfaces intersecting an infinite cylinder."""
    u=np.cross(axis,[1.,0.,0.] if abs(axis[0])<.8 else [0.,1.,0.]);u/=np.linalg.norm(u);v=np.cross(axis,u)
    offset=points-anchor
    radial=offset@np.column_stack((u,v));stations=offset@axis
    xy=radial[triangles];z=stations[triangles]
    lower,upper=xy.min(axis=1),xy.max(axis=1)
    nearest=np.maximum(lower,np.minimum(0.,upper))
    possible=np.sum(nearest*nearest,axis=1)<=radius*radius
    xy,z=xy[possible],z[possible]
    if not len(xy):return []
    inside=np.sum(xy*xy,axis=2)<=radius*radius
    lows=np.min(np.where(inside,z,np.inf),axis=1);highs=np.max(np.where(inside,z,-np.inf),axis=1)
    boundary=~inside.all(axis=1);indices=np.flatnonzero(boundary);xy,z=xy[boundary],z[boundary]
    for i,j in [(0,1),(1,2),(2,0)]:
        start=xy[:,i];direction=xy[:,j]-start
        a=np.sum(direction*direction,axis=1);b=2*np.sum(start*direction,axis=1);c=np.sum(start*start,axis=1)-radius*radius
        discriminant=b*b-4*a*c;valid=(a>1e-30)&(discriminant>=0)
        for sign in [-1,1]:
            t=np.zeros(len(a));t[valid]=(-b[valid]+sign*np.sqrt(discriminant[valid]))/(2*a[valid])
            chosen=valid&(t>=0)&(t<=1)
            values=z[chosen,i]+t[chosen]*(z[chosen,j]-z[chosen,i])
            np.minimum.at(lows,indices[chosen],values);np.maximum.at(highs,indices[chosen],values)
    # A circle can lie wholly inside a coarse triangle, without crossing its edges.
    e1,e2=xy[:,1]-xy[:,0],xy[:,2]-xy[:,0]
    determinant=e1[:,0]*e2[:,1]-e1[:,1]*e2[:,0]
    valid=abs(determinant)>1e-25
    indices=indices[valid]
    xy,z,e1,e2,determinant=xy[valid],z[valid],e1[valid],e2[valid],determinant[valid]
    dz1,dz2=z[:,1]-z[:,0],z[:,2]-z[:,0]
    gradient=np.column_stack(((dz1*e2[:,1]-dz2*e1[:,1])/determinant,
                              (e1[:,0]*dz2-e2[:,0]*dz1)/determinant))
    length=np.linalg.norm(gradient,axis=1)
    direction=np.zeros_like(gradient);nonzero=length>1e-15;direction[nonzero]=gradient[nonzero]/length[nonzero,None]
    for sign in [-1,1]:
        target=sign*radius*direction-xy[:,0]
        first=(target[:,0]*e2[:,1]-target[:,1]*e2[:,0])/determinant
        second=(e1[:,0]*target[:,1]-e1[:,1]*target[:,0])/determinant
        selected=(first>=-1e-12)&(second>=-1e-12)&(first+second<=1+1e-12)
        values=(z[:,0]+first*dz1+second*dz2)[selected]
        np.minimum.at(lows,indices[selected],values);np.maximum.at(highs,indices[selected],values)
    return merge_intervals([(float(a),float(b)) for a,b in zip(lows,highs) if a<=b])


def merge_intervals(intervals):
    merged=[]
    for low,high in sorted(intervals):
        if merged and low<=merged[-1][1]+FIT:
            merged[-1][1]=max(high,merged[-1][1])
        else:merged.append([low,high])
    return merged


def interval_intersections(first,second):
    for a,b in first:
        for c,d in second:
            low,high=max(a,c),min(b,d)
            gap=max(0.,low-high)
            if gap>FIT:continue
            if high<low:low=high=(low+high)/2
            yield low,high,gap


def cylinder_material_bounds(points, triangles, anchor, axis, radius):
    intervals=cylinder_material_intervals(points,triangles,anchor,axis,radius)
    return (intervals[0][0],intervals[-1][1]) if intervals else None


def motor_interiors(model, data, parts, world, cylinders, bearing_joints=()):
    """User-authorized motor cavities; exterior contacts are never excluded."""
    interiors = {}
    motors={int(model.actuator_trnid[actuator,0]) for actuator in range(model.nu)}
    joints=motors|{int(model.joint(name).id) for name in bearing_joints}
    for joint in joints:
        rotor = int(model.jnt_bodyid[joint]); stator = int(model.body_parentid[rotor])
        axis, anchor = data.xaxis[joint], data.xanchor[joint]
        candidates = []
        material_cache={}
        def material_at(geom,index,radius):
            key=(geom,index,radius)
            if key not in material_cache:
                material_cache[key]=cylinder_material_intervals(world[geom][index],parts[geom][index]['triangles'],anchor,axis,radius+FIT)
            return material_cache[key]
        radii = {}
        for origin, direction, radius, evidence in cylinders:
            if (abs(np.dot(axis, direction)) < np.cos(np.deg2rad(.02))
                    or np.linalg.norm(np.cross(anchor-origin, direction)) > 2*FIT):
                continue
            radii.setdefault(round(radius, 7), evidence)
        for geom, components in parts.items():
            if int(model.geom_bodyid[geom]) not in (stator,rotor):
                continue
            for index, component in enumerate(components):
                offsets = world[geom][index] - anchor
                stations = offsets @ axis
                radial = np.linalg.norm(np.cross(offsets, axis), axis=1)
                for radius, evidence in radii.items():
                    supported = abs(radial-radius)<FIT
                    points = offsets[supported]
                    if len(points)<12:
                        continue
                    u=np.cross(axis,[1.,0.,0.] if abs(axis[0])<.8 else [0.,1.,0.]);u/=np.linalg.norm(u);v=np.cross(axis,u)
                    angles=np.sort(np.arctan2(points@v,points@u))
                    gap=float(np.max(np.diff(np.r_[angles,angles[0]+2*np.pi])))
                    if gap>np.pi/2:
                        continue
                    material=material_at(geom,index,radius)
                    if not material:continue
                    other=[];other_records=[]
                    for other_geom,other_components in parts.items():
                        if int(model.geom_bodyid[other_geom])!=(rotor if int(model.geom_bodyid[geom])==stator else stator):continue
                        for other_index in range(len(other_components)):
                            other_part=other_components[other_index]
                            bounds=material_at(other_geom,other_index,radius)
                            other.extend(bounds)
                            if bounds:other_records.append({'component':other_part['tag'],'digest':other_part['digest'],'axial_intervals_m':bounds})
                    if not other:continue
                    other=merge_intervals(other)
                    for low,high,seam_gap in interval_intersections(material,other):
                        candidates.append({'kind':'cylinder','origin':anchor+axis*((low+high)/2),
                            'axis':axis.copy(),'radius':float(radius+PADDING),'half_length':(high-low)/2+PADDING,
                            'evidence':evidence+'; at least 270 degree housing arc supported by exact STL',
                            'circular_support_count':len(points),'maximum_angular_gap_deg':float(np.rad2deg(gap)),
                            'axial_bounds_from_joint_m':[low,high],
                            'axial_fit_gap_m':seam_gap,'axial_fit_tolerance_m':FIT,
                            'interface':'user-authorized finite motor interior' if joint in motors else 'verified finite coaxial bearing pocket',
                            'motor_joint':model.joint(joint).name,'stator_body':model.body(stator).name,
                            'rotor_body':model.body(rotor).name,'housing_component':component['tag'],
                            'housing_digest':component['digest'],'axial_extent_source':'Connected intersections of exact triangle-cylinder material projections',
                            'housing_axial_intervals_m':material,'opposite_axial_intervals_m':other,
                            'opposite_component_projections':other_records})

        if candidates:
            interiors[frozenset((stator,rotor))]=candidates
    return interiors


def bearing_interiors(model, parts, world, bearings):
    """Retaining screws share a verified bearing unit, not necessarily its mesh component."""
    result={}
    for bearing in bearings:
        rod, mate=bearing['body_pair'];center=bearing['origin'];axis=bearing['axis']
        ball_parts=[];eye_parts=[]
        for geom,components in parts.items():
            body=int(model.geom_bodyid[geom])
            if body not in (rod,mate):continue
            for index,component in enumerate(components):
                offset=world[geom][index]-center
                if body==mate and np.count_nonzero(abs(np.linalg.norm(offset,axis=1)-bearing['ball_radius'])<FIT)>=20:
                    ball_parts.append(component['digest'])
                if body==rod and np.count_nonzero((abs(np.linalg.norm(np.cross(offset,axis),axis=1)-bearing['eye_radius'])<FIT)
                                                 &(abs(offset@axis)<=bearing['half_length']+FIT))>=12:
                    eye_parts.append(component['digest'])
        if not ball_parts or not eye_parts:continue
        volume={'kind':'sphere','origin':center,'radius':float(np.hypot(bearing['eye_radius'],bearing['half_length'])+PADDING),
                'evidence':bearing['evidence'],'interface':'verified finite spherical bearing unit',
                'ball_component_digests':ball_parts,'receiver_component_digests':eye_parts}
        result.setdefault(frozenset((rod,mate)),[]).append(volume)
    return result
