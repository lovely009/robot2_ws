// Independent display geometry; no ROS/store imports or motor commands.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from '../vendor/URDFLoader.js';

function disposeModel(model) {
  const geometries = new Set(), materials = new Set(), textures = new Set();
  model.traverse(o => {
    if (o.geometry) geometries.add(o.geometry);
    for (const material of (Array.isArray(o.material) ? o.material : [o.material])) {
      if (!material) continue;
      materials.add(material);
      for (const value of Object.values(material)) if (value?.isTexture) textures.add(value);
    }
  });
  for (const resource of [...geometries, ...materials, ...textures]) resource.dispose();
}

function footBranches({ names, parents }) {
  const named = new Set(names.flatMap((name, i) => /foot|toe|ankle/i.test(name) ? [i] : []));
  const belowFoot = index => {
    for (let parent = parents[index]; parent >= 0; parent = parents[parent]) {
      if (named.has(parent)) return true;
    }
    return false;
  };
  return {
    roots: [...named].filter(index => !belowFoot(index)),
    indices: names.flatMap((_, i) => named.has(i) || belowFoot(i) ? [i] : []),
  };
}

export function createMotionPreview(canvas, data, { onModelStatus = () => {} } = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x131b1f);
  scene.add(new THREE.HemisphereLight(0xe9f1ff, 0x263134, 2));
  const light = new THREE.DirectionalLight(0xffffff, 2.5);
  light.position.set(2, -3, 4);
  scene.add(light);
  const camera = new THREE.PerspectiveCamera(42, 1, .01, 100);
  camera.up.set(0, 0, 1);
  camera.position.set(3.2, -3, 2.3);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0, .7, .8);
  controls.enableDamping = true;
  const grid = new THREE.GridHelper(8, 40, 0x3a4b50, 0x202d30);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  const robotToSource = new THREE.Matrix3().set(...data.source_to_robot.flat()).transpose();
  const robotToSourceRotation = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().setFromMatrix3(robotToSource));
  // Root rotations, including older cached results, are expressed in robot coordinates.
  const rootRotation = index => new THREE.Quaternion().fromArray(
    data.robot.root_quaternions?.[index] || [0, 0, 0, 1]);
  const displayRotation = index => robotToSourceRotation.clone().multiply(rootRotation(index));
  const groups = [data.source, data.robot].map((skeleton, index) => {
    const geometry = new THREE.BufferGeometry();
    const edges = skeleton.parents.flatMap((parent, child) => parent < 0 ? [] : [parent, child]);
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(edges.length * 3), 3));
    const material = new THREE.LineBasicMaterial({ color: index ? 0x85dfb8 : 0x86bcea });
    const lines = new THREE.LineSegments(geometry, material);
    scene.add(lines);
    const pointGeometry = new THREE.BufferGeometry();
    pointGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(skeleton.names.length * 3), 3));
    const pointMaterial = new THREE.PointsMaterial({ color: index ? 0x85dfb8 : 0x86bcea, size: .025 });
    const points = new THREE.Points(pointGeometry, pointMaterial);
    scene.add(points);
    return { skeleton, edges, geometry, material, pointGeometry, pointMaterial, lines, points };
  });
  const floorOffsets = groups.map(({ skeleton }, side) => -Math.min(...skeleton.frames[0].map(
    point => side
      ? new THREE.Vector3(...point).applyQuaternion(displayRotation(0)).z
      : point[2])));
  const sourceFeet = footBranches(data.source), robotFeet = footBranches(data.robot);
  const supportHeight = sourceFeet.roots.length >= 2 && robotFeet.roots.length >= 2;
  const sourceFootHeights = supportHeight ? data.source.frames.map(points =>
    Math.min(...sourceFeet.indices.map(index => points[index][2]))) : [];
  // shortcut: estimate a flat floor from the lower 5% of foot heights; use contact data for uneven ground.
  const sourceFloor = supportHeight ? [...sourceFootHeights].sort((a, b) => a - b)[
    Math.floor((sourceFootHeights.length - 1) * .05)] : 0;
  if (supportHeight) floorOffsets[0] = -sourceFloor;
  const sourceRoot = Math.max(0, data.source.names.indexOf(data.robot.root_rotation_source));
  const sourceReferenceHeight = data.source.frames[0][sourceRoot][2] - sourceFloor;
  const robotFootHeight = index => Math.min(...robotFeet.indices.map(id =>
    new THREE.Vector3(...data.robot.frames[index][id]).applyQuaternion(displayRotation(index)).z));
  const scaleHeight = height => height > 1e-6 && sourceReferenceHeight > 1e-6
    ? height / sourceReferenceHeight : 1;
  let heightScale = supportHeight ? scaleHeight(-robotFootHeight(0)) : 1;
  const airborneHeight = index => Math.max(0, sourceFootHeights[index] - sourceFloor) * heightScale;
  const supportHeights = [];
  let footLinks = [];
  const soleHeight = () => Math.min(...footLinks.map(link => new THREE.Box3().setFromObject(link, true).min.z));
  let frame = 0;
  let model = null, placement = null, disposed = false, grounded = false;
  function show(index) {
    frame = Math.max(0, Math.min(index, data.times.length - 1));
    if (model && data.robot.joint_positions) {
      placement.quaternion.copy(displayRotation(frame));
      model.setJointValues(Object.fromEntries(data.robot.joint_names.map(
        (name, i) => [name, data.robot.joint_positions[frame][i]])));
    }
    if (supportHeight) {
      if (grounded && footLinks.length >= 2) {
        if (supportHeights[frame] === undefined) {
          placement.position.z = 0;
          placement.updateMatrixWorld(true);
          supportHeights[frame] = -soleHeight() + airborneHeight(frame);
        }
        floorOffsets[1] = supportHeights[frame];
      } else floorOffsets[1] = -robotFootHeight(frame) + airborneHeight(frame);
      if (placement) placement.position.z = floorOffsets[1];
    }
    groups.forEach(({ skeleton, edges, geometry, pointGeometry }, side) => {
      const raw = skeleton.frames[frame];
      const points = raw.map(([x, y, z]) => {
        const p = new THREE.Vector3(x, y, z);
        // Source frames already contain the world-space BVH root orientation.
        if (side) p.applyQuaternion(displayRotation(frame))
          .add(new THREE.Vector3(0, 1.5, 0));
        else {
          p.x -= raw[0][0];
          p.y -= raw[0][1];
        }
        p.z += floorOffsets[side];
        return p;
      });
      const positions = geometry.attributes.position;
      edges.forEach((id, i) => positions.setXYZ(i, points[id].x, points[id].y, points[id].z));
      positions.needsUpdate = true;
      const vertices = pointGeometry.attributes.position;
      points.forEach((p, i) => vertices.setXYZ(i, p.x, p.y, p.z));
      vertices.needsUpdate = true;
      geometry.computeBoundingSphere();
      pointGeometry.computeBoundingSphere();
    });
  }
  if (data.robot.urdf && data.robot.joint_positions) {
    onModelStatus('正在加载机器人模型…');
    const document = new DOMParser().parseFromString(data.robot.urdf, 'text/xml');
    const expected = document.querySelectorAll('link > visual > geometry > mesh').length;
    const manager = new THREE.LoadingManager();
    // Uploaded URDF assets are resolved only within the project's mounted model directory.
    manager.setURLModifier(raw => {
      const url = new URL(raw, location.origin);
      if (url.origin !== location.origin || !url.pathname.startsWith('/robot_sim/'))
        throw Error('缺少工程内模型网格');
      return url.href;
    });
    const loader = new URDFLoader(manager);
    loader.packages = { robot_sim: '/robot_sim', robot_description: '/robot_sim' };
    loader.parseCollision = false;
    let pending = 0, loaded = 0;
    function finish() {
      if (disposed || !model || pending) return;
      let meshes = 0;
      model.traverse(o => { if (o.isMesh) meshes++; });
      const complete = meshes > 0 && loaded === expected;
      if (complete && !grounded) {
        model.setJointValues(Object.fromEntries(data.robot.joint_names.map(
          (name, i) => [name, data.robot.joint_positions[0][i]])));
        placement.position.z = 0;
        placement.quaternion.copy(displayRotation(0));
        placement.updateMatrixWorld(true);
        footLinks = robotFeet.roots.map(index => model.links[data.robot.names[index]]).filter(Boolean);
        floorOffsets[1] = -(supportHeight && footLinks.length >= 2 ? soleHeight()
          : new THREE.Box3().setFromObject(model).min.z);
        if (supportHeight) heightScale = scaleHeight(floorOffsets[1]);
        placement.position.z = floorOffsets[1];
        grounded = true;
        show(frame);
      }
      groups[1].lines.visible = groups[1].points.visible = !complete;
      onModelStatus(complete ? '机器人实体模型' : meshes
        ? '部分网格缺失，保留骨架' : '未找到模型网格，显示骨架');
    }
    loader.loadMeshCb = (path, assetManager, material, done) => {
      pending++;
      const complete = (object, error) => {
        pending--;
        if (disposed) { if (object) disposeModel(object); else material?.dispose(); return; }
        if (object) loaded++;
        done(object, error);
        finish();
      };
      try {
        if (!/\.(stl|dae)$/i.test(new URL(path, location.origin).pathname))
          throw Error('仅支持 STL 或 DAE 网格');
        loader.defaultMeshLoader(path, assetManager, material, complete);
      } catch (error) { complete(null, error); }
    };
    try {
      const name = document.documentElement.getAttribute('name') || '';
      model = loader.parse(document, `/robot_sim/models/${encodeURIComponent(name)}/urdf/`);
      if (data.robot.ignore_limits === true) {
        Object.values(model.joints).forEach(joint => { joint.ignoreLimits = true; });
      }
      placement = new THREE.Group();
      placement.quaternion.copy(displayRotation(0));
      placement.position.set(0, 1.5, floorOffsets[1]);
      placement.add(model);
      scene.add(placement);
      finish();
    } catch {
      onModelStatus('模型加载失败，显示骨架');
    }
  } else onModelStatus('机器人骨架');
  const resize = new ResizeObserver(() => {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    renderer.setSize(rect.width, rect.height, false);
    camera.aspect = rect.width / rect.height;
    camera.updateProjectionMatrix();
  });
  resize.observe(canvas);
  renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
  show(0);
  return {
    show,
    dispose() {
      disposed = true;
      renderer.setAnimationLoop(null);
      resize.disconnect();
      controls.dispose();
      if (model) disposeModel(model);
      groups.forEach(g => [g.geometry, g.material, g.pointGeometry, g.pointMaterial].forEach(r => r.dispose()));
      grid.geometry.dispose();
      (Array.isArray(grid.material) ? grid.material : [grid.material]).forEach(m => m.dispose());
      renderer.dispose();
    },
  };
}
