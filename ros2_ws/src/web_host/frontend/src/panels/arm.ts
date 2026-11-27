// URDF 三维视图，从 store 读取关节反馈，并提供轴标、拾取和状态着色。
// package://robot_description/ 资源由 loader 映射到 /robot_sim/。

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from '/vendor/URDFLoader.js';
import { store, DIRTY, markDirty, MODEL, availableModels, selectGroups } from '../core/store';
import { collectModelJoints } from '../core/tf';
import { $ } from '../core/format';
import type { MountedPanel } from '../contracts';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const MARKER_R = 0.022;
const AXIS_LENGTH = MARKER_R * 3;
const AXIS_RING_R = MARKER_R * 0.65;

let renderer: THREE.WebGLRenderer | null = null;
let scene: THREE.Scene | null = null;
let camera: THREE.PerspectiveCamera | null = null;
let controls: OrbitControls | null = null;
let robot: any = null;
let ghost: any = null;
let root: THREE.Group | null = null;
export const markers = new Map<string, THREE.Mesh>();
const picked: THREE.Mesh[] = [];
let selected: string | null = null;
let onSelect: ((name: string) => void) | null = null;
let ghostMat: THREE.MeshBasicMaterial | null = null;
let loadGen = 0;

let renderRequested = true;
let lastRenderAt = -Infinity;
function tick(now: number) {
  if (!renderer || !controls || !scene || !camera || !renderer.domElement.clientWidth || !renderer.domElement.clientHeight || now - lastRenderAt < 33) return;
  const moved = controls.update();
  if (!renderRequested && !moved) return;
  renderer.render(scene, camera);
  if (renderer.domElement.dataset.modelReady === 'true') renderer.domElement.dataset.modelRendered = 'true';
  renderRequested = false;
  lastRenderAt = now;
}

export function getSceneObjects() {
  return { robot, ghost, scene };
}

export function init(canvas: HTMLCanvasElement, opts: {
  urdfUrl: string;
  initialPositionsUrl?: string;
  packages?: Record<string, string>;
  onSelect?: (name: string) => void;
  onProgress?: (url: string, loaded: number, total: number) => void;
  onReady?: (joints: any[]) => void;
  onError?: (err: any) => void;
}) {
  const { urdfUrl, initialPositionsUrl, packages = {}, onProgress, onReady, onError } = opts;
  onSelect = opts.onSelect || null;

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  canvas.dataset.modelReady = 'false';
  canvas.dataset.modelRendered = 'false';
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  renderRequested = true;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1120);

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  camera.position.set(1.1, -1.1, 0.8);

  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 0.25);

  const hemi = new THREE.HemisphereLight(0xdfe9ff, 0x1b2438, 2.0);
  hemi.position.set(0, 0, 1);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(2, -2, 3);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.9);
  fill.position.set(-2, 2, -1.5);
  scene.add(fill);

  const grid = new THREE.GridHelper(2, 20, 0x2a3550, 0x1b2438);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  root = new THREE.Group();
  scene.add(root);

  ghostMat = new THREE.MeshBasicMaterial({
    color: 0x4da3ff, transparent: true, opacity: 0.18, depthWrite: false,
  });

  canvas.addEventListener('pointerdown', onPointerDown);

  const manager = new THREE.LoadingManager();
  manager.setURLModifier((url) => (/\.(?:stl|dae)$/i.test(url) ? `${url}?v=2` : url));
  const loader = new URDFLoader(manager);
  loader.packages = packages;
  const gen = ++loadGen;
  manager.onProgress = () => { if (gen === loadGen) renderRequested = true; };
  manager.onLoad = () => {
    if (gen !== loadGen || !robot || !root) return;
    if (ghost) root.remove(ghost);
    ghost = robot.clone();
    ghost.traverse((o: any) => { if (o.isMesh && !o.userData.hitOnly) o.material = ghostMat; });
    ghost.visible = false;
    root.add(ghost);
    if (!initialPositionsUrl) placeGround(robot, grid);
    frameCamera(robot);
    renderRequested = true;
    canvas.dataset.modelReady = 'true';
  };
  const load = (initialPositions: Record<string, number> | null) => loader.load(urdfUrl, (r) => {
    if (gen !== loadGen || !root) { return; }
    robot = r;
    if (!initialPositions && robot.robotName === 'full_body_robot' && robot.joints.BN_Spine_yaw) {
      robot.quaternion.copy(robot.joints.BN_Spine_yaw.quaternion).invert();
    }
    applyValues(robot, initialPositions);
    robot.traverse((o: any) => { if (o.isMesh) { o.frustumCulled = false; } });
    root.add(robot);

    const modelJoints = collectModelJoints(robot);
    addMarkers(robot, modelJoints.map(joint => joint.name));

    ghost = robot.clone();
    ghost.traverse((o: any) => { if (o.isMesh && !o.userData.hitOnly) { o.material = ghostMat; } });
    ghost.visible = false;
    root.add(ghost);

    frameCamera(robot);
    resize();
    if (onReady) { onReady(modelJoints); }
  }, (url, loaded, total) => {
    if (gen === loadGen && onProgress) { onProgress(url, loaded, total); }
  }, (err) => {
    if (gen !== loadGen) { return; }
    if (onError) { onError(err); }
  });
  if (initialPositionsUrl) {
    fetch(initialPositionsUrl).then(response => {
      if (!response.ok) throw new Error(`站姿配置读取失败（HTTP ${response.status}）`);
      return response.json();
    }).then(positions => {
      if (gen === loadGen) load(positions);
    }).catch(error => {
      if (gen === loadGen && onError) onError(error);
    });
  } else {
    load(null);
  }

  renderer.setAnimationLoop(tick);
}

export function placeGround(model: any, grid: THREE.GridHelper) {
  const bounds = new THREE.Box3();
  const part = new THREE.Box3();
  model.updateMatrixWorld(true);
  for (const visual of Object.values(model.visual || {}) as any[]) {
    visual.traverse((object: any) => {
      if (!object.isMesh) return;
      if (!object.geometry.boundingBox) object.geometry.computeBoundingBox();
      part.copy(object.geometry.boundingBox).applyMatrix4(object.matrixWorld);
      bounds.union(part);
    });
  }
  if (!bounds.isEmpty()) {
    const centre = bounds.getCenter(new THREE.Vector3());
    grid.position.set(centre.x, centre.y, bounds.min.z);
  }
}

export function addMarkers(r: any, names: string[]) {
  for (const name of names) {
    const joint = r.joints[name];
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(MARKER_R, 16, 12),
      new THREE.MeshBasicMaterial({
        transparent: true, opacity: 0, depthWrite: false, colorWrite: false,
      }));
    m.userData.jointName = name;
    m.userData.hitOnly = true;
    const axis = joint.axis && joint.axis.lengthSq() > 0
      ? joint.axis.clone().normalize() : new THREE.Vector3(0, 0, 1);
    const color = 0x7e8bab;
    const arrow = new THREE.ArrowHelper(
      axis, axis.clone().multiplyScalar(-AXIS_LENGTH / 2), AXIS_LENGTH,
      color, MARKER_R * 0.55, MARKER_R * 0.3);
    const points = Array.from({ length: 32 }, (_, i) => {
      const angle = i / 32 * Math.PI * 2;
      return new THREE.Vector3(Math.cos(angle) * AXIS_RING_R, Math.sin(angle) * AXIS_RING_R, 0);
    });
    const ring = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color }));
    ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
    for (const part of [arrow.line, arrow.cone, ring]) {
      part.material.depthTest = false; part.material.depthWrite = false;
      part.renderOrder = 20;
    }
    m.add(arrow, ring);
    joint.add(m);
    markers.set(name, m);
    picked.push(m);
  }
}

function setCameraView(view: 'front' | 'side' | 'top' | 'reset') {
  if (!controls || !camera || !robot) return;
  const box = new THREE.Box3().setFromObject(robot);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  const d = size * 1.15;
  controls.target.copy(c);
  if (view === 'front') camera.position.set(c.x + d, c.y, c.z + d * 0.15);
  else if (view === 'side') camera.position.set(c.x, c.y - d, c.z + d * 0.15);
  else if (view === 'top') camera.position.set(c.x + d * 0.05, c.y, c.z + d * 1.25);
  else camera.position.set(c.x + d * 0.75, c.y - d * 0.75, c.z + d * 0.5);
  camera.updateProjectionMatrix();
  renderRequested = true;
  controls.update();
}

function frameCamera(r: any) {
  if (!controls || !camera) return;
  const box = new THREE.Box3().setFromObject(r);
  if (box.isEmpty()) { return; }
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  const d = size * 1.15;
  controls.target.copy(c);
  camera.position.set(c.x + d * 0.75, c.y - d * 0.75, c.z + d * 0.5);
  camera.near = d / 100;
  camera.far = d * 20;
  camera.updateProjectionMatrix();
  renderRequested = true;
  controls.update();
}

function onPointerDown(ev: PointerEvent) {
  if (!onSelect || !robot || !renderer || !camera) { return; }
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    -((ev.clientY - rect.top) / rect.height) * 2 + 1);
  const ray = new THREE.Raycaster();
  ray.setFromCamera(ndc, camera);
  const hits = ray.intersectObjects(picked, false);
  if (hits.length) { onSelect(hits[0].object.userData.jointName); }
}

export function applyValues(target: any, angles: Record<string, number> | null) {
  if (!target || !angles) return;
  for (const [name, v] of Object.entries(angles)) {
    const joint = target.joints[name];
    if (joint && Number.isFinite(v) && joint.angle !== v) {
      joint.ignoreLimits = true;
      target.setJointValue(name, v);
      renderRequested = true;
    }
  }
}

export function setJoints(angles: Record<string, number>) { applyValues(robot, angles); }

export function setTargets(angles: Record<string, number> | null) {
  if (!ghost) { return; }
  if (!angles) { if (ghost.visible) renderRequested = true; ghost.visible = false; return; }
  applyValues(ghost, angles);
  ghost.visible = true;
}

export function setJointColors(colors: Record<string, string>) {
  for (const [name, m] of markers) {
    const next = (colors && colors[name]) || '#7e8bab';
    const [arrow, ring] = m.children as [any, any];
    if (`#${ring.material.color.getHexString()}` !== next) {
      arrow.setColor(next); ring.material.color.set(next); renderRequested = true;
    }
  }
}

export function setSelected(name: string | null) {
  if (selected !== name) renderRequested = true;
  if (selected && markers.has(selected)) {
    markers.get(selected)!.scale.setScalar(1);
  }
  selected = name;
  if (selected && markers.has(selected)) {
    markers.get(selected)!.scale.setScalar(1.35);
  }
}

export function resize() {
  if (!renderer || !camera) { return; }
  const c = renderer.domElement;
  const w = c.clientWidth || 1;
  const h = c.clientHeight || 1;
  const size = renderer.getSize(new THREE.Vector2());
  if (size.x === w && size.y === h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderRequested = true;
}

export function setRunning(on: boolean) {
  if (!renderer) { return; }
  renderer.setAnimationLoop(on ? tick : null);
}

const onWindowResize = () => resize();
let panelActive = true;
const onVisibilityChange = () => setRunning(panelActive && !document.hidden);

function disposeScene() {
  loadGen += 1;
  if (!scene) return;
  const geoms = new Set<any>();
  const mats = new Set<any>();
  scene.traverse((o: any) => {
    if (o.geometry) geoms.add(o.geometry);
    if (o.material) {
      for (const m of (Array.isArray(o.material) ? o.material : [o.material])) mats.add(m);
    }
  });
  for (const g of geoms) g.dispose();
  for (const m of mats) {
    for (const k of ['map', 'normalMap', 'roughnessMap', 'metalnessMap', 'alphaMap']) {
      if (m[k]) m[k].dispose();
    }
    m.dispose();
  }
  markers.clear();
  picked.length = 0;
  if (controls) { controls.dispose(); controls = null; }
  if (renderer) { renderer.dispose(); renderer = null; }
  scene = null; camera = null; robot = null; ghost = null; root = null;
  selected = null;
}

const JOINT_STALE_MS = 1500;
function groupJointNames(): string[] {
  return [...new Set<string>(store.modelTransmissions.groups.flatMap((g: any) => g.joints))];
}

export function armJointAngles(jointStates = store.jointStates, now = Date.now()): Record<string, number> {
  const angles: Record<string, number> = {};
  const names = jointStates.jointNames
    || store.modelTransmissions.groups.flatMap((g: any) => g.joints);
  for (const name of new Set<string>(names)) {
    const reading = jointStates.byName && jointStates.byName[name];
    if (reading && Number.isFinite(reading.position) && now - reading.at < JOINT_STALE_MS) {
      angles[name] = reading.position;
    }
  }
  return angles;
}

function armSyncJoints() {
  setJoints(armJointAngles());
}

function armSyncGhost() {
  setTargets(null);
}

function armSyncColors() {
  const colors: Record<string, string> = {};
  const now = Date.now();
  for (const name of groupJointNames()) {
    const reading = store.jointStates.byName[name];
    colors[name] = !reading ? '#7e8bab'
      : (now - reading.at >= JOINT_STALE_MS ? '#f5b94a' : '#37d67a');
  }
  setJointColors(colors);
}

export function mount(rootEl: HTMLElement, ctx: any): MountedPanel {
  const stage = rootEl.querySelector('.stage') || rootEl;
  let canvas = document.createElement('canvas');
  canvas.className = 'arm3d';
  const hud = document.createElement('div');
  hud.className = 'hud';
  hud.tabIndex = 0;
  hud.setAttribute('aria-label', '模型加载与映射诊断');
  hud.textContent = '臂视图加载中…';
  stage.prepend(hud);
  stage.prepend(canvas);

  let ready = false;
  let loadedModel: string | null = null;
  let hudFadeTimer = 0;
  let modelSelect: HTMLSelectElement | null = null;
  panelActive = true;

  const pickJoint = (jointName: string) => {
    const groups = store.modelTransmissions.groups
      .filter((g: any) => g.joints.includes(jointName)).map((g: any) => g.name);
    store.selectedJoint = jointName;
    selectGroups(groups);
  };

  modelSelect = $('armModel');
  if (modelSelect) {
    modelSelect.disabled = false;
    const fillOptions = (names: string[]) => {
      if (!modelSelect) return;
      const list = names.includes(MODEL) ? names : [MODEL, ...names];
      modelSelect.replaceChildren(...list.map((name) => {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        return option;
      }));
      modelSelect.value = MODEL;
    };
    fillOptions([]);
    availableModels().then(fillOptions).catch(() => {});
    modelSelect.addEventListener('change', () => {
      if (modelSelect?.value) { ctx.ui.setModel(modelSelect.value); }
    });
  }

  function startLoad() {
    loadedModel = MODEL;
    ready = false;
    window.clearTimeout(hudFadeTimer);
    hud.className = 'hud';
    hud.style.opacity = '';
    hud.textContent = '臂视图加载中…';
    if (renderer) {
      disposeScene();
      canvas.remove();
      canvas = document.createElement('canvas');
      canvas.className = 'arm3d';
      stage.prepend(canvas);
    }
    const simulationVisual = MODEL === 'full_body_robot';
    const modelUrl = `/robot_sim/models/${MODEL}`;
    const urdfName = simulationVisual ? `${MODEL}.standing.urdf` : `${MODEL}.urdf`;
    init(canvas, {
      urdfUrl: `${modelUrl}/urdf/${urdfName}`,
      initialPositionsUrl: simulationVisual ? `${modelUrl}/config/standing.json` : undefined,
      packages: { 'robot_description': '/robot_sim' },
      onSelect: pickJoint,
      onProgress: (_url, loaded, total) => {
        if (Number.isFinite(loaded) && Number.isFinite(total) && total > 0) {
          hud.textContent = `臂视图加载中 ${Math.round(loaded / total * 100)}%`;
        }
      },
      onReady: (joints) => {
        const names = joints.map((j) => j.name);
        const mapped = new Set(groupJointNames());
        const extra = simulationVisual ? [] : names.filter((j) => !mapped.has(j));
        hud.textContent = `${simulationVisual ? '整机仿真模型' : '臂视图'}就绪 · ${names.length} 个关节`
          + (extra.length ? ` · 未映射：${extra.join(' ')}` : '')
          + ' · 几何已加载';
        ready = true;
        setSelected(null);
        markDirty(DIRTY.MOTION);
        hudFadeTimer = window.setTimeout(() => { hud.style.opacity = '0.3'; }, 5000);
      },
      onError: (err) => {
        hud.className = 'hud err';
        hud.textContent = '臂视图加载失败：' + (err && err.message ? err.message : err)
          + '（查 static_server 是否挂载 /robot_sim、static server 日志）';
      },
    });
    setRunning(panelActive && !document.hidden);
  }

  startLoad();
  window.addEventListener('resize', onWindowResize);
  const onCamClick = (event: Event) => {
    const btn = (event.target as Element | null)?.closest<HTMLElement>('[data-cam-view]');
    const view = btn?.dataset.camView as 'front' | 'side' | 'top' | 'reset' | undefined;
    if (view) setCameraView(view);
  };
  rootEl.addEventListener('click', onCamClick);

  const sizeObserver = new ResizeObserver(() => resize());
  sizeObserver.observe(stage);
  document.addEventListener('visibilitychange', onVisibilityChange);

  const render = (dirty: Set<string>) => {
    if (dirty.has(DIRTY.MODEL) && loadedModel !== MODEL) { startLoad(); }
    if (modelSelect && modelSelect.value !== MODEL) { modelSelect.value = MODEL; }
    if (!ready) { return; }
    if (dirty.has(DIRTY.MOTION)) { armSyncJoints(); armSyncGhost(); }
    if (dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY)) {
      armSyncColors();
    }
    if (dirty.has(DIRTY.SELECTED)) {
      const joint = store.modelTransmissions.groups
        .find((g: any) => g.name === store.selectedGroup)?.joints[0] || null;
      setSelected(joint);
    }
  };
  render(new Set([DIRTY.MOTION, DIRTY.STATUS]));
  return {
    render,
    setActive(active: boolean) {
      panelActive = active;
      setRunning(active && !document.hidden);
      if (active) {
        renderRequested = true;
        resize();
      }
    },
    unmount() {
      panelActive = false;
      setRunning(false);
      rootEl.removeEventListener('click', onCamClick);
      disposeScene();
      window.removeEventListener('resize', onWindowResize);
      sizeObserver.disconnect();
      document.removeEventListener('visibilitychange', onVisibilityChange);
      ready = false;
    },
  };
}
