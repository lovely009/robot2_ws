// 臂视图面板：URDF → three.js，关节角由 store.motors 的 motion 帧驱动。
//
// 本模块**只做显示**：不订阅、不发布任何话题（core/ros.js 写入 store，这里读），
// 因此它坏掉也只是看不见臂，不影响任何控制链路。
//
// 依赖全部 vendor 在 web/vendor/（three 0.160.1 + urdf-loader 0.13.1），
// 由 index.html 的原生 import map 解析，无构建步骤。
//
// 几何来源：几何体不在 web/ 里（约 70 MB STL）——static_server 把
// robot_arm_gazebo 的 share 目录挂到 /robot_arm_gazebo，URDF 里的
// package://robot_arm_gazebo/... 经 loader.packages 映射成 /robot_arm_gazebo/...。
//
// 关节小球直接挂在 URDFJoint 下（随臂运动），点击选中与状态着色都在这里。
// 与重构前的 arm3d.js 相比：去掉 window.Arm3D 全局桥（改由 mount 直接调用），
// 新增 unmount 的资源释放（切屏会反复挂载，不释放会一路涨显存）。

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from '../../vendor/URDFLoader.js';
import { store, DIRTY, markDirty, JOINT_OF_MOTOR, MODEL, targetOf } from '../core/store.js';

// URDF 是 **Z-up**（ROS 约定），three.js 默认 Y-up——不设这一行整条臂会"躺着"。
// urdf-loader 自己不管这件事：它只把 box/cylinder 这类**图元**转 90° 适配 Y-up，
// 网格（我们的 STL）原样保留，所以 Z-up 的网格在 Y-up 场景里就是横的。
// 必须在创建任何 Object3D（相机、控制器）之前设：PerspectiveCamera 的 up 与
// OrbitControls 的旋转轴都是在构造时从 Object3D.DEFAULT_UP 取的。
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const MARKER_R = 0.022;

let renderer = null;
let scene = null;
let camera = null;
let controls = null;
let robot = null;
let ghost = null;
let root = null;
const markers = new Map();   // 关节名 -> marker Mesh
const picked = [];           // 供射线拾取的 marker 列表
let selected = null;
let onSelect = null;
let ghostMat = null;         // 幽灵共用一份半透明材质（几何与实体共享）

function tick() {
  controls.update();
  renderer.render(scene, camera);
}

export function init(canvas, opts) {
  const { urdfUrl, packages = {}, onProgress, onReady, onError } = opts;
  onSelect = opts.onSelect || null;

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1120);

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  camera.position.set(1.1, -1.1, 0.8);   // 载入完成前先给个 Z-up 的合理视角

  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 0.25);

  const hemi = new THREE.HemisphereLight(0xdfe9ff, 0x1b2438, 2.0);
  hemi.position.set(0, 0, 1);          // Z-up：天光从 +Z 来
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(2, -2, 3);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.9);
  fill.position.set(-2, 2, -1.5);
  scene.add(fill);

  // GridHelper 默认铺在 XZ 平面（Y-up），Z-up 场景里要转 90° 才躺在地上。
  const grid = new THREE.GridHelper(2, 20, 0x2a3550, 0x1b2438);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  root = new THREE.Group();
  scene.add(root);

  ghostMat = new THREE.MeshBasicMaterial({
    color: 0x4da3ff, transparent: true, opacity: 0.18, depthWrite: false,
  });

  canvas.addEventListener('pointerdown', onPointerDown);

  const loader = new URDFLoader();
  loader.packages = packages;      // { 包名: 路径前缀 } —— package:// 的解析表
  loader.load(urdfUrl, (r) => {
    robot = r;
    // 这臂没有关节限位动画，包围盒按几何算；关掉剔除免得绕到背后就整段消失。
    robot.traverse((o) => { if (o.isMesh) { o.frustumCulled = false; } });
    root.add(robot);

    addMarkers(robot);

    ghost = robot.clone();
    ghost.traverse((o) => { if (o.isMesh) { o.material = ghostMat; } });
    ghost.visible = false;
    root.add(ghost);

    frameCamera(robot);
    resize();
    if (onReady) { onReady(Object.keys(robot.joints || {})); }
  }, onProgress, (err) => { if (onError) { onError(err); } });

  renderer.setAnimationLoop(tick);
}

// 每个关节挂一颗小球：随臂运动，兼作点击靶与状态色标。
function addMarkers(r) {
  for (const [name, joint] of Object.entries(r.joints || {})) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(MARKER_R, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0x7e8bab }));
    m.userData.jointName = name;
    joint.add(m);
    markers.set(name, m);
    picked.push(m);
  }
}

function frameCamera(r) {
  const box = new THREE.Box3().setFromObject(r);
  if (box.isEmpty()) { return; }
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  const d = size * 1.15;
  controls.target.copy(c);
  // Z-up：从右前上方看（+X 侧、−Y 前方、抬高 +Z）。
  camera.position.set(c.x + d * 0.75, c.y - d * 0.75, c.z + d * 0.5);
  camera.near = d / 100;
  camera.far = d * 20;
  camera.updateProjectionMatrix();
  controls.update();
}

function onPointerDown(ev) {
  if (!onSelect || !robot) { return; }
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    -((ev.clientY - rect.top) / rect.height) * 2 + 1);
  const ray = new THREE.Raycaster();
  ray.setFromCamera(ndc, camera);
  const hits = ray.intersectObjects(picked, false);
  if (hits.length) { onSelect(hits[0].object.userData.jointName); }
}

function applyValues(target, angles) {
  if (!target || !angles) { return; }
  for (const [name, v] of Object.entries(angles)) {
    if (Number.isFinite(v)) { target.setJointValue(name, v); }
  }
}

export function setJoints(angles) { applyValues(robot, angles); }

// 目标幽灵：angles 为 null 时隐藏（没有在发的目标就不该凭空画一个出来）。
export function setTargets(angles) {
  if (!ghost) { return; }
  if (!angles) { ghost.visible = false; return; }
  applyValues(ghost, angles);
  ghost.visible = true;
}

// colors: { 关节名: '#rrggbb' }；缺省回落灰色。
export function setJointColors(colors) {
  for (const [name, m] of markers) {
    m.material.color.set((colors && colors[name]) || '#7e8bab');
  }
}

export function setSelected(name) {
  if (selected && markers.has(selected)) {
    markers.get(selected).scale.setScalar(1);
  }
  selected = name;
  if (selected && markers.has(selected)) {
    markers.get(selected).scale.setScalar(1.9);
  }
}

export function resize() {
  if (!renderer) { return; }
  const c = renderer.domElement;
  const w = c.clientWidth || 1;
  const h = c.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

// 页面隐藏时停掉渲染循环（WebGL 在后台标签页仍会跑满帧）。
export function setRunning(on) {
  if (!renderer) { return; }
  renderer.setAnimationLoop(on ? tick : null);
}

// 监听器具名化：旧实现是内联箭头函数，摘不掉；面板化后每次 mount 都注册一遍，
// 旧监听器仍引用已释放的 renderer —— 必须在 unmount 里成对摘掉。
const onWindowResize = () => resize();
const onVisibilityChange = () => setRunning(!document.hidden);

// 释放 three.js 资源。切屏会反复 mount/unmount，不释放会一路涨显存。
// 几何与材质在幽灵与实体之间共享（ghostMat 是共用材质），故先收集再释放，避免重复。
function disposeScene() {
  window.removeEventListener('resize', onWindowResize);
  document.removeEventListener('visibilitychange', onVisibilityChange);
  if (!scene) return;
  const geoms = new Set();
  const mats = new Set();
  scene.traverse((o) => {
    if (o.geometry) geoms.add(o.geometry);
    if (o.material) {
      for (const m of (Array.isArray(o.material) ? o.material : [o.material])) mats.add(m);
    }
  });
  for (const g of geoms) g.dispose();
  for (const m of mats) {
    for (const k of ['map', 'normalMap', 'roughnessMap', 'metalnessMap', 'alphaMap']) {
      if (m[k]) m[k].dispose();     // STL 没有贴图，但 ColladaLoader 拉进的 TGA 会有
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

// ---------- 同步：store → three.js ----------
// 关节角来源是**固件的 motion_states**，不是 /joint_states。理由：页面本来就在
// 订阅它，字段就是关节侧 rad；而 /joint_states 要额外起 firmware_topic_bridge，
// 且在 Gazebo 台架上会被 gz 自己的 joint_state_broadcaster 抢同名话题。
function armSyncJoints() {
  const angles = {};
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const mo = (store.motors.get(Number(idStr)) || {}).motion;
    if (mo && Number.isFinite(mo.position)) angles[joint] = mo.position;
  }
  setJoints(angles);
}

function armSyncGhost() {
  const angles = {};
  let any = false;
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const id = Number(idStr);
    const mo = (store.motors.get(id) || {}).motion;
    // 轨迹在播期间目标是固件插值出来的，页面无从得知（固件不发设定点）——
    // 宁可不画，也不画一个错的目标。target_source: 0=无 1=快照 2=轨迹
    if (mo && mo.target_source === 2) continue;
    // 幽灵 / 跟随误差用的「目标」= 页面最后一次真正发出去的目标位置（history[0]）。
    // 这不是固件的设定点——固件会钳位、软停、整包拒收，那些结论页面看不到，
    // 所以图例与提示里都写明了出处，不冒充固件真值。
    const v = targetOf(id);
    if (Number.isFinite(v)) { angles[joint] = v; any = true; }
  }
  setTargets(any ? angles : null);
}

function armSyncColors() {
  const colors = {};
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const r = store.motors.get(Number(idStr)) || {};
    const st = r.status || {}, t = r.topo || {};
    const online = st.online !== undefined ? st.online : t.online;
    const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
    const fault = st.unified_fault_code || 0;
    colors[joint] = !online ? '#7e8bab'
      : ((fault || lc === 6 || lc === 7) ? '#ff6b6b'
        : (lc === 4 ? '#37d67a' : '#f5b94a'));
  }
  setJointColors(colors);
}

export function mount(rootEl, ctx) {
  const stage = rootEl.querySelector('.stage') || rootEl;
  const canvas = document.createElement('canvas');
  canvas.className = 'arm3d';
  const hud = document.createElement('div');
  hud.className = 'hud';
  hud.textContent = '臂视图加载中…';
  // canvas 与 hud 由 JS 建（canvas 必须真元素、hud 要随加载状态改），
  // 图例在面板模板里（DOM 单一来源），插到 canvas/hud 之后保持旧层级。
  stage.prepend(hud);
  stage.prepend(canvas);

  let ready = false;
  const pickJoint = (jointName) => {
    // 臂上小球 → 电机 id（JOINT_OF_MOTOR 反查），点击与左栏选中双向联动
    const entry = Object.entries(JOINT_OF_MOTOR).find(([, j]) => j === jointName);
    if (entry) ctx.ui.selectMotor(parseInt(entry[0], 10));
  };

  init(canvas, {
    urdfUrl: `/robot_arm_gazebo/models/${MODEL}/urdf/${MODEL}.urdf`,
    // package://robot_arm_gazebo/... → static_server 挂在根部的 share 目录
    packages: {'robot_arm_gazebo': '/robot_arm_gazebo'},
    onSelect: pickJoint,
    // LoadingManager 的回调签名是 (url, loaded, total)；不保证一定拿到 total。
    onProgress: (url, loaded, total) => {
      if (Number.isFinite(loaded) && Number.isFinite(total) && total > 0) {
        hud.textContent = `臂视图加载中 ${Math.round(loaded / total * 100)}%`;
      }
    },
    onReady: (joints) => {
      // 没进映射表的 URDF 关节要点名——否则「这根不动」会被当成故障去排查。
      const mapped = Object.values(JOINT_OF_MOTOR);
      const extra = joints.filter((j) => !mapped.includes(j));
      hud.textContent = `臂视图就绪 · ${joints.length} 个关节`
        + (extra.length ? ` · 未映射：${extra.join(' ')}` : '')
        + ' · STL 约 70 MB（首次慢，之后走浏览器缓存）';
      ready = true;
      // 首帧渲染早于 URDF 加载完成（markers 还不存在），选中态在这里补一次；
      // 否则切屏返回后小球高亮要等到下一次用户点击才会恢复。
      setSelected(JOINT_OF_MOTOR[store.selectedId] || null);
      markDirty(DIRTY.MOTION);
      setTimeout(() => { hud.style.opacity = '0.3'; }, 5000);
    },
    onError: (err) => {
      hud.className = 'hud err';
      hud.textContent = '臂视图加载失败：' + (err && err.message ? err.message : err)
        + '（查 static_server 是否挂载 /robot_arm_gazebo、static server 日志）';
    },
  });
  window.addEventListener('resize', onWindowResize);
  document.addEventListener('visibilitychange', onVisibilityChange);
  setRunning(!document.hidden);     // 后台标签页不空转渲染

  const render = (dirty) => {
    if (!ready) return;
    if (dirty.has(DIRTY.MOTION)) { armSyncJoints(); armSyncGhost(); }
    if (dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY)) {
      armSyncColors();
    }
    if (dirty.has(DIRTY.SELECTED)) setSelected(JOINT_OF_MOTOR[store.selectedId] || null);
  };
  render(new Set([DIRTY.MOTION, DIRTY.STATUS]));
  return {
    render,
    unmount() {
      // three.js 资源必须显式释放：切屏会反复 mount/unmount，不释放会一路涨显存
      setRunning(false);
      disposeScene();
      ready = false;
    },
  };
}
