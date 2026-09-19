// 臂视图：URDF → three.js，关节角由 /joint_states 驱动。
//
// 本模块**只做显示**：不订阅、不发布任何话题（app.js 订阅后把角度喂进来），
// 因此它坏掉也只是看不见臂，不会影响任何控制链路。
//
// 依赖全部 vendor 在 web/vendor/（three 0.160.1 + urdf-loader 0.13.1），
// 由 index.html 的原生 import map 解析，无构建步骤。
//
// 几何来源：web/robot_arm_gazebo 是指向 ../../robot_arm_gazebo 的符号链接，
// 所以 URDF 里的 package://robot_arm_gazebo/... 相对页面原样可用（20 MB 的
// STL 因此不进 web/、也不进 git）。static server 的 SimpleHTTPRequestHandler
// 跟随符号链接。
//
// 关节小球直接挂在 URDFJoint 下（随臂运动），点击选中与状态着色都在这里。

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from './vendor/URDFLoader.js';

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

// app.js 是普通脚本，且在模块之前执行（模块默认 defer）——用全局 + 事件广播
// 交接，别用加载顺序赌。
window.Arm3D = {
  init, setJoints, setTargets, setJointColors, setSelected, resize, setRunning,
};
window.dispatchEvent(new Event('arm3d-ready'));
