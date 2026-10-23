<script setup lang="ts">
import { onMounted, onBeforeUnmount, ref, watch } from "vue";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader, { URDFRobot } from "urdf-loader";
import { Maximize2, RotateCcw, Layers, MousePointer2 } from "@lucide/vue";
import { demo, model, measured, preview_position, set_variant } from "../demo";

const host = ref<HTMLDivElement>();
const loading = ref(true),
  load_error = ref("");
let renderer: THREE.WebGLRenderer,
  scene: THREE.Scene,
  camera: THREE.PerspectiveCamera,
  controls: OrbitControls;
let robot: URDFRobot | null = null,
  ghost: URDFRobot | null = null;
let observer: ResizeObserver,
  generation = 0;
const picks: THREE.Mesh[] = [];
const markers = new Map<string, THREE.Mesh>();
const materials = new Set<THREE.Material>();
const geometries = new Set<THREE.BufferGeometry>();
function dispose_object(object: THREE.Object3D) {
  object.traverse((o) => {
    if (o instanceof THREE.Mesh) {
      geometries.add(o.geometry);
      (Array.isArray(o.material) ? o.material : [o.material]).forEach((m) =>
        materials.add(m),
      );
    }
  });
  for (const g of geometries) g.dispose();
  for (const m of materials) m.dispose();
  geometries.clear();
  materials.clear();
}
function frame() {
  if (!robot) return;
  const box = new THREE.Box3().setFromObject(robot),
    center = box.getCenter(new THREE.Vector3());
  const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.3);
  controls.target.copy(center);
  camera.position
    .copy(center)
    .add(new THREE.Vector3(size * 0.85, -size * 1.25, size * 0.5));
  camera.near = size / 100;
  camera.far = size * 30;
  camera.updateProjectionMatrix();
  controls.update();
}
function load() {
  if (!scene || !model.value) return;
  const gen = ++generation;
  if (robot) {
    scene.remove(robot);
    dispose_object(robot);
    robot = null;
  }
  if (ghost) {
    scene.remove(ghost);
    dispose_object(ghost);
    ghost = null;
  }
  picks.length = 0;
  markers.clear();
  loading.value = true;
  load_error.value = "";
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = { robot_sim: __ROBOT_SIM_ROOT__ };
  let candidate: URDFRobot | null = null;
  manager.onLoad = () => {
    if (gen !== generation || !candidate) {
      if (candidate) dispose_object(candidate);
      return;
    }
    robot = candidate;
    robot.traverse((o) => {
      if (o instanceof THREE.Mesh) {
        (Array.isArray(o.material) ? o.material : [o.material]).forEach((m) =>
          m.dispose(),
        );
        o.material = new THREE.MeshStandardMaterial({
          color: 0xcbd2d4,
          metalness: 0.48,
          roughness: 0.46,
        });
      }
    });
    ghost = robot.clone();
    ghost.traverse((o) => {
      if (o instanceof THREE.Mesh)
        o.material = new THREE.MeshBasicMaterial({
          color: 0x36dcbe,
          transparent: true,
          opacity: 0.22,
          depthWrite: false,
        });
    });
    scene.add(robot, ghost);
    for (const j of model.value?.joints || []) {
      const joint = robot.joints[j.name];
      if (!joint) continue;
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.014, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0x448d7a, depthTest: false }),
      );
      marker.renderOrder = 5;
      marker.userData.name = j.name;
      joint.add(marker);
      picks.push(marker);
      markers.set(j.name, marker);
    }
    loading.value = false;
    frame();
  };
  manager.onError = (url) => {
    if (gen === generation)
      load_error.value = `几何加载失败：${url.split("/").at(-1)}`;
  };
  loader.load(
    `${__ROBOT_SIM_ROOT__}models/${demo.model}/urdf/${demo.model}.urdf`,
    (r) => {
      candidate = r;
    },
    undefined,
    () => {
      if (gen === generation) {
        loading.value = false;
        load_error.value = "模型加载失败";
      }
    },
  );
}
function pick(event: PointerEvent) {
  const rect = renderer.domElement.getBoundingClientRect();
  const ray = new THREE.Raycaster();
  ray.setFromCamera(
    new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      (-(event.clientY - rect.top) / rect.height) * 2 + 1,
    ),
    camera,
  );
  const hit = ray.intersectObjects(picks)[0];
  if (hit) demo.selected = hit.object.userData.name;
}
function resize() {
  const el = host.value;
  if (!el || !renderer) return;
  renderer.setSize(el.clientWidth, el.clientHeight);
  camera.aspect = el.clientWidth / Math.max(el.clientHeight, 1);
  camera.updateProjectionMatrix();
}
onMounted(() => {
  if (!host.value) return;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  } catch {
    loading.value = false;
    load_error.value = "当前浏览器无法创建 WebGL 视图";
    return;
  }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.7));
  renderer.setClearColor(0x141c20, 1);
  host.value.appendChild(renderer.domElement);
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(34, 1, 0.01, 100);
  camera.up.set(0, 0, 1);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  const grid = new THREE.GridHelper(4, 40, 0x41504c, 0x27352f);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.02;
  scene.add(grid);
  scene.add(new THREE.HemisphereLight(0xe9fff8, 0x374841, 3));
  const light = new THREE.DirectionalLight(0xffedda, 4);
  light.position.set(2, -2, 4);
  scene.add(light);
  const rim = new THREE.DirectionalLight(0x90dddf, 2);
  rim.position.set(-2, 1, 1);
  scene.add(rim);
  observer = new ResizeObserver(resize);
  observer.observe(host.value);
  resize();
  renderer.domElement.addEventListener("pointerdown", pick);
  renderer.setAnimationLoop(() => {
    if (robot) {
      for (const j of model.value?.joints || []) {
        robot.setJointValue(j.name, measured(j));
        ghost?.setJointValue(j.name, preview_position(j));
        const marker = markers.get(j.name);
        if (marker) {
          (marker.material as THREE.MeshBasicMaterial).color.set(
            j.name === demo.selected ? 0x9affe0 : 0x418878,
          );
          marker.scale.setScalar(j.name === demo.selected ? 1.6 : 1);
        }
      }
    }
    if (ghost) ghost.visible = demo.show_ghost;
    controls.update();
    renderer.render(scene, camera);
  });
  load();
});
watch(() => demo.model, load);
onBeforeUnmount(() => {
  generation++;
  observer?.disconnect();
  if (!renderer) return;
  renderer.setAnimationLoop(null);
  renderer.domElement.removeEventListener("pointerdown", pick);
  controls?.dispose();
  scene?.traverse((o) => {
    if (o instanceof THREE.Mesh || o instanceof THREE.LineSegments) {
      o.geometry.dispose();
      const ms = Array.isArray(o.material) ? o.material : [o.material];
      ms.forEach((m) => m.dispose());
    }
  });
  renderer.dispose();
  renderer.domElement.remove();
});
</script>

<template>
  <section class="viewport surface">
    <div ref="host" class="three-host" />
    <div class="view-head">
      <span><i class="dot green" />机器人视图 <small>WORLD / Z-UP</small></span>
      <div class="button-group">
        <button
          class="icon-button"
          title="重置相机"
          aria-label="重置相机"
          @click="frame"
        >
          <RotateCcw :size="15" /></button
        ><button
          class="icon-button"
          title="目标叠影"
          aria-label="切换目标叠影"
          :class="{ active: demo.show_ghost }"
          @click="demo.show_ghost = !demo.show_ghost"
        >
          <Layers :size="15" /></button
        ><button
          class="icon-button"
          title="3D 优先布局"
          aria-label="3D 优先布局"
          @click="set_variant('B')"
        >
          <Maximize2 :size="15" />
        </button>
      </div>
    </div>
    <div class="view-legend">
      <span><i class="legend-real" />实测姿态（模拟）</span
      ><span v-if="demo.show_ghost"><i class="legend-target" />目标预览</span>
    </div>
    <div v-if="loading || load_error" class="view-loading">
      <span :class="{ spinner: loading && !load_error }" />{{
        load_error || "正在加载仓库模型…"
      }}
    </div>
    <div class="view-bottom">
      <span
        ><MousePointer2 :size="12" />拖拽旋转 · 滚轮缩放 · 点击关节点选中</span
      ><span class="mono">{{ model?.joints.length }} DOF</span>
    </div>
    <div class="axis">
      <span class="x">X</span><span class="y">Y</span><span class="z">Z</span>
    </div>
  </section>
</template>
