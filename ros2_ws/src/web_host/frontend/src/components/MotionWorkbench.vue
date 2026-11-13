<script setup lang="ts">
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { Upload, Play, Pause, RotateCcw, Download, Check, ArrowRight } from '@lucide/vue';

type Mapping = { mode?: undefined; torso: string; source_to_robot: number[][];
  arms: Record<string, { source: string[]; robot: string[] }> };
type ShoulderMapping = { mode: 'shoulders12'; source_to_robot: number[][]; source: string; joints: string[] };
type GenericMapping = { mode: 'generic'; source_root: string; reference_frame: number;
  source_to_robot: number[][]; limit_policy: 'urdf' | 'preview';
  targets: { source: string; target: string; confidence?: string; reason?: string }[] };
type Job = { id: string; state: 'mapping' | 'running' | 'done' | 'failed'; progress: number;
  bvh_name: string; urdf_name: string; robot_name: string; frames: number; frame_time: number;
  joint_count: number; source_joints: string[]; robot_links: string[]; mapping: Mapping | ShoulderMapping | GenericMapping | null;
  constraint_source?: string; velocity_limit_rad_s?: number;
  up_axis?: 'y_up' | 'z_up'; position_mode?: string;
  warnings?: string[]; mapping_origin?: string; mapping_saved?: boolean;
  result_cached?: boolean; result_saved?: boolean; cache_warning?: string;
  summary?: { mechanical_limits_enforced?: boolean; simulation_limits_enforced?: boolean;
    covered_joints?: string[]; unmapped_joints?: string[];
    orientation_max_deg?: number; direction_max_deg?: number }; error?: string };
type Preview = { times: number[]; duration: number };
const props = defineProps<{ currentModel: string }>();
const bvh = ref<File>();
const urdf = ref<File>();
const sources = ref<string[]>([]);
const models = ref<string[]>([]);
const bvhSource = ref('');
const robotModel = ref(props.currentModel);
const catalogLoading = ref(true);
const modelChosen = ref(false);
const upAxis = ref('auto');
const scale = ref(.01);
const job = ref<Job>();
const mapping = ref<Mapping>();
const genericMapping = ref<GenericMapping>();
const rememberMapping = ref(true);
const reviewMapping = ref(false);
const alignment = ref([0, 0, 0]);
const robotAxes = ref('cad');
const error = ref('');
const uploading = ref(false);
const restoring = ref(false);
const canvas = ref<HTMLCanvasElement>();
const preview = ref<Preview>();
const playing = ref(false);
const modelStatus = ref('机器人骨架');
const time = ref(0);
const busy = computed(() => uploading.value || restoring.value || job.value?.state === 'running');
const canSubmit = computed(() => !busy.value && !catalogLoading.value
  && !!(bvhSource.value || bvh.value) && !!(robotModel.value || urdf.value));
const ready = computed(() => job.value?.state === 'done' && !!preview.value);
const seconds = (value: number) => `${value.toFixed(2)} s`;
let display: { show(index: number): void; dispose(): void } | undefined;
let poll: ReturnType<typeof setTimeout>;
let animation = 0;
let disposed = false;
let lastTick = 0;
const abort = new AbortController();
const base = (id: string) => `/api/motion/jobs/${id}`;

watch(bvhSource, name => {
  bvh.value = undefined;
  if (name) sessionStorage.setItem('robot2.motionSource', name);
  else sessionStorage.removeItem('robot2.motionSource');
});
watch(robotModel, () => { urdf.value = undefined; });
watch(() => props.currentModel, name => {
  if (!modelChosen.value && models.value.includes(name)) robotModel.value = name;
});

async function loadCatalogs() {
  const results = await Promise.allSettled([request('/api/motion/sources'), request('/api/models')]);
  if (disposed) return;
  const [actions, robots] = results;
  if (actions.status === 'fulfilled' && Array.isArray(actions.value.sources)) {
    sources.value = actions.value.sources.filter((name: unknown): name is string => typeof name === 'string');
    const saved = sessionStorage.getItem('robot2.motionSource');
    bvhSource.value = sources.value.find(name => name === saved) || sources.value[0] || '';
  }
  if (robots.status === 'fulfilled' && Array.isArray(robots.value.models)) {
    models.value = robots.value.models.filter((name: unknown): name is string => typeof name === 'string');
    robotModel.value = models.value.includes(props.currentModel) ? props.currentModel : models.value[0] || '';
  } else robotModel.value = '';
  if (results.some(result => result.status === 'rejected')) error.value = '库列表加载失败，可重试刷新或导入本地文件';
  catalogLoading.value = false;
}

async function request(url: string, body?: unknown) {
  const response = await fetch(url, { signal: abort.signal,
    ...(body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }) });
  const result = await response.json();
  if (!response.ok) throw Object.assign(Error(result.error || '动作请求失败'), { status: response.status });
  return result;
}

function choose(event: Event, type: 'bvh' | 'urdf') {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (type === 'bvh') bvh.value = file; else urdf.value = file;
}

function showFrame() {
  if (!preview.value) return;
  const times = preview.value.times;
  let index = times.findIndex(t => t > time.value);
  display?.show(index < 0 ? times.length - 1 : Math.max(0, index - 1));
}

function tick(now: number) {
  if (playing.value && preview.value) {
    time.value = Math.min(preview.value.duration, time.value + (now - lastTick) / 1000);
    showFrame();
    if (time.value >= preview.value.duration) playing.value = false;
  }
  lastTick = now;
  animation = requestAnimationFrame(tick);
}

async function accept(current: Job) {
  if (disposed) return;
  job.value = current;
  if (restoring.value && models.value.includes(current.robot_name)) {
    robotModel.value = current.robot_name;
    modelChosen.value = true;
  }
  sessionStorage.setItem('robot2.motionJob', current.id);
  if (current.state === 'mapping') {
    if (current.mapping?.mode === 'generic') {
      genericMapping.value = structuredClone(current.mapping);
      mapping.value = undefined;
      const m = current.mapping.source_to_robot;
      alignment.value = [Math.atan2(m[2][1], m[2][2]), Math.atan2(-m[2][0], Math.hypot(m[0][0], m[1][0])),
        Math.atan2(m[1][0], m[0][0])].map(value => value * 180 / Math.PI);
    } else {
      genericMapping.value = undefined;
      mapping.value = (current.mapping && 'arms' in current.mapping ? current.mapping : undefined) || { torso: current.source_joints.find(n => /spine3|chest|torso/i.test(n)) || '',
      source_to_robot: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
      arms: Object.fromEntries(['right', 'left'].map(side => [side, {
        source: ['UpperArm', 'LowerArm', 'Hand'].map(part => current.source_joints.find(n => n.toLowerCase() === `${side}${part}`.toLowerCase()) || ''),
        robot: ['', '', ''],
      }])) };
    }
  } else if (current.state === 'running') {
    poll = setTimeout(() => void refresh(), 1000);
  } else if (current.state === 'done') {
    const data = await request(`${base(current.id)}/preview.json`);
    if (disposed) return;
    preview.value = data;
    await nextTick();
    const path = '/js/motion_preview.js';
    const module = await import(/* @vite-ignore */ path);
    if (disposed) return;
    display?.dispose();
    display = module.createMotionPreview(canvas.value!, data, {
      onModelStatus: (status: string) => { modelStatus.value = status; },
    });
    time.value = 0;
    playing.value = true;
    lastTick = performance.now();
  } else error.value = current.error || '转换失败，请检查文件与映射';
}

async function refresh() {
  if (!job.value || disposed) return;
  try { await accept(await request(base(job.value.id))); }
  catch (e) {
    if (disposed) return;
    if ((e as { status?: number }).status === 404 && job.value) {
      job.value.state = 'failed';
      sessionStorage.removeItem('robot2.motionJob');
    }
    error.value = String(e);
  }
}

async function submit() {
  if (!canSubmit.value) return;
  const sourceName = bvhSource.value;
  const modelName = robotModel.value;
  const bvhFile = bvh.value;
  const urdfFile = urdf.value;
  error.value = '';
  if ((bvhFile?.size || 0) + (urdfFile?.size || 0) > 20 * 1024 * 1024) {
    error.value = '两份文件合计请控制在 20 MB 内'; return;
  }
  clearTimeout(poll);
  playing.value = false;
  time.value = 0;
  preview.value = undefined;
  display?.dispose(); display = undefined;
  job.value = undefined;
  mapping.value = undefined;
  genericMapping.value = undefined;
  sessionStorage.removeItem('robot2.motionJob');
  uploading.value = true;
  try {
    let urdfText: string;
    if (modelName) {
      const name = encodeURIComponent(modelName);
      const response = await fetch(`/robot_sim/models/${name}/urdf/${name}.urdf`, { signal: abort.signal });
      if (!response.ok) throw Error(`模型 ${modelName} 的 URDF 加载失败`);
      urdfText = await response.text();
    } else urdfText = await urdfFile!.text();
    if ((new TextEncoder().encode(urdfText).length + (bvhFile?.size || 0)) > 20 * 1024 * 1024)
      throw Error('两份文件合计请控制在 20 MB 内');
    await accept(await request('/api/motion/jobs', {
      retargeting: 'generic', review_mapping: reviewMapping.value,
      ...(modelName ? {model: modelName} : {}),
      ...(sourceName ? { bvh_source: sourceName } : { bvh_name: bvhFile!.name, bvh_text: await bvhFile!.text() }),
      urdf_name: modelName ? `${modelName}.urdf` : urdfFile!.name,
      urdf_text: urdfText, up_axis: upAxis.value, scale: scale.value }));
  } catch (e) { if (!disposed) error.value = String(e); }
  finally { uploading.value = false; }
}

async function convert() {
  if (!job.value || (!mapping.value && !genericMapping.value)) return;
  error.value = ''; uploading.value = true;
  if (genericMapping.value) {
    const [r, p, y] = alignment.value.map(value => value * Math.PI / 180);
    const [sr, sp, sy] = [r, p, y].map(Math.sin), [cr, cp, cy] = [r, p, y].map(Math.cos);
    genericMapping.value.source_to_robot = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]];
  } else if (mapping.value) mapping.value.source_to_robot = robotAxes.value === 'cad'
    ? [[0, -1, 0], [1, 0, 0], [0, 0, 1]] : [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  try { await accept(await request(`${base(job.value.id)}/convert`, {
    mapping: genericMapping.value || mapping.value,
    remember_mapping: !!genericMapping.value && rememberMapping.value,
  })); }
  catch (e) { if (!disposed) error.value = String(e); }
  finally { uploading.value = false; }
}

function togglePlayback() {
  if (preview.value && time.value >= preview.value.duration) time.value = 0;
  lastTick = performance.now(); playing.value = !playing.value; showFrame();
}
function reset() { playing.value = false; time.value = 0; showFrame(); }

onMounted(async () => {
  animation = requestAnimationFrame(tick);
  const id = sessionStorage.getItem('robot2.motionJob');
  restoring.value = !!id;
  await loadCatalogs();
  if (!id) return;
  try { await accept(await request(base(id))); }
  catch { if (!disposed) { sessionStorage.removeItem('robot2.motionJob'); error.value = '上次临时任务已过期，请重新导入'; } }
  finally { restoring.value = false; }
});
onBeforeUnmount(() => {
  disposed = true; abort.abort(); clearTimeout(poll); cancelAnimationFrame(animation); display?.dispose();
});
</script>

<template>
  <div class="motion-workbench">
    <section class="motion-import surface">
      <div class="panel-title"><span>动作来源</span><Upload :size="14" /></div>
      <div class="motion-import-body">
        <div class="motion-file"><span class="eyebrow">01 / 人体动作</span><strong>动作库 BVH</strong>
          <select v-model="bvhSource" aria-label="动作库 BVH" :disabled="busy || catalogLoading">
            <option v-for="name in sources" :key="name" :value="name">{{ name }}</option>
            <option value="">导入本地 BVH…</option>
          </select>
          <input v-if="!bvhSource" type="file" accept=".bvh,.BHV,.bhv" aria-label="导入 BVH 文件" :disabled="busy || catalogLoading" @change="choose($event, 'bvh')" />
        </div>
        <div class="motion-file"><span class="eyebrow">02 / 目标机器人</span><strong>模型库</strong>
          <select v-model="robotModel" aria-label="目标机器人模型" :disabled="busy || catalogLoading" @change="modelChosen = true">
            <option v-for="name in models" :key="name" :value="name">{{ name }}</option>
            <option value="">导入本地 URDF…</option>
          </select>
          <input v-if="!robotModel" type="file" accept=".urdf,.xml" aria-label="导入 URDF 文件" :disabled="busy || catalogLoading" @change="choose($event, 'urdf')" />
        </div>
        <div class="motion-source-options">
          <label>BVH 坐标<select v-model="upAxis" :disabled="busy" aria-label="BVH 坐标"><option value="auto">自动识别</option><option value="y_up">Y 向上</option><option value="z_up">Z 向上（ROS）</option></select></label>
          <label>长度单位<select v-model.number="scale" :disabled="busy" aria-label="BVH 长度单位"><option :value=".01">厘米 cm</option><option :value="1">米 m</option><option :value=".001">毫米 mm</option></select></label>
        </div>
        <p class="motion-hint">{{ catalogLoading ? '正在读取动作库与模型库…' : '自动识别人体与机器人对应关系；疑点首次确认，已记住的相同组合自动转换。输出仅用于预览。' }}</p>
        <label class="motion-hint"><input v-model="reviewMapping" type="checkbox" :disabled="busy" />重新确认已记住的映射</label>
        <button class="motion-primary" :disabled="!canSubmit" @click="submit"><ArrowRight :size="15" />{{ busy ? '处理中…' : '导入并转换' }}</button>
        <div v-if="job" class="motion-metadata">
          <span>目标模型 <strong>{{ job.robot_name }}</strong></span>
          <span>原始动作 <strong>{{ job.frames.toLocaleString() }} 帧 · {{ (1 / job.frame_time).toFixed(1) }} Hz</strong></span>
          <span v-if="job.up_axis">BVH 格式 <strong>{{ job.up_axis === 'z_up' ? 'Z 向上' : 'Y 向上' }}{{ job.position_mode === 'local_translation' ? ' · 已识别骨段位置通道' : '' }}</strong></span>
          <span>机器人 <strong>{{ job.joint_count }} 个关节</strong></span>
          <span v-if="job.constraint_source === 'simulation'">预览范围 <strong>仿真几何与耦合限位 · {{ job.velocity_limit_rad_s?.toFixed(2) }} rad/s</strong></span>
          <span v-if="job.mapping_origin">映射来源 <strong>{{ job.mapping_origin === 'saved' ? '复用已确认映射' : job.mapping_origin === 'builtin' ? '已支持的双臂或肩关节映射' : job.mapping_saved ? '已确认并记住' : job.state === 'mapping' ? '候选待确认' : '本次已确认' }}</strong></span>
          <span v-if="job.mapping?.mode === 'shoulders12'">转换范围 <strong>右肩 → joint_link1、joint_link2；其余关节保持中立位，保留原帧与时间</strong></span>
        </div>
      </div>
    </section>

    <section class="motion-preview surface">
      <div class="panel-title"><span>动作预览</span><span class="micro">人体骨架 <i class="motion-key human" /> / 机器人 <i class="motion-key robot" /></span></div>
      <div class="motion-stage">
        <canvas v-show="ready" ref="canvas" aria-label="人体与机器人动作三维预览" />
        <span v-if="ready" class="motion-model-status" role="status" aria-label="机器人模型状态">{{ modelStatus }}</span>
        <div v-if="!ready" class="motion-placeholder">
          <div class="motion-stage-icon"><Upload v-if="!job" :size="30" /><Check v-else-if="job.state === 'done'" :size="30" /><ArrowRight v-else :size="30" /></div>
          <strong>{{ job?.state === 'running' ? '正在转换动作' : job?.state === 'mapping' ? '确认骨架对应关系' : job?.state === 'failed' ? '转换未完成' : '从一份动作开始' }}</strong>
          <span>{{ job?.state === 'running' ? '解析人体动作 → IK 求解 → 生成机器人预览' : '选择动作与目标模型，在这里查看转换结果' }}</span>
          <progress v-if="job?.state === 'running'" :value="job.progress" max="100" aria-label="动作转换进度" />
          <span v-if="job?.state === 'running'">{{ job.progress }}%</span>
        </div>
      </div>
      <div class="motion-transport">
        <button :disabled="!ready" :aria-label="playing ? '暂停预览' : '播放预览'" @click="togglePlayback"><Pause v-if="playing" :size="17" /><Play v-else :size="17" /></button>
        <button :disabled="!ready" aria-label="回到动作起点" @click="reset"><RotateCcw :size="15" /></button>
        <input v-model.number="time" type="range" min="0" :max="preview?.duration || 0" step=".001" :disabled="!ready" aria-label="动作播放进度" @input="playing = false; showFrame()" />
        <span class="micro motion-time">{{ seconds(time) }} / {{ seconds(preview?.duration || 0) }}</span>
      </div>
    </section>

    <section v-if="job?.state === 'mapping' && genericMapping" class="motion-mapping surface" aria-label="通用映射确认">
      <div class="panel-title">确认人体与机器人对应关系</div>
      <div class="motion-mapping-body">
        <label>人体根参考<select v-model="genericMapping.source_root" aria-label="人体根参考"><option v-for="name in job.source_joints" :key="name">{{ name }}</option></select></label>
        <label>参考帧<input v-model.number="genericMapping.reference_frame" type="number" min="0" :max="job.frames - 1" step="1" aria-label="参考帧" /></label>
        <div class="motion-alignment"><label v-for="(axis, i) in ['X', 'Y', 'Z']" :key="axis">对齐 {{ axis }}（°）<input v-model.number="alignment[i]" type="number" step="1" :aria-label="`对齐 ${axis} 角度`" /></label></div>
        <label>限位策略<select v-model="genericMapping.limit_policy" aria-label="限位策略"><option value="urdf">{{ job.constraint_source === 'simulation' ? '按仿真位置、速度和耦合范围' : '按 URDF 位置和速度限位' }}</option><option v-if="job.constraint_source !== 'simulation'" value="preview">仅几何预览：忽略缺失限位</option></select></label>
        <details v-if="job.warnings?.length" class="motion-mapping-notes"><summary>模型有 {{ job.warnings.length }} 项需核对</summary><ul><li v-for="warning in job.warnings" :key="warning">{{ warning }}</li></ul></details>
        <div class="motion-targets"><div v-for="target in genericMapping.targets" :key="target.target" class="motion-target-row">
          <label :for="`mapping-${target.target}`">{{ target.target }}<small>{{ target.reason || '已确认的对应关系' }}</small></label>
          <select :id="`mapping-${target.target}`" v-model="target.source" :aria-label="`${target.target} 人体对应`"><option value="">不设置目标</option><option v-for="name in job.source_joints" :key="name">{{ name }}</option></select>
        </div></div>
        <label class="motion-remember"><input v-model="rememberMapping" type="checkbox" aria-label="记住映射" />记住本次映射；模型、骨架或参考姿势改变后重新确认</label>
        <button class="motion-primary" :disabled="busy || !genericMapping.targets.some(t => t.source) || !genericMapping.source_root || !alignment.every(Number.isFinite)" @click="convert">确认映射并转换</button>
        <p class="motion-hint motion-mapping-notes">同一部位只映射到该组最后一个连杆，中间轴留空仍参与求解。{{ job.constraint_source === 'simulation' ? '按人体骨段方向拟合姿态，第一帧作为初始姿态，不要求从机器人零位起步。' : '参考帧对应机器人的中立姿势。' }}预览恢复人体根朝向；导出仍为固定基座关节动作，不包含平衡、碰撞或实机验收。</p>
      </div>
    </section>

    <section v-if="job?.state === 'mapping' && mapping" class="motion-mapping surface">
      <div class="panel-title">骨架对应关系</div>
      <div class="motion-mapping-body">
        <label>人体躯干<select v-model="mapping.torso" aria-label="人体躯干"><option value="">请选择</option><option v-for="n in job.source_joints" :key="n">{{ n }}</option></select></label>
        <label>机器人坐标<select v-model="robotAxes" aria-label="机器人坐标"><option value="cad">+Y 向前、+X 向右（当前 CAD）</option><option value="ros">+X 向前、+Y 向左（ROS）</option></select></label>
        <div v-for="(arm, side) in mapping.arms" :key="side" class="motion-arm-mapping">
          <strong>{{ side === 'right' ? '右臂' : '左臂' }}</strong>
          <div v-for="(part, i) in ['肩', '肘', '腕']" :key="part" class="motion-mapping-row">
            <span>{{ part }}</span><select v-model="arm.source[i]" :aria-label="`${side} 人体${part}`"><option value="">人体关节</option><option v-for="n in job.source_joints" :key="n">{{ n }}</option></select>
            <ArrowRight :size="13" /><select v-model="arm.robot[i]" :aria-label="`${side} 机器人${part}`"><option value="">机器人连杆</option><option v-for="n in job.robot_links" :key="n">{{ n }}</option></select>
          </div>
        </div>
        <button class="motion-primary" :disabled="busy || !mapping.torso || Object.values(mapping.arms).some(a => [...a.source, ...a.robot].some(n => !n))" @click="convert">确认映射并转换</button>
        <p class="motion-hint">请填齐躯干和两臂的肩、肘、腕。缺少这些骨骼的动作无法使用完整双臂映射。</p>
      </div>
    </section>

    <div v-if="error" class="motion-error" role="alert">{{ error }}<button v-if="job?.state === 'running'" @click="refresh">重试查询</button></div>
    <section v-if="job?.state === 'done'" class="motion-result surface" aria-label="动作转换结果">
      <div><Check :size="16" /><strong>转换完成</strong><span>{{ job.frames.toLocaleString() }} 帧 · {{ job.joint_count }} 个关节 · 可拖动进度查看动作</span></div>
      <a :href="`${base(job.id)}/motion.npz`" download><Download :size="14" />导出关节动作</a>
      <a :href="`${base(job.id)}/robot.urdf`" download>导出预览 URDF</a>
      <a :href="`${base(job.id)}/report.json`" download>转换报告</a>
      <span v-if="job.result_cached">已加载保存结果，无需重新转换</span>
      <span v-else-if="job.result_saved">结果已保存，相同输入下次直接复用</span>
      <span v-if="job.cache_warning">{{ job.cache_warning }}</span>
      <span v-if="job.summary?.covered_joints">覆盖 {{ job.summary.covered_joints.length }}/{{ job.joint_count }} 个关节 · {{ job.summary.simulation_limits_enforced ? '已检查仿真位置、速度和耦合范围' : job.summary.mechanical_limits_enforced ? '已检查位置与速度限位，仍仅供预览' : '几何预览，部分机械限位缺失' }}</span>
      <span v-if="job.summary?.direction_max_deg !== undefined">最大骨段方向误差 {{ job.summary.direction_max_deg.toFixed(1) }}°；详细目标误差见报告</span>
      <span v-else-if="job.summary?.orientation_max_deg !== undefined">最大朝向误差 {{ job.summary.orientation_max_deg.toFixed(1) }}°；详细目标误差见报告</span>
    </section>
    <p class="motion-footnote">机器人按 URDF 显示实体模型，缺失网格时保留骨架。双足预览按足底估计支撑高度，并保留源动作的腾空；这不是接触或平衡仿真。转换结果保存在本机；动作、映射、模型、约束和算法未变化时直接复用。页面临时保留最近 3 次导入，服务重启后再次导入也可复用已保存结果。</p>
  </div>
</template>

<style scoped>
.motion-workbench { display: grid; grid-template-columns: 290px minmax(0, 1fr); grid-auto-rows: max-content; gap: 14px; min-height: 0; flex: 1; overflow: auto; align-content: start; padding-right: 3px; }
.motion-import-body { display: grid; gap: 18px; padding: 18px; }
.motion-file { display: flex; flex-direction: column; gap: 9px; padding: 17px 14px; border: 1px dashed var(--line-2); border-radius: 6px; background: #141c20; min-width: 0; }
.motion-file strong { font-size: 14px; font-weight: 500; }
.motion-file > span:not(.eyebrow) { color: var(--muted); overflow-wrap: anywhere; }
.motion-file input { width: 100%; font-size: 11px; color: var(--muted); padding: 0; border: 0; background: none; }
.motion-file select { width: 100%; min-width: 0; }
.motion-file input::file-selector-button { background: var(--panel-2); border: 1px solid var(--line-2); border-radius: 4px; color: var(--text); padding: 6px 9px; cursor: pointer; margin-right: 7px; }
.motion-source-options { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.motion-source-options label, .motion-mapping-body > label { display: grid; gap: 7px; color: var(--muted); }
.motion-hint, .motion-footnote { color: var(--muted); font-size: 11px; line-height: 1.8; margin: 0; }
.motion-primary { justify-content: center; padding: 10px 12px; background: var(--accent); color: #18372c; border-radius: 4px; gap: 8px; font-weight: 600; }
.motion-metadata { display: grid; gap: 10px; border-top: 1px solid var(--line); padding-top: 15px; }
.motion-metadata span { display: flex; flex-direction: column; gap: 3px; color: var(--muted); font-size: 11px; }
.motion-metadata strong { font-size: 12px; color: var(--text); font-weight: 400; }
.motion-preview { display: flex; flex-direction: column; min-height: 510px; overflow: hidden; }
.motion-preview .panel-title > span:last-child { display: inline-flex; align-items: center; gap: 8px; }
.motion-key { display: inline-block; width: 7px; height: 7px; border-radius: 50%; }.human { background: #86bcea; }.robot { background: #85dfb8; }
.motion-stage { position: relative; flex: 1; min-height: 370px; background: #131b1f; }
.motion-stage canvas { position: absolute; width: 100%; height: 100%; display: block; }
.motion-model-status { position: absolute; top: 12px; left: 14px; padding: 4px 8px; color: var(--muted); background: #131b1fbb; border-radius: 4px; font-size: 11px; pointer-events: none; }
.motion-placeholder { position: absolute; inset: 0; display: flex; flex-direction: column; justify-content: center; align-items: center; gap: 15px; text-align: center; padding: 20px; color: var(--muted); }
.motion-stage-icon { display: grid; place-items: center; width: 68px; height: 68px; border: 1px solid var(--line); border-radius: 16px; color: var(--accent); }
.motion-placeholder strong { font-size: 17px; font-weight: 400; color: var(--text); }.motion-placeholder progress { width: 220px; accent-color: var(--accent); }
.motion-transport { display: flex; align-items: center; gap: 10px; padding: 13px; border-top: 1px solid var(--line); }
.motion-transport button { padding: 7px; }.motion-transport input[type=range] { flex: 1; min-width: 30px; padding: 0; accent-color: var(--accent); }.motion-time { white-space: nowrap; }
.motion-mapping, .motion-error, .motion-result, .motion-footnote { grid-column: 1 / -1; }
.motion-mapping-body { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; padding: 18px; }.motion-mapping-row { display: grid; grid-template-columns: 20px 1fr 14px 1fr; align-items: center; gap: 9px; margin-top: 12px; }.motion-mapping-row select { min-width: 0; width: 100%; }
.motion-targets, .motion-mapping-notes { grid-column: 1 / -1; }.motion-targets { max-height: 380px; overflow: auto; }.motion-target-row { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; align-items: center; padding: 9px 0; border-bottom: 1px solid var(--line); }.motion-target-row label { overflow-wrap: anywhere; }.motion-target-row small { display: block; color: var(--muted); margin-top: 4px; }.motion-target-row select { min-width: 0; width: 100%; }.motion-alignment { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }.motion-alignment input { min-width: 0; width: 100%; }.motion-remember { display: flex !important; align-items: center; }.motion-mapping-notes ul { max-height: 180px; overflow: auto; color: var(--warn); }
.motion-error { color: var(--err); padding: 12px; border: 1px solid #69463d; border-radius: 5px; overflow-wrap: anywhere; }.motion-error button { margin-left: 15px; text-decoration: underline; }
.motion-result { display: flex; flex-wrap: wrap; align-items: center; gap: 20px; padding: 15px 18px; }.motion-result > div { display: flex; align-items: center; flex-wrap: wrap; gap: 10px; margin-right: auto; color: var(--accent); }.motion-result span { color: var(--muted); }.motion-result a { display: inline-flex; align-items: center; gap: 7px; color: var(--accent); }
@media(max-width: 1000px) { .motion-workbench { grid-template-columns: 1fr; }.motion-import-body { grid-template-columns: 1fr 1fr; }.motion-mapping-body { grid-template-columns: 1fr; }.motion-preview { min-height: 460px; } }
</style>
