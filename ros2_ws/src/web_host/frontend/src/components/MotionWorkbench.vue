<script setup lang="ts">
import { computed, nextTick, onBeforeUnmount, onMounted, ref } from 'vue';
import { Upload, Play, Pause, RotateCcw, Download, Check, ArrowRight } from '@lucide/vue';

type Mapping = { torso: string; source_to_robot: number[][];
  arms: Record<string, { source: string[]; robot: string[] }> };
type Job = { id: string; state: 'mapping' | 'running' | 'done' | 'failed'; progress: number;
  bvh_name: string; urdf_name: string; robot_name: string; frames: number; frame_time: number;
  joint_count: number; source_joints: string[]; robot_links: string[]; mapping: Mapping | null; error?: string };
type Preview = { times: number[]; duration: number };
const bvh = ref<File>();
const urdf = ref<File>();
const upAxis = ref('y_up');
const scale = ref(.01);
const job = ref<Job>();
const mapping = ref<Mapping>();
const robotAxes = ref('cad');
const error = ref('');
const uploading = ref(false);
const canvas = ref<HTMLCanvasElement>();
const preview = ref<Preview>();
const playing = ref(false);
const modelStatus = ref('机器人骨架');
const time = ref(0);
const busy = computed(() => uploading.value || job.value?.state === 'running');
const ready = computed(() => job.value?.state === 'done' && !!preview.value);
const seconds = (value: number) => `${value.toFixed(2)} s`;
let display: { show(index: number): void; dispose(): void } | undefined;
let poll: ReturnType<typeof setTimeout>;
let animation = 0;
let disposed = false;
let lastTick = 0;
const abort = new AbortController();
const base = (id: string) => `/api/motion/jobs/${id}`;

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
  sessionStorage.setItem('robot2.motionJob', current.id);
  if (current.state === 'mapping') {
    mapping.value = current.mapping || { torso: current.source_joints.find(n => /spine3|chest|torso/i.test(n)) || '',
      source_to_robot: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
      arms: Object.fromEntries(['right', 'left'].map(side => [side, {
        source: ['UpperArm', 'LowerArm', 'Hand'].map(part => current.source_joints.find(n => n.toLowerCase() === `${side}${part}`.toLowerCase()) || ''),
        robot: ['', '', ''],
      }])) };
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
  if (!bvh.value || !urdf.value || busy.value) return;
  error.value = '';
  if (bvh.value.size + urdf.value.size > 20 * 1024 * 1024) {
    error.value = '两份文件合计请控制在 20 MB 内'; return;
  }
  clearTimeout(poll);
  playing.value = false;
  time.value = 0;
  preview.value = undefined;
  display?.dispose(); display = undefined;
  job.value = undefined;
  mapping.value = undefined;
  sessionStorage.removeItem('robot2.motionJob');
  uploading.value = true;
  try {
    await accept(await request('/api/motion/jobs', { bvh_name: bvh.value.name, urdf_name: urdf.value.name,
      bvh_text: await bvh.value.text(), urdf_text: await urdf.value.text(), up_axis: upAxis.value, scale: scale.value }));
  } catch (e) { if (!disposed) error.value = String(e); }
  finally { uploading.value = false; }
}

async function convert() {
  if (!job.value || !mapping.value) return;
  error.value = ''; uploading.value = true;
  mapping.value.source_to_robot = robotAxes.value === 'cad'
    ? [[0, -1, 0], [1, 0, 0], [0, 0, 1]] : [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  try { await accept(await request(`${base(job.value.id)}/convert`, { mapping: mapping.value })); }
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
  if (!id) return;
  try { await accept(await request(base(id))); }
  catch { if (!disposed) { sessionStorage.removeItem('robot2.motionJob'); error.value = '上次临时任务已过期，请重新导入'; } }
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
        <label class="motion-file"><span class="eyebrow">01 / 人体动作</span><strong>导入 BVH</strong>
          <span>{{ bvh?.name || job?.bvh_name || '选择本地 .bvh 文件' }}</span>
          <input type="file" accept=".bvh,.BHV,.bhv" aria-label="导入 BVH 文件" :disabled="busy" @change="choose($event, 'bvh')" />
        </label>
        <label class="motion-file"><span class="eyebrow">02 / 目标机器人</span><strong>导入 URDF</strong>
          <span>{{ urdf?.name || job?.urdf_name || '选择本地 .urdf 文件' }}</span>
          <input type="file" accept=".urdf,.xml" aria-label="导入 URDF 文件" :disabled="busy" @change="choose($event, 'urdf')" />
        </label>
        <div class="motion-source-options">
          <label>BVH 坐标<select v-model="upAxis" :disabled="busy" aria-label="BVH 坐标"><option value="y_up">Y 向上</option><option value="z_up">Z 向上（ROS）</option></select></label>
          <label>长度单位<select v-model.number="scale" :disabled="busy" aria-label="BVH 长度单位"><option :value=".01">厘米 cm</option><option :value="1">米 m</option><option :value=".001">毫米 mm</option></select></label>
        </div>
        <p class="motion-hint">首帧使用参考 T-pose。当前模型自动映射，其他模型可选择肩、肘、腕对应关系。</p>
        <button class="motion-primary" :disabled="busy || !bvh || !urdf" @click="submit"><ArrowRight :size="15" />{{ busy ? '处理中…' : '导入并转换' }}</button>
        <div v-if="job" class="motion-metadata">
          <span>目标模型 <strong>{{ job.robot_name }}</strong></span>
          <span>原始动作 <strong>{{ job.frames.toLocaleString() }} 帧 · {{ (1 / job.frame_time).toFixed(1) }} Hz</strong></span>
          <span>机器人 <strong>{{ job.joint_count }} 个关节</strong></span>
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
          <span>{{ job?.state === 'running' ? '解析人体动作 → IK 求解 → 生成机器人预览' : '导入 BVH 与 URDF，在这里查看转换结果' }}</span>
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
      </div>
    </section>

    <div v-if="error" class="motion-error" role="alert">{{ error }}<button v-if="job?.state === 'running'" @click="refresh">重试查询</button></div>
    <section v-if="job?.state === 'done'" class="motion-result surface" aria-label="动作转换结果">
      <div><Check :size="16" /><strong>转换完成</strong><span>{{ job.frames.toLocaleString() }} 帧 · {{ job.joint_count }} 个关节 · 可拖动进度查看动作</span></div>
      <a :href="`${base(job.id)}/motion.npz`" download><Download :size="14" />导出关节动作</a>
      <a :href="`${base(job.id)}/report.json`" download>转换报告</a>
    </section>
    <p class="motion-footnote">机器人按 URDF 显示实体模型，自动加载工程内 STL / DAE 网格；缺失网格时保留骨架。临时保留最近 3 次导入，服务重启后需重新导入。</p>
  </div>
</template>

<style scoped>
.motion-workbench { display: grid; grid-template-columns: 290px minmax(0, 1fr); gap: 14px; min-height: 0; flex: 1; overflow: auto; align-content: start; padding-right: 3px; }
.motion-import-body { display: grid; gap: 18px; padding: 18px; }
.motion-file { display: flex; flex-direction: column; gap: 9px; padding: 17px 14px; border: 1px dashed var(--line-2); border-radius: 6px; background: #141c20; min-width: 0; }
.motion-file strong { font-size: 14px; font-weight: 500; }
.motion-file > span:not(.eyebrow) { color: var(--muted); overflow-wrap: anywhere; }
.motion-file input { width: 100%; font-size: 11px; color: var(--muted); padding: 0; border: 0; background: none; }
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
.motion-error { color: var(--err); padding: 12px; border: 1px solid #69463d; border-radius: 5px; overflow-wrap: anywhere; }.motion-error button { margin-left: 15px; text-decoration: underline; }
.motion-result { display: flex; flex-wrap: wrap; align-items: center; gap: 20px; padding: 15px 18px; }.motion-result > div { display: flex; align-items: center; flex-wrap: wrap; gap: 10px; margin-right: auto; color: var(--accent); }.motion-result span { color: var(--muted); }.motion-result a { display: inline-flex; align-items: center; gap: 7px; color: var(--accent); }
@media(max-width: 1000px) { .motion-workbench { grid-template-columns: 1fr; }.motion-import-body { grid-template-columns: 1fr 1fr; }.motion-mapping-body { grid-template-columns: 1fr; }.motion-preview { min-height: 460px; } }
</style>
