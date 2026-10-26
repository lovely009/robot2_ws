import { computed, reactive, ref } from "vue";
import type { DemoEvent, Joint, RobotModel, Screen, Variant } from "./types";

// 所有控制只改变本页内存；几何和拓扑是唯一读取仓库的素材。
export const models = ref<RobotModel[]>([]);
const params = new URLSearchParams(location.search);
export const demo = reactive({
  screen: (["run", "teach", "sim"].includes(location.hash.slice(2))
    ? location.hash.slice(2)
    : "run") as Screen,
  variant: (["A", "B", "C"].includes(params.get("variant") || "")
    ? params.get("variant")
    : "A") as Variant,
  model: "",
  selected: "",
  query: "",
  connected: true,
  observer: "curve",
  target: "仿真",
  session: "running",
  backend: "MuJoCo",
  controller: "主机驱动",
  transport: "vcan",
  preview: false,
  executing: false,
  progress: 0,
  frame: 0,
  clock: 0,
  message: "",
  result: "未下发",
  result_joint: "",
  show_ghost: true,
  drafts: {} as Record<
    string,
    { target: number; kp: number; kd: number; enabled: boolean }
  >,
  frames: [] as {
    name: string;
    time: number;
    positions: Record<string, number>;
  }[],
  events: [] as DemoEvent[],
});
export const model = computed(() =>
  models.value.find((m) => m.name === demo.model),
);
export const selected = computed(() =>
  model.value?.joints.find((j) => j.name === demo.selected),
);
export const motors = computed(
  () => model.value?.joints.filter((j) => j.motor) || [],
);
export const draft = computed(() => demo.drafts[demo.selected]);
export const connected = computed(
  () => demo.connected && demo.session === "running",
);
export const can_command = computed(
  () => connected.value && !!selected.value?.motor && !demo.executing,
);
export const frame_end = computed(() => demo.frames.at(-1)?.time || 1);
export const labels: Record<Screen, string> = {
  run: "真机调试",
  teach: "动作工作台",
  sim: "仿真与 HIL",
};

export function log(text: string, level: DemoEvent["level"] = "info") {
  demo.events.unshift({
    at: new Date().toLocaleTimeString("zh-CN", { hour12: false }),
    level,
    text,
  });
  demo.events.splice(80);
}
export function notify(text: string) {
  demo.message = text;
  log(text);
}
export function go(screen: Screen) {
  demo.screen = screen;
  location.hash = `/${screen}`;
}
export function set_variant(variant: Variant) {
  demo.variant = variant;
  const url = new URL(location.href);
  url.searchParams.set("variant", variant);
  history.replaceState(null, "", url);
}
export function choose_model(name: string) {
  demo.model = name;
  demo.selected = model.value?.joints[0]?.name || "";
  demo.drafts = {};
  for (const j of model.value?.joints || []) {
    demo.drafts[j.name] = {
      target: neutral(j),
      kp: j.motor?.command?.kp ?? 0,
      kd: j.motor?.command?.kd ?? 0,
      enabled: false,
    };
  }
  demo.frames = [
    {
      name: "初始姿态",
      time: 0,
      positions: Object.fromEntries(
        (model.value?.joints || []).map((j) => [j.name, neutral(j)]),
      ),
    },
  ];
  stop();
  demo.frame = 0;
  demo.result = "未下发";
  if (
    model.value &&
    !(demo.backend === "MuJoCo" ? model.value.mujoco : model.value.gazebo)
  ) {
    demo.backend = model.value.mujoco ? "MuJoCo" : "Gazebo";
  }
  log(
    `载入 ${name} · ${model.value?.joints.length || 0} 个关节 · ${motors.value.length} 个映射设备`,
  );
}
export function neutral(j: Joint) {
  return Math.max(j.lower, Math.min(j.upper, 0));
}
export function measured(j: Joint, time = demo.clock) {
  const base = neutral(j);
  if (!connected.value) return base;
  const offset = Math.sin(time * 0.85 + (j.motor?.id || 0) * 0.3) * 0.012;
  return Math.max(j.lower, Math.min(j.upper, base + offset));
}
export function preview_position(j: Joint) {
  if (demo.screen === "teach" && demo.frames.length) {
    const frames = demo.frames;
    const next = frames.findIndex((f) => f.time > demo.frame);
    const a = frames[next < 0 ? frames.length - 1 : Math.max(0, next - 1)]!;
    const b = frames[next < 0 ? frames.length - 1 : next]!;
    const mix =
      b.time === a.time ? 0 : (demo.frame - a.time) / (b.time - a.time);
    return (
      (a.positions[j.name] ?? neutral(j)) * (1 - mix) +
      (b.positions[j.name] ?? neutral(j)) * mix
    );
  }
  return demo.drafts[j.name]?.target ?? neutral(j);
}
let command_timer: ReturnType<typeof setTimeout> | undefined;
export function send() {
  if (!can_command.value || !draft.value || !selected.value) return;
  const d = draft.value,
    j = selected.value;
  if (
    ![d.target, d.kp, d.kd].every(Number.isFinite) ||
    d.target < j.lower ||
    d.target > j.upper ||
    d.kp < 0 ||
    d.kd < 0
  ) {
    demo.result = "草稿无效";
    notify("请检查目标范围与增益数值");
    return;
  }
  clearTimeout(command_timer);
  demo.result_joint = j.name;
  demo.result = "模拟发送中";
  log(`${j.name} · 目标 ${d.target.toFixed(3)} rad · 模拟发送`);
  command_timer = setTimeout(() => {
    demo.result = connected.value
      ? "模拟已受理 · 未验证设备执行"
      : "模拟失败 · 连接已断开";
    log(
      `${demo.result_joint} · ${demo.result}`,
      connected.value ? "ok" : "warn",
    );
  }, 700);
}
export function capture() {
  if (demo.executing || !model.value) return;
  const positions = Object.fromEntries(
    model.value.joints.map((j) => [
      j.name,
      demo.drafts[j.name]?.target ?? neutral(j),
    ]),
  );
  demo.frames.push({
    name: `关键帧 ${demo.frames.length + 1}`,
    time: demo.frames.length * 2,
    positions,
  });
  demo.frame = frame_end.value;
  log(`添加关键帧 · ${demo.frames.length} 帧`);
}
export function stop() {
  demo.preview = false;
  demo.executing = false;
  demo.progress = 0;
}
export function toggle_preview() {
  demo.executing = false;
  if (demo.frame >= frame_end.value) demo.frame = 0;
  demo.preview = !demo.preview;
  log(demo.preview ? "开始本地预览 · 无下行命令" : "暂停本地预览");
}
export function execute() {
  if (!connected.value || !motors.value.length || demo.frames.length < 2)
    return;
  demo.preview = false;
  demo.executing = true;
  demo.frame = 0;
  log(`模拟执行 · ${demo.frames.length} 帧 · 目标 ${demo.target}`, "warn");
}
export function tick(dt: number) {
  if (connected.value) demo.clock += dt;
  if (demo.preview || demo.executing) {
    demo.frame = Math.min(frame_end.value, demo.frame + dt);
    demo.progress = demo.frame / frame_end.value;
    if (demo.frame >= frame_end.value) {
      log(
        demo.executing ? "模拟动作结束 · 无真实设备执行" : "本地预览结束",
        "ok",
      );
      demo.preview = false;
      demo.executing = false;
    }
  }
}
export function disconnect() {
  demo.connected = !demo.connected;
  if (!demo.connected) {
    demo.executing = false;
    demo.result = "连接中断";
  }
  log(
    demo.connected ? "模拟连接恢复" : "模拟连接断开 · 控制已禁用",
    demo.connected ? "ok" : "warn",
  );
}
export function cleanup() {
  clearTimeout(command_timer);
}
