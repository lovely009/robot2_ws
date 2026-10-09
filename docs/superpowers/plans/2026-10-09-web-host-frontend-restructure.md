# web_host 前端重构实施计划（控制台范式）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `web_host/web/` 从「单页 + 8 抽屉」（`app.js` 2613 行）重构为外壳 / 屏 / 面板三层的控制台界面，纯搬迁不新增功能，保持纯静态无构建链。

**Architecture:** `index.html` 持有全部标记（外壳 + 14 个面板 `<template>` + 5 个屏 `<template>`）；`js/core/` 提供 rosbridge、状态 store、hash 路由、Agent 进程管理与格式化；`js/panels/*.js` 是可挂载组件，`mount(root, ctx)` 返回 `{render(dirty), unmount()}`。订阅在入口全局建立并写 store，面板卸载只停渲染不丢数据。

**Tech Stack:** 原生 ES module + 浏览器 import map、标准库 `ThreadingHTTPServer`、three 0.160.1 + urdf-loader 0.13.1（vendor，不动）、roslib.min.js（不动）、Python 3 / pytest（静态契约测试）。

**Spec:** `docs/superpowers/specs/2026-10-09-web-host-frontend-restructure-design.md`

## Global Constraints

- **纯搬迁，不新增功能**：不新增话题订阅、不新增写操作、不改变任何安全语义与阈值。现有行为以 spec 的「功能对等清单」17 项为准，逐项核对。
- **保持无构建链**：不引框架、不引 npm/Vite、不引 JS 测试框架。`web/vendor/` 与 `roslib.min.js` 只搬位置不改版本。
- **`web/` 内不得出现目录符号链接**：新目录 `css/`、`js/` 必须是真实目录（`_web_data_files()` 的 `os.walk(followlinks=True)` 依赖这一条）。
- **`web/` 下新增任何文件后必须重跑 `colcon build --packages-select web_host`**，否则安装目录缺文件、页面 404 而构建「成功」。
- **DOM 单一来源**：所有标记写在 `index.html`（含全部 `<template>`）。**JS 不得生成带 `id` 的元素**
  （`el.id = 'x'` 与 `setAttribute('id', ...)` 都算），动态生成的行只用 class 与 `data-*`。
  注意 `data-id="..."` 是合法的（契约测试的正则用 `(?<![A-Za-z0-9_-])` 排除它）。
  这条由契约测试第 3 条断言。
- **id 全局唯一，且只在两处出现**：`index.html` 的 id 只允许出现在①外壳标记与②**面板模板**里。
  **屏模板里不得出现任何 `id`** —— 抽屉之类的屏内结构在 run/teach/device 三份屏模板里各出现
  一次，用 id 会在同一份 `index.html` 里重复，契约测试第 1 条直接变红（运行时因为只有一个屏挂载
  而不会暴露，所以必须靠测试挡住）。屏内结构一律用 class 或 `data-*`；面板内需要唯一钩子的元素
  放在面板模板里（定义一次，多屏引用时克隆）。
- **数据在 store，渲染在面板**：面板不得持有跨屏数据。`unmount()` 不得清空 store。
- **重表必须节流**：`motion_states` 是 50 Hz，重建表格的面板（`jointList`、`deviceParams`、`policy`）
  在 `mount()` 返回值里声明 `intervalMs: 200`，与重构前的刷新节奏一致；canvas 类面板（`arm`、`curve`、`imu`）
  各自在 render 内按需重绘。
- **中文注释写意图**，沿用现有注释密度与风格（`// ---------- 分区 ----------`）。标识符与 ROS 2 API 保留英文。
- **不改后端 Python 模块**：`static_server.py` / `agent_manager.py` / `rosbridge_shim.py` / `simulation_manager.py` 一行不改。
- **不改历史文档**：不重写 `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` 等历史正文里对 `web/app.js` 的引用。
- **既有测试不回退**：`ros2_ws/src/web_host/test/test_agent_manager.py` 保持通过。

## 配色与度量（唯一权威）

- **色板完全沿用现状**，不自造：`--bg:#0a0e16` `--panel:#121824` `--panel-2:#18202f` `--line:#212b3d` `--line-2:#2c3852` `--text:#d7e0ef` `--dim:#7f8ca6` `--dim-2:#55617a` `--accent:#3d9bff` `--ok:#2fd07a` `--warn:#e0a93a` `--err:#ff5f5f`。
- **度量按 V2 调整**：正文 12px → 12.5px，行内边距 3px 6px → 7px 11px，面板内边距 → 12px，圆角 5px → 9px，栅格间距 → 8px。
- **断点**：≥1360 三区全开；1080–1360 翼默认收起；820–1080 左导航收成 56px 图标导轨且翼收起；<820 单列（底部标签栏 + 顶部状态带 + 连接控件进浮层）。
- **折叠优先级**：用户显式选择 > 断点默认；`<820` 强制收起。

## 面板与屏的分发表（唯一权威）

| 面板 id | `js/panels/<id>.js` | 挂载屏 |
|---|---|---|
| `jointList` | 关节索引 | run, teach |
| `arm` | 臂视图（three.js） | run, teach |
| `readout` | 选中关节读数 | run, teach |
| `policy` | 使能与增益 | run, teach |
| `command` | 下发（快照 / 100Hz） | run, teach |
| `teach` | 示教 | teach |
| `deviceParams` | 全部参数（含行内设零/方向/保护） | device |
| `sim` | 仿真启动器 + 日志 | sim |
| `curve` | 曲线 | run, teach, diag |
| `events` | 事件日志 | run, teach, device, diag |
| `history` | 命令历史 | run, diag |
| `imu` | IMU | diag |
| `wing` | 状态翼 | 外壳（不进任何屏模板） |
| `notes` | 说明 | 说明浮层（不进任何屏模板） |

屏：`run` `teach` `device` `sim` `diag`；`run` 与 `teach` 的次级抽屉用 tab 切换，`device` 只有一个抽屉面板。

---

### Task 1: 外壳骨架、路由、状态与样式

**Files:**
- Create: `ros2_ws/src/web_host/web/css/tokens.css`
- Create: `ros2_ws/src/web_host/web/css/shell.css`
- Create: `ros2_ws/src/web_host/web/js/core/format.js`
- Create: `ros2_ws/src/web_host/web/js/core/store.js`
- Create: `ros2_ws/src/web_host/web/js/core/router.js`
- Create: `ros2_ws/src/web_host/web/js/main.js`
- Create: `ros2_ws/src/web_host/test/test_web_assets.py`
- Modify: `ros2_ws/src/web_host/web/index.html`（整体重写为外壳 + 5 个空屏模板）
- Modify: `ros2_ws/src/web_host/setup.py`（`_web_data_files()` 无需改动，但要在本任务后重跑构建验证子目录被收入）

**Interfaces:**
- Consumes: 无（首个任务）。
- Produces:
  - `js/core/format.js`：`$ = (id) => HTMLElement|null`、`fmt(v, d=2) => string`、`ageText(ms) => string`、`quatToEuler(x,y,z,w) => {roll,pitch,yaw}`、`rad2deg(r) => number`、`download(name, text, mime)`、`busLabel(index) => string`、`lifecyclePill(lc) => string`、`rad2deg`、常量 `LIFECYCLE` `EVENT_NAMES` `FAULT_NAMES` `DEV_SLOT_NAMES` `DEV_INIT_STATE` `PARAM_FIELD` `SYS_CMD` `MOTOR_ID_NONE` `STREAM_PERIOD_MS` `CHART_COLORS` `TRAJ_MAX_POINTS` `TRAJ_MAX_MOTORS`。
  - `js/core/store.js`：`store`（见下）、`markDirty(...keys)`、`onDirty(fn)`、`runRenderLoop()`、`MOUNT`（当前屏已挂载面板的注册表）。
  - `js/core/router.js`：`SCREENS`、`DEFAULT_SCREEN`、`initRouter({onScreenChange})`、`gotoScreen(id)`、`currentScreen()`。
  - `js/main.js`：`mountPanel(panelId, container)`、`unmountAll()`、`unmountPanel(panelId)`、上下文 `ctx`。
    （已挂载面板的注册表是 `main.js` 模块内的 `mounted`，不对外导出。）

- [ ] **Step 1: 写 `css/tokens.css`**

色板照抄现有 `style.css:2-19` 的值，只把度量换成 V2。全部尺寸必须走变量，后续任务不再手写像素。

```css
/* 设计 token：色板沿用重构前的值，度量按 V2（放松密度，靠间距做分层）。
   改颜色只改这里，改密度也只改这里。 */
:root {
  /* ---- 色板（与重构前完全一致，不自造） ---- */
  --bg: #0a0e16;
  --panel: #121824;
  --panel-2: #18202f;
  --line: #212b3d;
  --line-2: #2c3852;
  --text: #d7e0ef;
  --dim: #7f8ca6;
  --dim-2: #55617a;
  --accent: #3d9bff;
  --ok: #2fd07a;
  --warn: #e0a93a;
  --err: #ff5f5f;
  --mono: ui-monospace, "SF Mono", "JetBrains Mono", "Cascadia Mono", Consolas, monospace;
  --sans: -apple-system, "Segoe UI", "Microsoft YaHei", system-ui, sans-serif;

  /* ---- 度量（V2：行内边距 3/6 → 7/11，圆角 5 → 9，栅格间距 → 8） ---- */
  --fs-base: 12.5px;
  --fs-sm: 11px;
  --fs-xs: 10px;
  --pad-row: 7px 11px;
  --pad-panel: 12px;
  --gap: 8px;
  --gap-sm: 5px;
  --radius: 9px;
  --radius-sm: 5px;

  /* ---- 骨架尺寸 ---- */
  --nav-w: 132px;        /* 左导航展开宽 */
  --nav-w-icon: 56px;    /* <1080 图标导轨宽 */
  --wing-w: 168px;       /* 状态翼展开宽 */
  --wing-w-min: 34px;    /* 状态翼收起宽 */
  --topbar-h: 46px;
}

* { box-sizing: border-box; }

body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font: var(--fs-base)/1.5 var(--sans);
  -webkit-font-smoothing: antialiased;
}

/* 数字一律等宽：位置/速度/力矩/时间列对齐才读得出来 */
.num, .kv b, table td, table th { font-variant-numeric: tabular-nums; }
```

- [ ] **Step 2: 写 `css/shell.css`**

骨架三区 + 顶栏 + 左导航 + 状态翼 + 浮层 + 四个断点。断点只改这三处的宽度与可见性，屏内布局不在这里管。

```css
/* ---------- 骨架：顶栏 / 左导航 / 主区 / 状态翼 ---------- */
.shell {
  display: grid;
  grid-template-columns: var(--nav-w) minmax(0, 1fr) var(--wing-w);
  grid-template-rows: var(--topbar-h) minmax(0, 1fr);
  grid-template-areas: "top top top" "nav main wing";
  height: 100vh;
  gap: var(--gap-sm);
  padding: var(--gap-sm);
}
body.wing-collapsed .shell { grid-template-columns: var(--nav-w) minmax(0, 1fr) var(--wing-w-min); }

.topbar { grid-area: top; display: flex; align-items: center; gap: var(--gap-sm);
  background: var(--panel); border: 1px solid var(--line); border-radius: var(--radius);
  padding: 0 11px; font-size: var(--fs-sm); }
.nav { grid-area: nav; background: var(--panel); border: 1px solid var(--line);
  border-radius: var(--radius); padding: var(--gap-sm); display: flex; flex-direction: column; gap: 3px; }
.main { grid-area: main; min-width: 0; overflow: auto; }
.wing { grid-area: wing; background: var(--panel); border: 1px solid var(--line);
  border-radius: var(--radius); padding: var(--gap-sm); overflow: auto;
  display: flex; flex-direction: column; gap: var(--gap-sm); }

/* 左导航项：按钮不是链接——切屏不产生历史记录栈，hash 由 router 写 */
.nav button { display: flex; align-items: center; gap: 7px; width: 100%; text-align: left;
  background: transparent; border: 1px solid transparent; border-radius: var(--radius-sm);
  color: var(--dim); font: inherit; font-size: var(--fs-sm); padding: 7px 9px; cursor: pointer; }
.nav button:hover { background: var(--panel-2); color: var(--text); }
.nav button[aria-current="page"] { background: #14243c; border-color: #24456e; color: var(--text); }
.nav .navkey { margin-left: auto; color: var(--dim-2); font: var(--fs-xs) var(--mono); }

/* ---------- 顶栏 ---------- */
.brand { display: flex; align-items: center; gap: 7px; }
.brand .logo { width: 8px; height: 8px; border-radius: 2px; background: var(--accent); }
.chips { display: flex; gap: 5px; flex-wrap: wrap; }
.chip { border: 1px solid var(--line); border-radius: 999px; padding: 2px 9px; color: var(--dim); white-space: nowrap; }
.chip.ok { color: var(--ok); border-color: #1d5537; }
.chip.warn { color: var(--warn); border-color: #5e4a1d; }
.chip.err { color: var(--err); border-color: #5e2628; }
.conn { margin-left: auto; display: flex; align-items: center; gap: var(--gap-sm); }
.conn .dot { width: 7px; height: 7px; border-radius: 50%; background: var(--err); flex: none; }
.conn .dot.on { background: var(--ok); box-shadow: 0 0 5px rgba(47, 208, 122, .7); }
.conn input { background: var(--bg); border: 1px solid var(--line); border-radius: var(--radius-sm);
  color: var(--text); font: var(--fs-sm) var(--mono); padding: 5px 7px; width: 200px; }
.conn select { background: var(--bg); border: 1px solid var(--line); border-radius: var(--radius-sm);
  color: var(--text); font: inherit; font-size: var(--fs-sm); padding: 5px 7px; max-width: 240px; }
.agent-state { color: var(--dim); font-size: var(--fs-xs); max-width: 260px; overflow: hidden;
  text-overflow: ellipsis; white-space: nowrap; }
.agent-state.running { color: var(--ok); }
.agent-state.failed { color: var(--err); }

/* ---------- 按钮 ---------- */
button { background: var(--panel-2); border: 1px solid var(--line-2); border-radius: var(--radius-sm);
  color: var(--text); font: inherit; font-size: var(--fs-sm); padding: 5px 11px; cursor: pointer; }
button:hover:not(:disabled) { border-color: var(--accent); }
button.primary { background: #1b4b86; border-color: #2a6bb8; }
button.danger { background: #5a1f22; border-color: #7d3236; }
button.secondary { background: transparent; }
button.tiny-btn { font-size: var(--fs-xs); padding: 3px 8px; }
button:disabled, select:disabled, input:disabled { opacity: .55; cursor: not-allowed; }

/* ---------- 断连置灰：不隐藏，保留最后一帧数值并标注陈旧 ---------- */
body.offline .main, body.offline .wing { opacity: .6; }
.stale-badge { color: var(--warn); font-size: var(--fs-xs); }

/* ---------- 状态翼 ---------- */
.wing-head { display: flex; align-items: center; justify-content: space-between; color: var(--dim); font-size: var(--fs-xs); }
.wing-toggle { background: transparent; border: none; color: var(--dim); cursor: pointer; padding: 0 3px; }
body.wing-collapsed .wing > *:not(.wing-rail) { display: none; }
.wing-rail { display: none; flex-direction: column; align-items: center; gap: 6px; font: var(--fs-xs) var(--mono); }
body.wing-collapsed .wing-rail { display: flex; }

/* ---------- 浮层（说明 / 窄屏连接控件） ---------- */
.overlay { position: fixed; inset: 0; background: rgba(4, 7, 12, .72); display: none;
  align-items: center; justify-content: center; z-index: 20; }
.overlay[open], .overlay.on { display: flex; }
.overlay .sheet { background: var(--panel); border: 1px solid var(--line-2); border-radius: var(--radius);
  max-width: 900px; width: min(92vw, 900px); max-height: 84vh; overflow: auto; padding: var(--pad-panel); }

/* ---------- 断点 ---------- */
@media (max-width: 1360px) { :root { --wing-w: var(--wing-w-min); } }
@media (max-width: 1080px) {
  :root { --nav-w: var(--nav-w-icon); }
  .nav button .navlabel { display: none; }
  .nav button { justify-content: center; padding: 9px 0; }
}
@media (max-width: 820px) {
  .shell { grid-template-columns: minmax(0, 1fr);
    grid-template-rows: var(--topbar-h) auto minmax(0, 1fr) auto;
    grid-template-areas: "top" "wing" "main" "nav"; }
  /* 翼在窄屏只剩一条状态带；导航变底部标签栏 */
  :root { --wing-w: 100%; }
  .wing { flex-direction: row; align-items: center; }
  .nav { flex-direction: row; justify-content: space-around; }
  .conn .detail { display: none; }   /* 连接细节进「连接」浮层 */
}
```

- [ ] **Step 3: 写 `js/core/format.js`**

把 `app.js` 的常量区（`app.js:8-58`）与工具区（`app.js:61-79`）整体搬过来，**逐字照抄，不重写**。新增 `download()`（从 `app.js:1016` 搬）。

```js
// 常量与格式化：从重构前的 app.js 常量区与工具区原样搬移。
// 与固件的隐式契约（DEV_SLOT_NAMES 位序、PARAM_FIELD 次序）在下方注释里保留原样。

export const TOPICS = {
  topology: {name: '/micro_ros_up/motor_topology', type: 'can_bridge_msgs/msg/MotorTopology'},
  motion: {name: '/micro_ros_up/motion_states', type: 'can_bridge_msgs/msg/MotorMotionStateArray'},
  status: {name: '/micro_ros_up/motor_status_array', type: 'can_bridge_msgs/msg/MotorStatusArray'},
  events: {name: '/micro_ros_up/motor_events', type: 'can_bridge_msgs/msg/MotorEvent'},
  imu: {name: '/micro_ros_up/imu/data', type: 'sensor_msgs/msg/Imu'},
  temp: {name: '/micro_ros_up/imu/temp', type: 'std_msgs/msg/Float32'},
  command: {name: '/micro_ros_do/motor_command_array', type: 'can_bridge_msgs/msg/MotorCommandArray'},
  trajectory: {name: '/micro_ros_do/motor_trajectory', type: 'can_bridge_msgs/msg/MotorTrajectory'},
  trajectoryState: {name: '/micro_ros_up/trajectory_state', type: 'can_bridge_msgs/msg/MotorTrajectoryState'},
  policy: {name: '/micro_ros_do/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
  devParams: {name: '/micro_ros_up/device_param_report', type: 'can_bridge_msgs/msg/DeviceParamReport'},
  sysStatus: {name: '/micro_ros_up/system_status', type: 'can_bridge_msgs/msg/SystemStatus'},
  paramSet: {name: '/micro_ros_do/param_set', type: 'can_bridge_msgs/msg/ParamSetArray'},
  sysCmd: {name: '/micro_ros_do/system_command', type: 'can_bridge_msgs/msg/SystemCommand'},
};

export const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
export const EVENT_NAMES = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL', 'DEV_INIT', 'TRAJECTORY_REJECTED'];
// 索引即故障码。8 是历史值（ZERO_STALE，2026-09-20 按真机实测删除）——
// 空着不复用，免得旧日志/事件里的 8 被认成别的意思。
export const FAULT_NAMES = ['无', '协议', '厂家', 'Profile', '冲突', '离线',
  '设备参数', '策略代数', undefined, '缺武装确认'];
// ⚠️ 与固件位图的隐式契约：槽位顺序必须逐位对齐 firmware 的 device_raw 槽索引，
// 权威定义在 micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg 的 EVENT_DEV_INIT 注释块。
export const DEV_SLOT_NAMES = ['位置下限', '位置上限', '最大转速', '峰值电流',
  '保护开关(0x2650:00)', '方向(0x2541)'];
export const DEV_INIT_STATE = ['无设备对象', '待回读', '已核对', '与记录不一致',
  '回读不成功', '本机无记录'];
// ⚠️ param_device_slot_t 的次序（= DEV_SLOT_NAMES 下标 = 行序 = mismatch_mask 位序），
// **不是** param_field_t。两套里 field=3 都合法 → 互用不会被拒绝，会静默写错槽位。
export const PARAM_FIELD = {POS_MIN: 0, POS_MAX: 1, VEL_LIMIT: 2,
  TORQUE_LIMIT: 3, LIMIT_PROTECT: 4, DIRECTION: 5};
export const SYS_CMD = {SAVE: 0, REFRESH: 1, SET_ZERO: 2, CLEAR_FAULT: 3, REPORT: 4};
export const MOTOR_ID_NONE = 65535;
export const STREAM_PERIOD_MS = 10;
export const CHART_COLORS = ['#4da3ff', '#37d67a', '#f5b94a', '#ff6b6b', '#c792ea',
  '#f78c6c', '#82aaff', '#ff9e64', '#7e8bab'];
export const TRAJ_MAX_POINTS = 256;
export const TRAJ_MAX_MOTORS = 64;
export const TRAJ_STATE_TIMEOUT_PAD_MS = 3000;

export const $ = (id) => document.getElementById(id);
export const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v))
  ? '--' : v.toFixed(d);
export const ageText = (ms) => {
  if (ms === null || ms === undefined) return '--';
  if (ms === 0xFFFFFFFF) return '从未';
  return ms < 1000 ? ms + ' ms' : (ms / 1000).toFixed(1) + ' s';
};
export function quatToEuler(x, y, z, w) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = sinp >= 1 ? Math.PI / 2 : (sinp <= -1 ? -Math.PI / 2 : Math.asin(sinp));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}
export const rad2deg = (r) => r * 180 / Math.PI;

export function lifecyclePill(lc) {
  // 原样搬 app.js:508-514
}
export function busLabel(index) {
  // 原样搬 app.js:515-519
}
export function download(name, text, mime) {
  // 原样搬 app.js:1016-1024
}
export const CONTROLLER_PORT_STORAGE_KEY = 'robot2.controllerSerialPort';
export const ROS_URL_STORAGE_KEY = 'robot2.rosbridgeUrl';
export const FOLD_KEY = 'web_host.fold';
export const WING_KEY = 'web_host.wing';
```

`lifecyclePill` / `busLabel` / `download` 三处**逐字复制** `app.js:508-519` 与 `app.js:1016-1024` 的函数体，
不要改写，也不要改注释。

- [ ] **Step 4: 写 `js/core/store.js`**

新增文件（无对应旧代码）：把 `app.js` 的 `state` 对象（`app.js:83-105`）、`chartBuf`/`chart`
（`app.js:121-122`）、`history`（`app.js:125`）与 `paramCache`（`app.js:1823`）收进一个模块，
并加脏标记与 rAF 渲染循环。

```js
// 共享状态：所有跨屏数据都住在这里。面板只读它、只通过 ui 动作改它。
// 订阅在 main.js 全局建立并写这里，与屏是否可见无关——所以切屏不丢曲线缓冲与事件列表。
// 脏标记与 rAF 渲染循环也在本文件内实现，不另开模块。
```

```js
export const store = {
  // text：顶栏连接文案（setConn 写它，renderShell 画它）
  conn: {connected: false, text: '未连接', url: '', agent: 'stopped', agentDevice: '',
         agentError: '', agentReturncode: null, canStop: false, lastMsgAt: 0},
  // motors: id -> {topo, motion, status}（与重构前的 state.motors 同形状，搬迁期不改字段名）
  motors: new Map(),
  lastError: 0,
  lastErrorMotor: -1,
  sysStatus: {refresh_seq: 0, set_zero_seq: 0, clear_fault_seq: 0,
              save_seq: 0, save_in_progress: false, seen: false},
  devParamSeq: 0,
  devInit: new Map(),
  policy: new Map(),
  events: [],
  imu: {count: 0, last: 0, roll: 0, pitch: 0, yaw: 0, q: null},
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
  selectedId: 0,
  // 曲线：motor_id -> {t:[], position:[], velocity:[], effort:[], current:[]}
  chartBuf: new Map(),
  chart: {running: true, windowMs: 10000},
  history: [],
  paramCache: new Map(),
  traj: {activeTrajectoryId: 0, waitingState: false, segmentIndex: 0, segmentCount: 0},
};

let dirty = new Set();
let dirtyListener = null;

// 标脏：面板的 render(dirty) 会收到自上次渲染以来的键集合
export function markDirty(...keys) {
  for (const k of keys) dirty.add(k);
}
export function onDirty(fn) { dirtyListener = fn; }

// 单一 rAF 循环：没有任何面板标脏时不做任何事，因此未挂载的屏不空转。
export function runRenderLoop() {
  const frame = () => {
    if (dirty.size && dirtyListener) {
      const batch = dirty;
      dirty = new Set();
      dirtyListener(batch);
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

// 脏标记的键（面板按这些键判断要重绘什么）
export const DIRTY = {
  CONN: 'conn', TOPOLOGY: 'topology', MOTION: 'motion', STATUS: 'status',
  EVENTS: 'events', HISTORY: 'history', PARAMS: 'params', TRAJ: 'traj',
  IMU: 'imu', POLICY: 'policy', SELECTED: 'selected', ALERTS: 'alerts',
};
```

`store.motors` 保留重构前的 `{topo, motion, status}` 形状与全部字段名（`motor_id`、`online`、
`position` …），这样面板里的取值代码可以逐字搬移，不需要边搬边改字段名。

- [ ] **Step 5: 写 `js/core/router.js`**

```js
// hash 路由。屏不是模块：屏的标记是 index.html 里的 <template data-screen="...">，
// 这里只持有屏 id 的权威集合与默认屏。
export const SCREENS = ['run', 'teach', 'device', 'sim', 'diag'];
export const DEFAULT_SCREEN = 'run';

let onChange = null;
let current = null;

const normalize = (hash) => {
  const id = String(hash || '').replace(/^#\/?/, '').split('?')[0];
  return SCREENS.includes(id) ? id : DEFAULT_SCREEN;
};

export function currentScreen() { return current; }

export function gotoScreen(id) {
  const next = SCREENS.includes(id) ? id : DEFAULT_SCREEN;
  if (`#/${next}` === location.hash) return;
  location.hash = `#/${next}`;   // 触发 hashchange → 统一在 handler 里切
}

export function initRouter({onScreenChange}) {
  onChange = onScreenChange;
  const apply = () => {
    const next = normalize(location.hash);
    if (next === current) return;
    current = next;
    onChange(next);
  };
  window.addEventListener('hashchange', apply);
  apply();     // 首次：空 hash 或未知 hash 都落 DEFAULT_SCREEN
}
```

- [ ] **Step 6: 写契约测试 `test/test_web_assets.py`（此时必红）**

```python
"""web_host 前端静态契约测试。

重构后把全部标记放在 index.html（含面板与屏的 <template>），JS 只做行为。结构一旦漂移
就会出现两类错误——「改名漏改」与「新增文件忘重跑 colcon」——它们都不让构建失败，只在
浏览器里静默失效。本文件把这两类错误变成测试失败。

纯标准库解析，不引 Node、不启浏览器、不连 ROS。
"""
import json
import os
import re
import runpy
from html.parser import HTMLParser
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
WEB = PKG / 'web'
INDEX = WEB / 'index.html'

# 不属任何屏的面板：wing 由外壳挂载，notes 由说明浮层挂载
SHELL_ONLY_PANELS = {'wing', 'notes'}


class IndexParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = []
        self.screens = []
        self.panel_slots = []
        self.panel_templates = []
        self.gotos = []
        self.assets = []
        self._in_importmap = False
        self._importmap = []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if a.get('id'):
            self.ids.append(a['id'])
        if a.get('data-screen'):
            self.screens.append(a['data-screen'])
        if a.get('data-panel'):
            self.panel_slots.append(a['data-panel'])
        if a.get('data-panel-template'):
            self.panel_templates.append(a['data-panel-template'])
        if a.get('data-goto'):
            self.gotos.append(a['data-goto'])
        if tag == 'link' and a.get('href'):
            self.assets.append(a['href'])
        if tag == 'script':
            if a.get('src'):
                self.assets.append(a['src'])
            if a.get('type') == 'importmap':
                self._in_importmap = True

    def handle_endtag(self, tag):
        if tag == 'script':
            self._in_importmap = False

    def handle_data(self, data):
        if self._in_importmap:
            self._importmap.append(data)


@pytest.fixture(scope='module')
def index():
    assert INDEX.is_file(), f'缺少 {INDEX}'
    parser = IndexParser()
    parser.feed(INDEX.read_text(encoding='utf-8'))
    return parser


def js_files():
    files = sorted((WEB / 'js').rglob('*.js'))
    assert files, f'{WEB / "js"} 下没有任何 .js'
    return files


def router_screens():
    src = (WEB / 'js' / 'core' / 'router.js').read_text(encoding='utf-8')
    m = re.search(r'export\s+const\s+SCREENS\s*=\s*\[([^\]]*)\]', src)
    assert m, 'router.js 里找不到 SCREENS 常量'
    return [s.strip().strip('\'"') for s in m.group(1).split(',') if s.strip()]


# 1. id 唯一：面板模板会被克隆，同屏出现两个同 id 会让 getElementById 取到错的元素
def test_all_ids_unique(index):
    dupes = sorted({i for i in index.ids if index.ids.count(i) > 1})
    assert not dupes, f'index.html 里重复的 id：{dupes}'


# 2. JS 引用的 DOM id 必须存在（重构最常犯的错：改名漏改）
def test_js_dom_refs_exist(index):
    known = set(index.ids)
    ref = re.compile(r"\$\(\s*'([^']+)'\s*\)|getElementById\(\s*'([^']+)'\s*\)")
    missing = {}
    for f in js_files():
        for m in ref.finditer(f.read_text(encoding='utf-8')):
            name = m.group(1) or m.group(2)
            if name not in known:
                missing.setdefault(f.relative_to(WEB).as_posix(), set()).add(name)
    assert not missing, f'JS 引用了 index.html 里不存在的 id：{missing}'


# 3. JS 不得生成带 id 的 DOM（否则 id 会随数据重复，契约测试也失去意义）
def test_no_ids_generated_in_js():
    bad = {}
    literal = re.compile(r'(?<![A-Za-z0-9_-])id\s*=\s*["\'][^"\']+["\']')
    setattr_id = re.compile(r"setAttribute\(\s*['\"]id['\"]")
    for f in js_files():
        src = f.read_text(encoding='utf-8')
        hits = literal.findall(src) + setattr_id.findall(src)
        if hits:
            bad[f.relative_to(WEB).as_posix()] = hits[:5]
    assert not bad, f'JS 生成了带 id 的元素（动态行请只用 class）：{bad}'


# 4. 屏的三处表达必须一致：router 的 SCREENS、屏模板名、左导航的 data-goto
def test_screens_consistent(index):
    screens = router_screens()
    assert screens, 'SCREENS 为空'
    assert len(screens) == len(set(screens)), f'SCREENS 有重复：{screens}'
    assert set(index.screens) == set(screens), (
        f'屏模板与 SCREENS 不一致：模板 {sorted(index.screens)} vs SCREENS {sorted(screens)}')
    assert set(index.gotos) == set(screens), (
        f'左导航 data-goto 与 SCREENS 不一致：{sorted(index.gotos)} vs {sorted(screens)}')
    src = (WEB / 'js' / 'core' / 'router.js').read_text(encoding='utf-8')
    m = re.search(r"export\s+const\s+DEFAULT_SCREEN\s*=\s*'([^']+)'", src)
    assert m, 'router.js 里找不到 DEFAULT_SCREEN'
    assert m.group(1) in screens, f'DEFAULT_SCREEN {m.group(1)} 不在 SCREENS 里'


# 5. 屏模板里声明的每个面板占位都有对应模板
def test_panel_slots_have_templates(index):
    slots, templates = set(index.panel_slots), set(index.panel_templates)
    assert slots <= templates, f'屏模板引用了没有 <template> 的面板：{sorted(slots - templates)}'


# 6. 每个面板模板都有对应模块文件
def test_panel_templates_have_modules(index):
    missing = sorted(p for p in set(index.panel_templates)
                     if not (WEB / 'js' / 'panels' / f'{p}.js').is_file())
    assert not missing, f'缺少面板模块 js/panels/*.js：{missing}'


# 7. 每个面板模块都被某屏引用（wing/notes 由外壳挂载，豁免）
def test_panel_modules_are_referenced(index):
    used = set(index.panel_slots) | SHELL_ONLY_PANELS
    unused = sorted(f.stem for f in (WEB / 'js' / 'panels').glob('*.js')
                    if f.stem not in used)
    assert not unused, f'面板模块没有被任何屏引用：{unused}'


# 8. 相对 import 目标存在
def test_relative_imports_exist():
    pat = re.compile(r"""(?:^|\n)\s*(?:import|export)[^'"\n]*?from\s*['"]([^'"]+)['"]""")
    missing = []
    for f in js_files():
        for m in pat.finditer(f.read_text(encoding='utf-8')):
            spec = m.group(1)
            if spec.startswith('.'):
                target = (f.parent / spec).resolve()
                if not target.is_file():
                    missing.append(f'{f.relative_to(WEB)} -> {spec}')
    assert not missing, f'import 目标文件不存在：{missing}'


# 9. 从 main.js 出发的 import 图无环
def test_import_graph_acyclic():
    pat = re.compile(r"""(?:^|\n)\s*(?:import|export)[^'"\n]*?from\s*['"]([^'"]+)['"]""")

    def edges(f):
        out = []
        for m in pat.finditer(f.read_text(encoding='utf-8')):
            spec = m.group(1)
            if spec.startswith('.'):
                out.append((f.parent / spec).resolve())
        return out

    seen, stack = set(), []

    def walk(f):
        assert f not in stack, f'import 图有环：{" -> ".join(p.name for p in stack + [f])}'
        if f in seen:
            return
        stack.append(f)
        for nxt in edges(f):
            walk(nxt)
        stack.pop()
        seen.add(f)

    walk((WEB / 'js' / 'main.js').resolve())


# 10. index.html 引用的静态资产都在磁盘上
def test_index_assets_exist(index):
    missing = []
    for ref in index.assets:
        if ref.startswith(('http://', 'https://', '//', 'data:')):
            continue
        if not (WEB / ref).is_file():
            missing.append(ref)
    assert not missing, f'index.html 引用了不存在的文件：{missing}'


# 11. data_files 必须覆盖 web/ 下的每个文件（防「构建成功但页面 404」）
def test_data_files_cover_web_tree(monkeypatch):
    captured = {}
    import setuptools

    monkeypatch.setattr(setuptools, 'setup', lambda **kw: captured.update(kw))
    cwd = os.getcwd()
    os.chdir(PKG)
    try:
        runpy.run_path(str(PKG / 'setup.py'), run_name='__main__')
    finally:
        os.chdir(cwd)

    assert 'data_files' in captured, 'setup.py 没有调用 setuptools.setup'
    declared = {Path(f).resolve() for _dest, files in captured['data_files'] for f in files}
    on_disk = {p.resolve() for p in WEB.rglob('*') if p.is_file()}
    uncovered = sorted(str(p.relative_to(WEB)) for p in on_disk - declared)
    assert not uncovered, (
        f'web/ 下这些文件不会被安装（新增文件后必须重跑 colcon build）：{uncovered}')


# 12. importmap 的每个映射目标存在
def test_importmap_targets_exist(index):
    raw = ''.join(index._importmap).strip()
    assert raw, 'index.html 缺少 importmap'
    imports = json.loads(raw).get('imports', {})
    assert imports, 'importmap 里没有 imports'
    for _key, value in imports.items():
        if value.startswith(('http://', 'https://', '//')):
            continue
        assert (WEB / value).exists(), f'importmap 目标不存在：{value}'
```

- [ ] **Step 7: 跑测试确认它失败**

Run: `cd ros2_ws && python3 -m pytest src/web_host/test/test_web_assets.py -x -q`
Expected: FAIL —— `缺少 .../web/index.html` 里没有 `data-screen`，或 `js/panels` 不存在导致
`test_panel_modules_are_referenced` 报找不到目录。**这一步是必须的**：若它意外通过，说明断言
没生效，先修测试。

- [ ] **Step 8: 重写 `web/index.html` 为外壳**

保留文件头（`<!DOCTYPE>`、`<meta>`、`<title>`、importmap 三行）不变；`<body>` 整段替换。
现有 `index.html:22-51`（顶栏）与 `index.html:354-361`（页脚与脚本）里的 id 全部保留原名，
**不要改名**——改名会让后续面板搬迁的 `$('id')` 逐处失效。

```html
<body>
  <div class="shell">
    <!-- ============ 顶栏：全局常驻（连接 / 控制板 / Agent / 计数 / 说明） ============ -->
    <header class="topbar">
      <div class="brand"><span class="logo"></span><b>Robot2</b></div>
      <div class="chips">
        <span class="chip" id="chipOnline">在线 -/-</span>
        <span class="chip" id="chipActive">运行 0</span>
        <span class="chip" id="chipConflict">冲突 0</span>
        <span class="chip" id="chipFault">故障 0</span>
        <span class="chip" id="chipHz">运动流 -- Hz</span>
      </div>
      <div class="conn">
        <span class="dot" id="connDot"></span>
        <span id="connText">未连接</span>
        <span class="detail">
          <input id="rosUrl" type="text" spellcheck="false" placeholder="ws://localhost:9090"
            aria-label="rosbridge WebSocket 地址"
            title="地址自动保存；刷新页面自动连接或重连 rosbridge，不会启动或停止 Agent">
          <label class="port-picker" title="选择 micro-ROS 控制板串口">
            <span>控制板</span>
            <select id="serialPort" aria-label="控制板串口">
              <option value="">扫描串口中…</option>
            </select>
          </label>
          <button id="btnPortRefresh" class="secondary">扫描</button>
          <button id="btnConnect"
            title="启动所选控制板串口的 micro-ROS Agent；不改变 rosbridge 连接">启动并连接</button>
          <button id="btnDisconnect" class="secondary" disabled
            title="停止由本页面启动的 micro-ROS Agent">断开</button>
        </span>
        <span id="agentText" class="agent-state">Agent 未启动</span>
        <button id="btnHelp" class="secondary tiny-btn" title="安全与操作注记、单位约定、话题清单">说明</button>
      </div>
    </header>

    <!-- ============ 左导航：屏切换（1–5 快捷键），data-goto 与 SCREENS 一致 ============ -->
    <nav class="nav" id="nav">
      <button data-goto="run"    aria-current="page"><span class="navlabel">运行</span><span class="navkey">1</span></button>
      <button data-goto="teach"><span class="navlabel">示教</span><span class="navkey">2</span></button>
      <button data-goto="device"><span class="navlabel">设备</span><span class="navkey">3</span></button>
      <button data-goto="sim"><span class="navlabel">仿真</span><span class="navkey">4</span></button>
      <button data-goto="diag"><span class="navlabel">诊断</span><span class="navkey">5</span></button>
    </nav>

    <!-- ============ 主区：屏模板克隆到这里 ============ -->
    <main class="main" id="screenRoot"></main>

    <!-- ============ 状态翼：外壳挂载，不属任何屏 ============ -->
    <aside class="wing" id="wingPanel" aria-label="状态翼">
      <div class="wing-rail" id="wingRail"></div>
      <div class="wing-head">状态<span><button class="wing-toggle" id="wingToggle" aria-label="收起或展开状态翼">◂</button></span></div>
      <div id="wingBody"></div>
    </aside>
  </div>

  <!-- ============ 浮层：说明（Esc 或点遮罩关闭） ============ -->
  <div class="overlay" id="helpOverlay">
    <div class="sheet">
      <div class="wing-head">说明
        <button class="tiny-btn secondary" id="helpClose">关闭</button></div>
      <div id="helpBody"></div>
    </div>
  </div>

  <!-- ============ 屏模板：每个屏一项，主区按 hash 克隆其中之一 ============ -->
  <template data-screen="run"></template>
  <template data-screen="teach"></template>
  <template data-screen="device"></template>
  <template data-screen="sim"></template>
  <template data-screen="diag"></template>

  <script src="roslib.min.js"></script>
  <script type="module" src="js/main.js"></script>
</body>
```

原 `index.html` 里 `<style>`/`<link rel="stylesheet" href="style.css">` 换成三份 CSS：

```html
  <link rel="stylesheet" href="css/tokens.css">
  <link rel="stylesheet" href="css/shell.css">
  <link rel="stylesheet" href="css/panels.css">
```

本任务需要先建一个**空的** `css/panels.css`（后续任务往里加面板样式）：

```css
/* 面板通用样式：随各面板任务逐步补充。骨架样式在 shell.css。 */
```

`<template>` 里此时是空的——面板随各自任务填入。`index.html` 旧内容（`index.html:57-352`）的
每一块标记在后继任务里逐块搬进对应的面板模板，**本任务先不删**：把旧文件另存为
`web/index.legacy.html` 前先确认 git 里已提交（`git log --oneline -1` 有 Task 1 的提交后即可
`git rm` 时用 `git show HEAD~N:...` 找回）。更稳妥的做法：本任务直接重写 `index.html`，
旧标记从 git 历史取（`git show HEAD:ros2_ws/src/web_host/web/index.html`），不另存副本。

- [ ] **Step 9: 写 `js/main.js`**

```js
// 入口：装外壳、初始化路由、挂面板、建立全局订阅、起 rAF 渲染循环。
// 屏切换只换主区里克隆的模板与其面板；顶栏/左导航/状态翼不重建。
import { store, markDirty, onDirty, runRenderLoop, DIRTY } from './core/store.js';
import { $, FOLD_KEY, WING_KEY } from './core/format.js';
import { SCREENS, initRouter, currentScreen } from './core/router.js';

// 已挂载面板：panelId -> 面板实例
const mounted = new Map();

export function mountPanel(panelId, container) {
  // 动态 import：未使用的面板模块不会被下载
  return import(`./panels/${panelId}.js`).then((mod) => {
    const inst = mod.mount(container, ctx);
    mounted.set(panelId, inst);
    return inst;
  });
}

export function unmountAll() {
  for (const [, inst] of mounted) {
    try { inst.unmount(); } catch (e) { console.error('unmount 失败', e); }
  }
  mounted.clear();
}

export function unmountPanel(panelId) {
  const inst = mounted.get(panelId);
  if (!inst) return;
  try { inst.unmount(); } catch (e) { console.error('unmount 失败', e); }
  mounted.delete(panelId);
}

// 面板契约（权威）：
//   mount(root, ctx) -> { render(dirty), unmount(), intervalMs? }
//   - root：屏模板里的 [data-panel] 占位元素（wing 例外，见 Task 3）
//   - render(dirty)：dirty 是自上次渲染以来变脏的键集合（core/store.js 的 DIRTY）
//   - intervalMs：可选，>0 时该面板的 render 至多每 N 毫秒一次（重表用 200）
//   - unmount()：解绑事件、释放 canvas/three 资源；**不得清空 store**
// 面板上下文：所有面板共用的能力都在这里，面板不直接操作别的面板
export const ctx = {
  store,
  markDirty,
  ui: {
    selectMotor(id) {
      store.selectedId = id;
      markDirty(DIRTY.SELECTED);
    },
    gotoScreen(id) { gotoScreen(id); },
    toast(text, level = '') { /* Task 2 接事件日志；本任务只记 console */ console.log('[toast]', level, text); },
  },
};

// 面板渲染节流：motion 流是 50 Hz，若每帧都重建表格会比重构前的 200 ms 定时刷新
// 更糟。面板可在 mount 返回值里声明 intervalMs（默认 0 = 每脏帧都渲染）。
const lastRender = new Map();

function renderPanels(dirty, now) {
  for (const [id, inst] of mounted) {
    const every = inst.intervalMs || 0;
    if (every > 0) {
      const prev = lastRender.get(id) || 0;
      if (now - prev < every) continue;
      lastRender.set(id, now);
    }
    try { inst.render(dirty); } catch (e) { console.error(`面板 ${id} 渲染失败`, e); }
  }
}

// 外壳文案（顶栏与 chips）由 core/agent.js 与 core/ros.js 写 store 后标脏，
// 渲染在 Task 2 接上。本任务先只分发面板。
onDirty((dirty) => renderPanels(dirty, performance.now()));

const FOLD_OF_SCREEN = (screen) => `${FOLD_KEY}.${screen}`;

// 切屏：卸载旧屏的全部面板 → 克隆新屏模板 → 挂载其面板
async function showScreen(screen) {
  unmountAll();
  const root = $('screenRoot');
  root.replaceChildren();
  const tpl = document.querySelector(`template[data-screen="${screen}"]`);
  if (!tpl) { console.error('未知的屏', screen); return; }
  root.append(tpl.content.cloneNode(true));
  for (const btn of document.querySelectorAll('#nav button')) {
    btn.setAttribute('aria-current', btn.dataset.goto === screen ? 'page' : 'false');
  }
  // 屏内所有 [data-panel] 占位按出现顺序挂载
  for (const slot of root.querySelectorAll('[data-panel]')) {
    await mountPanel(slot.dataset.panel, slot);
  }
  markDirty(DIRTY.ALERTS);
}

function gotoScreen(id) {
  if (id === currentScreen()) return;
  location.hash = `#/${id}`;
}

// 状态翼的折叠：记忆优先，<820 由 CSS 强制
function initWing() {
  const collapsed = localStorage.getItem(WING_KEY) === '1';
  document.body.classList.toggle('wing-collapsed', collapsed);
  $('wingToggle').onclick = () => {
    const next = !document.body.classList.contains('wing-collapsed');
    document.body.classList.toggle('wing-collapsed', next);
    localStorage.setItem(WING_KEY, next ? '1' : '0');
    markDirty(DIRTY.ALERTS);
  };
}

function initHelp() {
  const overlay = $('helpOverlay');
  $('btnHelp').onclick = () => overlay.classList.add('on');
  $('helpClose').onclick = () => overlay.classList.remove('on');
  overlay.addEventListener('click', (e) => { if (e.target === overlay) overlay.classList.remove('on'); });
}

function initKeys() {
  document.addEventListener('keydown', (e) => {
    if (e.target.closest('input, select, textarea')) return;
    if (e.key === 'Escape') { $('helpOverlay').classList.remove('on'); return; }
    const idx = '12345'.indexOf(e.key);
    if (idx >= 0 && idx < SCREENS.length) gotoScreen(SCREENS[idx]);
  });
}

function initNav() {
  $('nav').addEventListener('click', (e) => {
    const btn = e.target.closest('button[data-goto]');
    if (btn) gotoScreen(btn.dataset.goto);
  });
}

function boot() {
  initNav();
  initWing();
  initHelp();
  initKeys();
  initRouter({onScreenChange: showScreen});
  runRenderLoop();
}

boot();
```

- [ ] **Step 10: 跑测试确认通过**

Run: `cd ros2_ws && python3 -m pytest src/web_host/test/test_web_assets.py -q`
Expected: PASS（12 项）。此时 `js/panels/` 目录还不存在——先建目录并放**一个空占位**是不对的；
正确做法是本任务内不建 `js/panels/`，把 `test_panel_modules_are_referenced` 与
`test_panel_templates_have_modules` 写成对空集合成立（上面的代码已经如此：模板集合为空时两条
断言都不触发）。`js_files()` 的断言只要求 `js/` 下有 `.js`，`js/core/*` 已满足。

- [ ] **Step 11: 构建并做浏览器验证**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select web_host
source install/setup.bash
ros2 launch web_host web_host.launch.py
```

浏览器打开 `http://localhost:8080`，逐条核对：

1. 三区骨架出现（顶栏 / 左导航 / 空主区 / 状态翼），无 404、控制台无未捕获异常；
2. 点左导航 5 项与按 `1`–`5`，hash 依次变为 `#/run` `#/teach` `#/device` `#/sim` `#/diag`，
   当前项高亮；刷新后停在原屏；
3. 直接访问 `http://localhost:8080/#/nonsense` 落回 `#/run`；
4. 点状态翼 `◂` 收起成 34px 竖条，刷新后保持收起；
5. 窗口拖到 1080 与 820 附近，左导航变图标导轨、翼收起、<820 变底部标签栏；
6. `curl -sI http://localhost:8080/css/tokens.css | head -1` 与
   `curl -sI http://localhost:8080/js/main.js | head -1` 都是 `200 OK`。

- [ ] **Step 12: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/web_host/web/index.html ros2_ws/src/web_host/web/css \
        ros2_ws/src/web_host/web/js ros2_ws/src/web_host/test/test_web_assets.py
git commit -m "feat(web_host): 前端外壳、hash 路由与共享状态

单页 app.js 拆为外壳/屏/面板三层的第一步：index.html 成为 DOM 单一来源，
router 持有屏 id 的权威集合，store 收拢跨屏状态并加脏标记 rAF 循环。

契约测试同时就位，把「改名漏改」与「新增文件忘重跑 colcon」两类静默失效
变成测试失败。面板为空，旧 app.js 尚未接入，页面此时只有骨架。"
```

---

### Task 2: rosbridge 订阅与 Agent 进程管理

**Files:**
- Create: `ros2_ws/src/web_host/web/js/core/ros.js`
- Create: `ros2_ws/src/web_host/web/js/core/agent.js`
- Modify: `ros2_ws/src/web_host/web/js/main.js`（`boot()` 里接上两者 + 定时器）

**Interfaces:**
- Consumes: `core/store.js` 的 `store`、`markDirty`、`DIRTY`；`core/format.js` 的 `TOPICS`、`$`、`MOTOR_ID_NONE`、`EVENT_NAMES`。
- Produces:
  - `core/ros.js`：`connect()`、`closeSocket()`、`pub(key, message) => boolean`、`ensurePublisher(key)`、`isConnected() => boolean`、`waitSysStatus(timeoutMs=2000)`、`waitDevParamSeq(baseline, timeoutMs)`、`sendSystemCommand(command, motorId)`、`sendParamSet(targets)`、`sendTrajectory(points, targetIds)`、`cancelTrajectory()`、`primeTrajectorySnapshot(segment)`、`TRAJ_SEGMENT_MARK`（示教回执对齐用）。
  - `core/agent.js`：`refreshAgentControls()`、`refreshAgentStatus()`、`startAgent()`、`stopAgent()`；写 `store.conn.agent*` 并 `markDirty(DIRTY.CONN)`。

- [ ] **Step 1: 写 `js/core/ros.js`**

把 `app.js:127-505` 整段搬过来，只做四处适配：

1. `state` → `store`，`state.motors` 保持原字段名。
2. 每次写入 store 后加 `markDirty(...)`：`topology` 帧 → `DIRTY.TOPOLOGY | DIRTY.ALERTS`；
   `motion` 帧 → `DIRTY.MOTION`；`status` 帧 → `DIRTY.STATUS | DIRTY.ALERTS`；
   `events` 帧 → `DIRTY.EVENTS | DIRTY.PARAMS | DIRTY.ALERTS`；`imu`/`temp` → `DIRTY.IMU`；
   `devParams` → `DIRTY.PARAMS`；`sysStatus` → `DIRTY.ALERTS`。
3. **撤掉所有直接 `renderXxx()` 调用**（`renderMotors()` / `renderEvents()` / `renderParamSummary()` /
   `renderDevRows()` / `renderPolicyRows()`）——改为 `markDirty`，由 rAF 循环驱动面板渲染。
   这是本次重构的核心改动：订阅层不认识面板。
4. `setConn()` 改写为写 store 并切 `body.offline`：

```js
export function setConn(ok, text) {
  store.conn.connected = ok;
  store.conn.text = text || (ok ? '已连接' : '未连接');
  document.body.classList.toggle('offline', !ok);
  markDirty(DIRTY.CONN);
}
```

重连退避（`app.js:296-338` 的 2 s 定时器）、掉线清场（清 `store.motors` 与 `store.paramCache`、
`store.sysStatus.seen = false`）、发布端预热（连接后 `['command','trajectory','policy','paramSet','sysCmd'].forEach(ensurePublisher)`）
三条行为逐字保留，注释也保留——它们是踩过坑的记录。

- [ ] **Step 2: 写 `js/core/agent.js`**

把 `app.js:132-271`（`setAgentStatus`、`agentApi`、`refreshAgentControls`、`refreshAgentStatus`、
`startAgent`、`stopAgent`）整段搬过来。`setAgentStatus` 改为写 `store.conn` 的
`agent`/`agentDevice`/`agentError`/`agentReturncode`/`canStop` 五个字段并 `markDirty(DIRTY.CONN)`，
顶栏文案由面板层（本任务里是 main.js 的壳接线）渲染。其余函数逐字照搬。

- [ ] **Step 3: 在 `main.js` 里接线并渲染顶栏**

`boot()` 里追加，并在 `ctx.ui` 之外加一个壳渲染函数（订阅 `DIRTY.CONN`）：

```js
import { connect, closeSocket, waitSysStatus } from './core/ros.js';
import { refreshAgentControls, refreshAgentStatus, startAgent, stopAgent } from './core/agent.js';

// 顶栏文案：连接态、Agent 状态、chips。订阅层不碰 DOM，全部在这里画。
function renderShell(dirty) {
  if (dirty.has(DIRTY.CONN)) {
    $('connDot').classList.toggle('on', store.conn.connected);
    $('connText').textContent = store.conn.text || (store.conn.connected ? '已连接' : '未连接');
    const el = $('agentText');
    el.classList.remove('running', 'failed');
    // 三段文案与重构前 setAgentStatus 逐字一致（见 app.js:138-165）
    if (store.conn.agent === 'running') {
      el.textContent = `Agent 进程运行中${store.conn.agentDevice ? ` · ${store.conn.agentDevice}` : ''}`;
      el.classList.add('running');
      el.title = '由 web_host 管理的 micro-ROS Agent 进程';
      $('btnDisconnect').disabled = false;
    } else if (store.conn.agent === 'failed') {
      el.textContent = `Agent 已退出${store.conn.agentReturncode === null ? '' : ` · code ${store.conn.agentReturncode}`}`;
      el.classList.add('failed');
      el.title = store.conn.agentError || store.conn.agentDevice || 'Agent 进程已退出';
      $('btnDisconnect').disabled = true;
    } else if (store.conn.agent === 'error') {
      el.textContent = store.conn.canStop
        ? `Agent 已运行${store.conn.agentDevice ? ` · ${store.conn.agentDevice}` : ''}`
        : (store.conn.agentError ? `Agent 启动失败 · ${store.conn.agentError}` : 'Agent 启动失败');
      el.classList.add('failed');
      el.title = store.conn.agentError || '启动 Agent 失败';
      $('btnDisconnect').disabled = !store.conn.canStop;
    } else {
      el.textContent = 'Agent 未启动';
      el.title = '';
      $('btnDisconnect').disabled = true;
    }
  }
  if (dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.MOTION)) {
    renderChips();
  }
}
```

**把 `renderShell` 接进脏帧分发**（Task 1 的回调目前只分发面板，`renderShell` 正是本任务定义的）：

```js
onDirty((dirty) => {
  const now = performance.now();
  renderShell(dirty);          // 顶栏连接文案与 Agent 状态
  renderPanels(dirty, now);
});
```

`renderChips()` 搬 `app.js:1712-1735` 的 `refreshHeader()`（在线/运行/冲突/故障与 `chipHz`），
字段名不变。`onDirty` 的回调改为「先 `renderShell(dirty)`，再分发给 `mounted` 里的面板」。

顶栏控件接线（搬 `app.js:2529-2538`）：

```js
  const defaultRosUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  $('rosUrl').value = localStorage.getItem(ROS_URL_STORAGE_KEY) || defaultRosUrl;
  $('rosUrl').addEventListener('input', () => {
    localStorage.setItem(ROS_URL_STORAGE_KEY, $('rosUrl').value.trim());
  });
  $('btnConnect').onclick = startAgent;
  $('btnDisconnect').onclick = stopAgent;
  $('btnPortRefresh').onclick = refreshAgentControls;
  $('serialPort').addEventListener('change', (e) => {
    if (e.target.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, e.target.value);
  });
```

`boot()` 末尾（搬 `app.js:2603-2605` 的语义，一行不改顺序）：`refreshAgentControls();`
`setInterval(refreshAgentStatus, 2000);` `connect();`。

连接建立后自动读取参数的行为（`app.js:315-318` 的 `waitSysStatus().then(() => loadParams())`）
本任务先**不接**，改为在 Task 8 接进设备参数面板——中间几批页面没有参数表，提前调用会引用
不存在的 DOM。

- [ ] **Step 4: 建立「断连禁用写操作」的统一规则**

spec 的「断连与错误」要求写操作按钮在断连时直接禁用、不等超时。这条覆盖四个面板的按钮，
因此在这里一次定规则，后续面板任务照办。`core/store.js` 加：

```js
// 写操作是否可用。断连时页面不猜、不降级、不静默重试（与 motor_hardware_interface 的
// fail-fast 语义一致）：按钮直接禁用，而不是等请求超时。
export function writesAllowed() { return store.conn.connected; }

// 统一给一组按钮设禁用态；面板在自己的 render 里调用。
export function syncWriteButtons(ids, allow) {
  for (const id of ids) {
    const el = document.getElementById(id);
    if (el) el.disabled = !allow;
  }
}
```

各面板负责的写按钮（后续任务在自己的 `render` 里调 `syncWriteButtons(WRITE_IDS, writesAllowed())`）：

| 面板 | 写按钮 id |
|---|---|
| `command` | `btnSendOnce` `btnAddRow` `btnFillOnline` `btnClearRows`（`chkStream` 用 `disabled` 属性） |
| `policy` | 每行的「策略」按钮，以及 `chkAllPolicy` 之外的启用/增益输入框（行是动态生成的，按 `[data-pol]` 查询后逐个设禁用） |
| `deviceParams` | `btnParamLoad` `btnParamApply` `btnParamSave` 与行内的设零/方向/限位保护输入 |
| `teach` | `btnTeachPlay` `btnTeachPause` `btnTeachStep` `btnTeachRecord` |

`btnParamLoad`（「读取」）在断连时也禁用：它发 `system_command(REPORT)`，属于下行写操作。

- [ ] **Step 5: 手动验证**

先不做真机：只连 rosbridge（终端里 `ros2 launch web_host web_host.launch.py`），浏览器打开。
核对：

1. 控制台无异常，`connDot` 变绿、`connText` 显示「已连接」；
2. `ros2 topic pub` 一条 `motor_topology` 后 chips 有数字（无固件时全 0 是正常的）；
3. 顶栏「启动并连接」在未选串口时给出「请先选择控制板串口」；选一个不存在权限的串口会显示
   Agent 启动失败而非静默；
4. **杀掉 rosbridge**（Ctrl+C）：3 秒内 `connDot` 变红、`connText` 显示「已断开，2s 后重连…」，
   重启 launch 后自动恢复连接（`app.js:293-295` 记录的重连坑）；
5. 断连期间 `body` 上有 `offline` 类（可用 `document.body.className` 核对）。

- [ ] **Step 6: 提交**

```bash
git add ros2_ws/src/web_host/web/js/core/ros.js ros2_ws/src/web_host/web/js/core/agent.js \
        ros2_ws/src/web_host/web/js/core/store.js ros2_ws/src/web_host/web/js/main.js
git commit -m "feat(web_host): rosbridge 订阅与 Agent 管理搬入 core 模块

订阅层不再认识面板：写入 store 后只 markDirty，渲染交给 rAF 循环。重连退避、
掉线清场、发布端预热三条行为与注释原样保留。"
```

---

### Task 3: 状态翼

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/wing.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（`#wingBody` 里加标记）
- Modify: `ros2_ws/src/web_host/web/css/shell.css`（翼内的计数与告警清单样式）

**Interfaces:**
- Consumes: `store.motors`、`store.devInit`、`store.events`、`store.selectedId`；`ctx.ui.gotoScreen`、`ctx.ui.selectMotor`。
- Produces: 面板 id `wing`（**不进任何屏模板**，由外壳挂载——契约测试的 `SHELL_ONLY_PANELS` 已豁免）。

翼的内容与数据来源（spec「状态翼」一节）：

| 元素 | 数据来源 |
|---|---|
| 臂缩略 | Task 5 的 `arm` 面板提供的第二个挂载点（本任务先留空占位 `#wingArm`） |
| 在线/运行/冲突/故障计数 | 与顶栏 chips 同一份计算（`store.motors` + `store.status`） |
| 告警清单 | ① `store.status` 的 `lifecycle === 7`（`CONFIG_CONFLICT`）→ 跳「设备」屏并选中该台；② `store.devInit` 的 `state` 为 3/4/5 → 同上；③ `lifecycle === 8`（`OFFLINE`）→ 跳「运行」屏 |
| 最近一条事件 | `store.events[0]` |

- [ ] **Step 1: `index.html` 的 `#wingBody` 加标记**

```html
      <div id="wingBody">
        <div id="wingArm" class="wing-arm"></div>
        <div class="wing-counts" id="wingCounts">--</div>
        <ul class="wing-alerts" id="wingAlerts"><li class="empty">无告警</li></ul>
        <div class="wing-last tiny muted" id="wingLast">--</div>
      </div>
```

`#wingRail`（收起态 34px）由 JS 填：状态点 + 在线比 + 告警计数 + 展开钮文字。

- [ ] **Step 2: 写 `js/panels/wing.js`**

```js
// 状态翼：任何屏都常驻的窄栏。它只读 store，不改任何状态（点告警会跳屏并选中该台）。
import { store, DIRTY } from '../core/store.js';
import { $ } from '../core/format.js';

const ALERT_SCREEN = {configConflict: 'device', offline: 'run', params: 'device'};

function collectAlerts() {
  const out = [];
  for (const [id, row] of store.motors) {
    const lc = row.status ? row.status.lifecycle : undefined;
    const dev = store.devInit.get(id);
    if (lc === 7 || (dev && dev.state === 3)) {
      out.push({id, kind: 'configConflict', text: `ID${id} 参数未核对`});
    } else if (dev && (dev.state === 4 || dev.state === 5)) {
      out.push({id, kind: 'params', text: `ID${id} ${dev.state === 4 ? '回读不成功' : '本机无记录'}`});
    } else if (lc === 8) {
      out.push({id, kind: 'offline', text: `ID${id} 离线`});
    }
  }
  return out;
}
```

**重要**：面板标记已经在 `index.html` 的 `#wingBody` 里，**不要克隆模板、不要用 innerHTML 复制**。
本面板的挂载点就是 `#wingBody` 本身：`mount(root, ctx)` 收到 `root === #wingBody`，直接绑事件、
直接渲染其子元素即可，**不需要克隆模板**。翼是外壳的一部分，不进屏模板，所以它不是
`<template data-panel-template>` 定义的面板——契约测试里 `wing` 走 `SHELL_ONLY_PANELS` 豁免，
`js/panels/wing.js` 只需存在且有被豁免的引用即可。

```js
export function mount(root, ctx) {
  const counts = $('wingCounts');
  const alertsEl = $('wingAlerts');
  const lastEl = $('wingLast');
  const railEl = $('wingRail');

  alertsEl.addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (!li) return;
    ctx.ui.selectMotor(parseInt(li.dataset.id, 10));
    ctx.ui.gotoScreen(li.dataset.screen);
  });

  const render = (dirty) => {
    if (dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY)
        || dirty.has(DIRTY.PARAMS)) {
      const {online, total, active, conflict, fault} = countMotors();
      counts.textContent = `● 在线 ${online}/${total}　运行 ${active}　冲突 ${conflict}　故障 ${fault}`;
      counts.classList.toggle('err', conflict + fault > 0);
    }
    if (dirty.has(DIRTY.ALERTS) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.PARAMS)) {
      const list = collectAlerts();
      alertsEl.replaceChildren();
      if (!list.length) {
        const li = document.createElement('li');
        li.className = 'empty';
        li.textContent = '无告警';
        alertsEl.append(li);
      }
      for (const a of list) {
        const li = document.createElement('li');
        li.dataset.id = String(a.id);          // 动态行只用 data- 与 class，不用 id
        li.dataset.screen = ALERT_SCREEN[a.kind];
        li.className = 'alert';
        li.textContent = a.text;
        alertsEl.append(li);
      }
      railEl.textContent = list.length ? `⚠${list.length}` : '●';
      railEl.title = list.map((a) => a.text).join('\n') || '无告警';
    }
    if (dirty.has(DIRTY.EVENTS)) {
      const e = store.events[0];
      lastEl.textContent = e ? `最近：${e.name} ID${e.id}` : '最近：--';
    }
  };

  render(new Set([DIRTY.MOTION, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.ALERTS, DIRTY.EVENTS]));
  return {render, unmount() { /* 外壳常驻，无定时器与 canvas 需要释放 */ }};
}
```

`countMotors()` 是本文件内的私有函数，统计口径与 `renderChips()` 必须一致（同一份 `store.motors`）；
两处都实现一遍是重复，改为在 `js/core/store.js` 里加 `export function countMotors()` 并让两边都用它。

- [ ] **Step 3: `main.js` 挂载翼**

`boot()` 里 `initWing()` 之后加 `mountPanel('wing', $('wingBody'))`（`.then` 不必等待）。

- [ ] **Step 4: 手动验证**

用 `ros2 topic pub` 造两条数据：一条 `motor_topology`（ID 4 在线）、一条 `motor_status_array`
（ID 4 的 `lifecycle = 7`）。核对：翼里出现「ID4 参数未核对」，点它跳到 `#/device`；
把窗口收窄到 1080 以下，翼自动收起为竖条并显示 `⚠1`；点竖条展开回完整翼（在记忆允许的
范围内——`<820` 时强制收起）。

- [ ] **Step 5: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/wing.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/shell.css ros2_ws/src/web_host/web/js/core/store.js
git commit -m "feat(web_host): 常驻状态翼与告警清单

任何屏都能一眼看到在线/运行/冲突/故障与告警明细，点告警跳对应屏并选中该台。
计数口径与顶栏 chips 共用 core/store.js 的 countMotors()，避免两处各算一遍。"
```

---

### Task 4: 运行屏 —— 关节索引 / 选中关节读数 / 使能与增益

本任务同时完成**本次重构最核心的一次拆分**：`renderMotors()`（`app.js:520-571`）一次干了五件事
——渲染关节行、更新顶栏 chips、刷新选中关节读数、同步臂着色、刷新参数总览。拆开后：

| `renderMotors()` 的职责 | 新归属 |
|---|---|
| `list.innerHTML = rows…`（`app.js:522-554`） | `panels/jointList.js` |
| chips 汇总（`app.js:555-570`） | `core/store.js` 的 `countMotors()` + `main.js` 的 `renderChips()` |
| `renderSelected()`（`app.js:679-711`） | `panels/readout.js` |
| `arm3dSyncColors()`（`app.js:656-671`） | `panels/arm.js`（Task 5） |
| `renderParamSummary()`（`app.js:2169-2223`） | `panels/deviceParams.js`（Task 8） |

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/jointList.js`
- Create: `ros2_ws/src/web_host/web/js/panels/readout.js`
- Create: `ros2_ws/src/web_host/web/js/panels/policy.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（`<template data-screen="run">` 与三个面板模板）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`
- Modify: `ros2_ws/src/web_host/web/js/core/store.js`（`countMotors()`、`devDetailTitle()`、`paramRowIds(showAll)`、`targetOf(id)`、`URLQ`/`MODEL`/`JOINT_OF_MOTOR`）
- Modify: `ros2_ws/src/web_host/web/js/core/format.js`（`devSlotNames()`、`devRecoveryText()`、`fmtParam()`）
- Modify: `ros2_ws/src/web_host/web/js/main.js`（`renderChips()`，`onDirty` 分发已就位）

**Interfaces:**
- Consumes: `store`、`markDirty`、`DIRTY`、`ctx.ui.selectMotor`、`core/ros.js` 的 `pub()`。
- Produces: 面板 `jointList`、`readout`、`policy`；store 选择器 `devDetailTitle(id) => string`、`paramRowIds(showAll) => number[]`、`targetOf(id) => number`；format 的 `devSlotNames(mask)`、`devRecoveryText(mask)`、`fmtParam(v)`。

- [ ] **Step 1: 先搬共享选择器进 `core/store.js` 与 `core/format.js`**

`store.js` 追加（都是纯读，不改状态）。`countMotors()` 已在 Task 3 建好，这里不重复：

```js
// URL 参数：换模型或换接线不改代码（?joints=1:joint_link1,2:joint_link2&model=<模型目录名>）
export const URLQ = new URLSearchParams(location.search);
export const JOINT_OF_MOTOR = (() => {
  const q = URLQ.get('joints');
  if (!q) return {1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4'};
  const m = {};
  for (const pair of q.split(',')) {
    const [id, j] = pair.split(':');
    if (id && j) m[parseInt(id, 10)] = j;
  }
  return m;
})();
export const MODEL = URLQ.get('model') || 'zhoubuguanjie_youbi_001';

// 页面上最后发出的目标值（history[0]）。⚠️ 这不是固件的设定点：固件侧的钳位、
// 软停、整包拒收页面都看不到；轨迹在播期间（target_source == 2）目标由固件插值，
// 页面无从得知，此时宁可不画（调用方负责判 target_source）。
export function targetOf(id) {
  const h = store.history[0];
  if (!h) return NaN;
  const t = h.targets.find((x) => x.id === id);
  return t ? t.pos : NaN;
}

// chips 与状态翼共用的计数口径（原先散在 renderMotors 的汇总段里）
// 行悬停明细：哪几项与固件记录不一致。搬 app.js:2140-2155 的 devDetailTitle。
export function devDetailTitle(id) { /* 原样搬 app.js:2140-2155 */ }

// 行筛选（两张表各自的「显示全部」都走它）。搬 app.js:1915-1921，把
// $('chkAllParams') 改为调用方传入的 showAll —— 重构后设备参数表与使能表分处
// 两个屏，各自一个复选框，不再共用一个 DOM 元素。
export function paramRowIds(showAll) {
  const all = [...store.paramCache.keys()].sort((a, b) => a - b);
  return (showAll || !store.selectedId) ? all : all.filter((i) => i === store.selectedId);
}
```

`format.js` 追加 `devSlotNames()`（搬 `app.js:2123-2130`）、`devRecoveryText()`（搬 `app.js:2131-2139`）、
`fmtParam()`（搬 `app.js:2156-2168`，7 位有效数字），三者逐字照搬。

- [ ] **Step 2: `index.html` 加三个面板模板与运行屏**

`<template data-screen="run">` 的内容（此时只含本任务存在的三个面板：

```html
  <template data-screen="run">
    <div class="screen run">
      <section class="col-index"><div data-panel="jointList"></div></section>
      <section class="col-stage"></section>
      <section class="col-side">
        <div data-panel="readout"></div>
        <div data-panel="policy"></div>
      </section>
      <section class="col-command"></section>
      <section class="col-drawer"></section>
    </div>
  </template>
```

**只写已存在的槽位**：`col-stage` / `col-command` / `col-drawer` 三个 `<section>` 本任务先留空壳，
里面的 `<div data-panel="...">` 分别在 Task 5（`arm`）、Task 6（`command`）、Task 7（`curve`）里加上。
理由是契约测试第 5/6 条要求「每个槽位都有模板、每个模板都有模块」，提前加槽位会让测试变红。
这是本次迁移的**硬规则：槽位与其面板模块必须在同一批提交里成对出现**。

面板模板（标记来源是 git 里的旧 `index.html`，用
`git show 2c77589:ros2_ws/src/web_host/web/index.html` 取回）：

```html
  <!-- 关节索引：标记搬自旧 index.html:61-67 的 <details> 内容 -->
  <template data-panel-template="jointList">
    <details class="card" data-fold="joints" open>
      <summary>关节 <span class="spacer"></span><span class="sum" id="sumJoints">--</span></summary>
      <ul class="joints" id="motorRows">
        <li class="empty">等待 motor_topology…</li>
      </ul>
    </details>
  </template>

  <!-- 选中关节读数：搬自旧 index.html:87-100 -->
  <template data-panel-template="readout">
    <details class="card" data-fold="sel" open>
      <summary id="selTitle">未选中关节（点左栏或臂上的小球）</summary>
      <div class="kv">
        <div><span>位置</span><b id="selPosition">--</b></div>
        <div><span>速度</span><b id="selVelocity">--</b></div>
        <div><span>力矩</span><b id="selEffort">--</b></div>
        <div><span>电流</span><b id="selCurrent">--</b></div>
        <div><span>温度</span><b id="selTemp">--</b></div>
        <div><span>跟随误差</span><b id="selErr">--</b></div>
        <div><span>命令源</span><b id="selSource">--</b></div>
        <div><span>故障</span><b id="selFault">--</b></div>
        <div><span>测量年龄</span><b id="selAge">--</b></div>
      </div>
    </details>
  </template>

  <!-- 使能与增益：搬自旧 index.html:165-178；「显示全部」改用新 id chkAllPolicy -->
  <template data-panel-template="policy">
    <details class="sec" data-fold="pol" open>
      <summary>使能与增益<span class="spacer"></span><span class="sum" id="sumPol">--</span></summary>
      <div class="ctrl-row wrap">
        <label class="check tiny"><input type="checkbox" id="chkAllPolicy"> 显示全部</label>
      </div>
      <div class="table-wrap">
        <table class="params">
          <thead><tr><th class="num">ID</th><th>型号</th><th>启用</th><th class="num">kp</th><th class="num">kd</th><th></th></tr></thead>
          <tbody id="polRows"><tr class="empty"><td colspan="6">连接后点「读取」载入电机列表</td></tr></tbody>
        </table>
      </div>
      <div id="polResult" class="result tiny"></div>
    </details>
  </template>
```

**`chkAllParams` 这个 id 留给 Task 8 的设备参数表**（旧 id 不变），使能表用新的 `chkAllPolicy`；
两者各自绑自己的筛选，`paramRowIds(showAll)` 由调用方传入 —— 这是重构后唯一一处行为可见的变化，
已在 spec 的功能对等清单第 7 项写明（「本行筛选（默认只显示选中台）、显示全部」）。

- [ ] **Step 3: 写 `js/panels/jointList.js`**

```js
// 关节索引：一行一台，点击选中（与臂上的小球双向联动）。只读 store。
import { store, markDirty, DIRTY, devDetailTitle } from '../core/store.js';
import { $, fmt, busLabel, LIFECYCLE, FAULT_NAMES } from '../core/format.js';

export function mount(root, ctx) {
  const list = $('motorRows');
  const summary = $('sumJoints');

  // 事件委托：列表每次渲染都整体重建
  list.addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (li) ctx.ui.selectMotor(parseInt(li.dataset.id, 10));
  });

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.MOTION)
          || dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.PARAMS))) return;
    // 行渲染逐字搬 app.js:522-554（含 jdot 着色、状态文案、devDetailTitle 悬停）
    // ...
    summary.textContent = `${store.motors.size} 台`;
  };

  render(new Set([DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.MOTION, DIRTY.SELECTED, DIRTY.PARAMS]));
  return {render, intervalMs: 200, unmount() {}};
}
```

行渲染那段的搬移规则（**逐字**，只改数据来源）：

| 旧写法 | 新写法 |
|---|---|
| `state.motors` | `store.motors` |
| `state.selectedId` | `store.selectedId` |
| `devDetailTitle(id)` | 从 `core/store.js` import |
| `busLabel` / `fmt` / `LIFECYCLE` / `FAULT_NAMES` | 从 `core/format.js` import |
| 空表文案 `暂无数据（等待 motor_topology）` | 不变 |

注意 `renderMotors` 里用的是 `var(--muted)`（旧 `style.css` 的变量名），新 token 里叫
`--dim-2` —— 改成 `var(--dim-2)`，否则色点会静默变成透明。

- [ ] **Step 4: 写 `js/panels/readout.js`**

`renderSelected()`（`app.js:679-711`）整段搬过来，三处改动：

1. `state` → `store`；`targetOf` 从 `core/store.js` import。
2. `if (window.Arm3D) window.Arm3D.setSelected(...)` 两处删除 —— 臂的选中态由它自己的面板在
   `DIRTY.SELECTED` 响应（Task 5），不再由读数面板顺带同步。
3. `JOINT_OF_MOTOR` 不再需要（同第 2 条）。

```js
export function mount(root, ctx) {
  const render = (dirty) => {
    if (!(dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS))) return;
    renderSelected();     // app.js:679-711 的搬移版
  };
  render(new Set([DIRTY.SELECTED]));
  return {render, intervalMs: 200, unmount() {}};
}
```

- [ ] **Step 5: 写 `js/panels/policy.js`**

搬四个函数：`renderPolicyRows`（`app.js:2093-2122`）、`polInputs`（`app.js:1754-1763`）、
`sendPolicyRow`（`app.js:1780-1818`）、`hasPendingPolicyEdit`（`app.js:2074-2088`），
以及 `writeResult`（`app.js:1901-1908`）与 `setPolResult`（`app.js:1910`）。

改动：

1. `paramRowIds()` → `paramRowIds($('chkAllPolicy').checked)`（本面板自己的复选框）。
2. `pub('policy', ...)` 从 `core/ros.js` import。
3. **保留防抹除机制**：`render` 里先判 `hasPendingPolicyEdit()`，为真就跳过重绘。这段注释
   （「漏判的后果是操作员勾上启用，不到一秒就被下一帧重建抹掉」）必须原样保留——它记的是
   踩过的坑。
4. `state.policy` → `store.policy`；`state.motors` → `store.motors`。
5. `chkAllPolicy.onchange` 与 `motorRows` 的选中变化都要触发重绘（后者走 `DIRTY.SELECTED`）。

6. 按 Task 2 定下的规则禁用写操作：`render` 末尾调
   `syncWriteButtons(['chkAllPolicy'], writesAllowed())`，并对行内动态生成的
   `[$('polRows').querySelectorAll('[data-pol], [data-pol-field]')]` 逐个 `el.disabled = !writesAllowed()`
   （行是每次渲染重建的，所以禁用态要在重建之后设）。

```js
export function mount(root, ctx) {
  $('chkAllPolicy').onchange = () => render(new Set([DIRTY.POLICY, DIRTY.SELECTED]));
  const render = (dirty) => {
    if (!(dirty.has(DIRTY.POLICY) || dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.TOPOLOGY)
          || dirty.has(DIRTY.PARAMS) || dirty.has(DIRTY.CONN))) return;
    if (hasPendingPolicyEdit()) return;    // 有未下发的手改就不重建
    renderPolicyRows();
    $('sumPol').textContent = `${paramRowIds($('chkAllPolicy').checked).length} 台`;
    const allow = writesAllowed();
    syncWriteButtons(['chkAllPolicy'], allow);
    for (const el of $('polRows').querySelectorAll('[data-pol], [data-pol-field]')) {
      el.disabled = !allow;
    }
  };
  render(new Set([DIRTY.POLICY, DIRTY.SELECTED, DIRTY.TOPOLOGY, DIRTY.PARAMS]));
  return {render, intervalMs: 200, unmount() {}};
}
```

- [ ] **Step 6: `main.js` 加 `renderChips()`**

搬 `app.js:555-570` 的 chips 段，改用 `countMotors()`：

```js
function renderChips() {
  const c = countMotors();
  $('chipOnline').textContent = `在线 ${c.online}/${c.total}`;
  $('chipActive').textContent = `运行 ${c.active}`;
  $('chipActive').classList.toggle('ok', c.active > 0);
  $('chipConflict').textContent = `冲突 ${c.conflict}`;
  $('chipConflict').classList.toggle('warn', c.conflict > 0);
  $('chipFault').textContent = `故障 ${c.fault}`;
  $('chipFault').classList.toggle('err', c.fault > 0);
}
```

并把「运动流 Hz」的 1 s 定时器（`app.js:1722-1728`）搬进 `boot()`；「IMU Hz」的定时器留给
Task 11 的 `imu` 面板自己管。

- [ ] **Step 7: 跑契约测试 + 构建 + 手动验证**

```bash
cd ros2_ws && python3 -m pytest src/web_host/test/test_web_assets.py -q   # 期望 PASS
colcon build --symlink-install --packages-select web_host && source install/setup.bash
```

核对（无真机时用 `ros2 topic pub` 造 `motor_topology` / `motor_status_array` / `motion_states`）：

1. 运行屏出现关节索引、读数、使能与增益三块；点关节行 → 高亮切换、读数跟着变、顶栏 chips
   数字正确；
2. 点「策略」按钮 → `polResult` 出现结果文本；勾上「启用」后 1 秒内**不被重绘抹掉**（这是
   `hasPendingPolicyEdit` 的保护，必须专门试）；
3. 「显示全部」勾上后列出全部台；
4. 切到「示教」「设备」屏再切回「运行」，索引与读数的数据仍在（store 存活），且曲线缓冲没被清空
   （此时还没有曲线面板，用 `window.__store` 之类临时手段核对即可——**这个临时手段不要提交**）；
5. Performance 面板里确认切屏后 `jointList` 的 render 不再被调用。

- [ ] **Step 8: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/jointList.js \
        ros2_ws/src/web_host/web/js/panels/readout.js \
        ros2_ws/src/web_host/web/js/panels/policy.js \
        ros2_ws/src/web_host/web/js/core/store.js ros2_ws/src/web_host/web/js/core/format.js \
        ros2_ws/src/web_host/web/js/main.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 运行屏的关节索引、读数与使能增益

拆开 renderMotors：它原先一次干五件事（渲染行、顶栏 chips、读数、臂着色、参数总览），
现在各自有归属，chips 计数收进 store.countMotors() 由顶栏与状态翼共用。

使能表的「显示全部」改用新 id chkAllPolicy：设备参数表与使能表分处两屏，不再共用一个
复选框（spec 功能对等清单第 7 项）。"
```

---

### Task 5: 臂视图面板

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/arm.js`
- Move: `ros2_ws/src/web_host/web/arm3d.js` → `js/panels/arm.js` 的 three.js 部分（本任务先**复制**，
  旧文件在 Task 12 删除）
- Modify: `ros2_ws/src/web_host/web/index.html`（`run` 与 `teach` 屏的 `arm` 槽位 + 面板模板）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `store.motors`、`store.selectedId`、`store.history`、`targetOf`、`JOINT_OF_MOTOR`、`MODEL`、`ctx.ui.selectMotor`。
- Produces: 面板 `arm`；模块内部沿用 `arm3d.js` 的既有导出（`init` / `setJoints` / `setTargets` / `setJointColors` / `setSelected` / `resize` / `setRunning`）。

- [ ] **Step 1: 建立 `panels/arm.js` 骨架**

`arm3d.js` 已有的 `init/setJoints/setTargets/setJointColors/setSelected/resize/setRunning`
七个导出与全部 three.js 代码**逐字搬入**新文件（含 `THREE.Object3D.DEFAULT_UP.set(0, 0, 1)` 上方
那段 Z-up 注释、vendor 路径说明、`MARKER_R`、`tick()`）。三处改动：

1. **删掉模块尾部的全局桥**（旧 `arm3d.js:204-207` 的 `window.Arm3D = {...}` 与
   `window.dispatchEvent(new Event('arm3d-ready'))`）——现在由 `mount()` 直接调用，不再赌脚本顺序。
2. `import URDFLoader from './vendor/URDFLoader.js'` 的路径改为 `'../../vendor/URDFLoader.js'`
   （文件下沉了一层）。
3. `scene.background = new THREE.Color(0x0b1120)` 保持不变（`--bg` 的近邻色，深色台架下已调过）。

- [ ] **Step 2: 加 `mount()`，把 app.js 的同步逻辑搬进来**

搬 `arm3dSyncJoints()`（`app.js:620-632`）、`arm3dSyncGhost()`（`app.js:640-655`）、
`arm3dSyncColors()`（`app.js:656-671`）、`arm3dStart()`（`app.js:712-753`）。

```js
// 臂视图面板：只做显示。不订阅、不发布任何话题——关节角与状态从 store 读，
// 因此它坏掉也只是看不见臂，不影响任何控制链路。
export function mount(root, ctx) {
  const canvas = document.createElement('canvas');
  canvas.className = 'arm3d';
  const hud = document.createElement('div');
  hud.className = 'hud';
  hud.textContent = '臂视图加载中…';
  root.replaceChildren(canvas, hud);
  // 图例与"拖拽旋转 · 点击关节"提示搬自旧 index.html:74-82，用同样的类名
  // ...

  let ready = false;
  const onSelect = (jointName) => {
    // 臂上小球 → 电机 id（JOINT_OF_MOTOR 的反查），点击与左栏选中双向联动
    const entry = Object.entries(JOINT_OF_MOTOR).find(([, j]) => j === jointName);
    if (entry) ctx.ui.selectMotor(parseInt(entry[0], 10));
  };
  init(canvas, {urdfUrl, packages, onReady: () => { ready = true; markDirty(DIRTY.MOTION); },
                onError: (e) => { hud.classList.add('err'); hud.textContent = `臂视图加载失败：${e.message || e}`; },
                onSelect});

  const render = (dirty) => {
    if (!ready) return;
    if (dirty.has(DIRTY.MOTION)) { arm3dSyncJoints(); arm3dSyncGhost(); }
    if (dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY)) arm3dSyncColors();
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
```

`urdfUrl` 与 `packages` 的拼法逐字搬 `arm3dStart()`（`app.js:712-753`），
`urdfUrl = \`/robot_arm_gazebo/models/${MODEL}/urdf/${MODEL}.urdf\``、
`packages = {'robot_arm_gazebo': '/robot_arm_gazebo'}`。

`disposeScene()` 是本任务新增的**唯一新代码**（旧 `arm3d.js` 从不需要释放——它一辈子只挂一次）：

```js
// 释放 three.js 资源。切屏会反复 mount/unmount，不释放会一路涨显存。
// 几何与材质在幽灵与实体之间共享（ghostMat 是共用材质），故先收集再释放，避免重复。
function disposeScene() {
  // 旧 app.js:749-751 在 arm3dStart() 里注册了两个 window 级监听器却从不摘除。
  // 面板化之后每次 mount 都注册一遍、旧监听器仍引用已释放的 renderer —— 必须成对摘掉。
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
```

两个监听器是**新提名的具名函数**（旧代码里是内联箭头函数，摘不掉）：

```js
const onWindowResize = () => resize();
const onVisibilityChange = () => setRunning(!document.hidden);
```

在 `mount()` 里注册：

```js
  window.addEventListener('resize', onWindowResize);
  document.addEventListener('visibilitychange', onVisibilityChange);
  setRunning(!document.hidden);     // 与旧 app.js:751 一致
```

`arm3d.js` 没有 ResizeObserver：`resize()`（`arm3d.js:186-196`）直接读 `canvas.clientWidth/clientHeight`，
由 window 的 resize 事件驱动。`setRunning(false)` 走 `renderer.setAnimationLoop(null)`
（`arm3d.js:197-203`），`disposeScene()` 必须在它之后调用。

- [ ] **Step 3: 幽灵目标的既有两条限制要保留**

`arm3dSyncGhost()` 顶部「轨迹在播期间（`target_source == 2`）不画幽灵」与
「幽灵用的是页面自己最后一次发出去的目标，不是固件设定点」两条注释**逐字保留**——
它们是写在界面上的限制，不是实现细节。

- [ ] **Step 4: `index.html` 加槽位与模板**

`<template data-screen="run">` 里 `col-stage` 那行已经从 Task 4 起就存在（`data-panel="arm"`），
但**当时不能加**——按本计划的硬规则，槽位与模块必须同批出现。因此：

- 把 `<div data-panel="arm"></div>` 填进 Task 4 留下的 `<section class="col-stage">` 空壳；
- 新建 `<template data-screen="teach">`（内容见下）并同样填一行。

`teach` 屏本任务只建壳：`col-side`、`col-command`、`col-teach` 里先不放槽位，
`jointList`（已存在）与 `arm` 两个槽位本任务就位，其余在 Task 6（`command`）、
Task 7（`curve`）、Task 9（`teach`）里按同一硬规则加回。

`teach` 屏本任务只建壳（面板在 Task 9 填）：

```html
  <template data-screen="teach">
    <div class="screen teach">
      <section class="col-index"><div data-panel="jointList"></div></section>
      <section class="col-stage"><div data-panel="arm"></div></section>
      <section class="col-side"></section>
      <section class="col-command"></section>
      <section class="col-teach"></section>
    </div>
  </template>
```

臂面板模板：

```html
  <template data-panel-template="arm">
    <div class="stage" id="armStage">
      <div class="legend">
        <span><i class="sw" style="background:#3d9bff"></i>实际</span>
        <span><i class="sw ghost"></i>目标</span>
        <span><i class="dt" style="background:#2fd07a"></i>就绪</span>
        <span><i class="dt" style="background:#e0a93a"></i>过渡</span>
        <span><i class="dt" style="background:#ff5f5f"></i>故障</span>
        <span class="spacer"></span>
        <span>拖拽旋转 · 点击关节</span>
      </div>
    </div>
  </template>
```

canvas 与 hud 由 JS 建（见 Step 2），因此模板里不放 `id="arm3d"` 这个 canvas——
**旧 id `arm3d` 与 `armHud` 就此退役**，`index.html` 里不再出现。旧 `style.css` 里
`#arm3d` / `.hud` 那几条规则搬进 `css/panels.css` 并改成 `.stage .arm3d` / `.stage .hud`
（canvas 高度用 `clamp(300px, 48vh, 640px)`，与旧值一致）。

- [ ] **Step 5: 手动验证**

1. 运行屏出现臂视图，URDF 与 STL 加载成功（`/robot_arm_gazebo/...` 的 200）；
2. 造 `motion_states` 帧，臂动；点臂上的小球 → 左栏对应行高亮；点左栏行 → 臂上小球高亮；
3. 造一条 `motor_trajectory` 播放期的 `motion_states`（`target_source = 2`）→ 幽灵消失；
4. 在「运行」与「示教」屏之间来回切 10 次，DevTools 的 Memory 里 WebGL 上下文数不增长
   （`renderer.info.memory.geometries` 每次 mount 后回到同一起点）；
5. 把 `robot_arm_gazebo` 的 share 目录临时改名（或在 launch 里传 `model_dir:=/nonexistent`）
   → hud 显示加载失败，页面其余部分照常工作。

- [ ] **Step 6: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/arm.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 臂视图改为可挂载面板

arm3d.js 的 three.js 代码逐字搬入 panels/arm.js，去掉 window.Arm3D 全局桥
（改由 mount 直接调用，不再赌模块与脚本的顺序）。unmount 显式释放 geometry 与
material —— 切屏会反复挂载，不释放会一路涨显存。

走在界面上的两条限制照旧保留：幽灵是页面自己发出去的目标、轨迹在播时不画幽灵。"
```

---

### Task 6: 下发面板（命令快照与 100 Hz 连发）

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/command.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（`run`/`teach` 屏的 `col-command` 槽位 + 面板模板）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `store.motors`、`store.paramCache`、`store.devInit`、`core/ros.js` 的 `pub()`、`core/format.js` 的 `MOTOR_ID_NONE`、`STREAM_PERIOD_MS`、`TRAJ_MAX_POINTS`、`TRAJ_MAX_MOTORS`。
- Produces: 面板 `command`；模块内 `export function currentTargets()`（供 `teach` 面板的「记录当前命令」取当前目标值，Task 9 消费）。

- [ ] **Step 1: 建面板模板**

标记搬自旧 `index.html:105-124`（`<details class="sendbar">` 整块），id 全部保留：
`sumSend` `cmdRows` `btnAddRow` `btnFillOnline` `btnClearRows` `btnSendOnce` `chkStream` `cmdResult`。

```html
  <template data-panel-template="command">
    <details class="sendbar" data-fold="send" open>
      <summary title="/micro_ros_do/motor_command_array">下发<span class="spacer"></span><span class="sum" id="sumSend">--</span></summary>
      <div class="table-wrap">
        <table class="cmd">
          <thead><tr><th>ID</th><th>position rad</th><th>velocity rad/s</th><th>effort N·m</th><th></th></tr></thead>
          <tbody id="cmdRows"></tbody>
        </table>
      </div>
      <div class="ctrl-row wrap">
        <button id="btnAddRow">+ 添加</button>
        <button id="btnFillOnline" class="secondary">填入在线电机（原位）</button>
        <button id="btnClearRows" class="secondary">清空</button>
        <span class="spacer"></span>
        <button id="btnSendOnce" class="primary">单发一次</button>
        <label class="check"><input type="checkbox" id="chkStream"> 100Hz 连发</label>
      </div>
      <div id="cmdResult" class="result tiny"></div>
    </details>
  </template>
```

- [ ] **Step 2: 写 `js/panels/command.js`**

整段搬这些函数（**逐字，只改数据来源**）：

| 新位置 | 旧行号 |
|---|---|
| `sequence`（原 `seq`，初值 `(Date.now() & 0x7fffffff) >>> 0`） | `app.js:786` |
| `addCmdRow` | `app.js:789-801` |
| `readCmdTargets` | `app.js:802-821` |
| `setCmdResult` | `app.js:822-830` |
| `noteBlockedTargets` | `app.js:831-846` |
| `playbackReadinessError` | `app.js:847-870` |
| `sendSnapshot` | `app.js:871-883` |
| `sendTargets` | `app.js:884-915` |
| `startStream` / `stopStream` | `app.js:1040-1050` |

改动只有三处：

1. `state.motors` / `state.paramCache` / `state.devInit` → `store.*`；`paramCache` 在 store 里叫
   `store.paramCache`，保持字段名不变。
2. 顶层的 `seq`、`streamTimer` 两个模块级变量搬进 `command.js` 的模块作用域（不再跨面板共享）。
3. `history.unshift(...)` → `store.history.unshift(...)` 并 `markDirty(DIRTY.HISTORY)`，
   `if (store.history.length > 200) store.history.pop()` 的上限逻辑搬 `sendTargets` 里的原样。

```js
export function mount(root, ctx) {
  $('btnAddRow').onclick = () => addCmdRow();
  $('btnFillOnline').onclick = () => {
    // 逐字搬 app.js:2570-2578：只填拓扑中在线的电机，目标取当前实测位置
  };
  $('btnClearRows').onclick = () => { stopStream(); $('cmdRows').innerHTML = ''; };
  $('btnSendOnce').onclick = () => { stopStream(); sendSnapshot(); };
  $('chkStream').onchange = (e) => e.target.checked ? startStream() : stopStream();

  addCmdRow(1, 0, 0, 0);      // 等价旧 app.js:2588 的初始行

  const render = (dirty) => {
    if (dirty.has(DIRTY.HISTORY)) {
      $('sumSend').textContent = `${store.history.length} 条已发`;
    }
  };
  render(new Set());
  // 连发定时器与 unmount：切屏必须停连发——否则离开运行屏后 100 Hz 还在发
  return {
    render,
    unmount() { stopStream(); $('chkStream').checked = false; },
  };
}

// 供 teach 面板「记录当前命令」用（面板依赖面板的唯一一处：示教记录的正是下发
// 快照里的目标值）。command 面板未挂载时 #cmdRows 不在 DOM 上，返回空数组而不是抛错。
export function currentTargets() {
  return $('cmdRows') ? readCmdTargets() : [];
}
```

**`unmount()` 必须停连发**：这是重构带来的新要求——旧代码里没有「离开下发区域」这件事，
连发只有手动取消一条路。不停的话，切到别的屏后 100 Hz 仍在发，而按钮已经不在 DOM 上、
用户没法停。这条要写进代码注释。

- [ ] **Step 3: 手动验证**

1. 加行、填在线（原位）、清空、单发一次都正常，`cmdResult` 有结果文本；
2. 勾 100 Hz 连发 → 发起来；**切到「示教」屏 → 连发停止**（在终端 `ros2 topic hz /micro_ros_do/motor_command_array`
   确认频率掉下来）；切回来勾选框是未勾状态；
3. 目标里含 `CONFIG_CONFLICT` 的台 → 点名提示（`noteBlockedTargets`）；
4. 输入一个非有限值（浏览器里手动改成 `NaN`）→ 整包拒收提示。

- [ ] **Step 4: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/command.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 下发面板与 100Hz 连发

新增行为：unmount 时停连发。旧代码里连发只有手动取消一条路，切屏后定时器仍在跑，
而按钮已不在 DOM 上 —— 用户没法停。"
```

---

### Task 7: 曲线、事件日志、命令历史与屏内抽屉

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/curve.js`
- Create: `ros2_ws/src/web_host/web/js/panels/events.js`
- Create: `ros2_ws/src/web_host/web/js/panels/history.js`
- Modify: `ros2_ws/src/web_host/web/js/core/store.js`（`chartPush()` 加环形上限）
- Modify: `ros2_ws/src/web_host/web/js/core/ros.js`（三处 `chartPush` 调用改从 store import）
- Modify: `ros2_ws/src/web_host/web/index.html`（四个屏的抽屉 + 三个面板模板）
- Modify: `ros2_ws/src/web_host/web/js/main.js`（`initDrawers()`）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `store.chartBuf`、`store.chart`、`store.events`、`store.history`、`ctx.ui`。
- Produces: 面板 `curve`、`events`、`history`；store 的 `chartPush(id, ch, val, t)`；main.js 的 `initDrawers(root, screen)`。

- [ ] **Step 1: 曲线缓冲搬进 store 并加环形上限**

`app.js:497-506` 的 `chartPush` 搬到 `core/store.js`，并加容量上限（spec：30 s × 50 Hz = 1500）：

```js
// 曲线数据入缓冲：每通道独立时间轴（缺失 NaN 保留，绘制时跳过）。
// 上限 1500 样本/通道 = 30 s × 50 Hz，恰好是最大时间窗；超过就从头丢。
// 上限放在这里而不是绘制侧：没有曲线面板时缓冲也必须收敛，否则切屏一整天后内存只涨不落。
const CHART_CAP = 1500;

export function chartPush(id, ch, val, t) {
  let b = store.chartBuf.get(id);
  if (!b) {
    b = {t: [], position: [], velocity: [], effort: [], current: []};
    store.chartBuf.set(id, b);
  }
  b[ch].push(Number.isFinite(val) ? val : NaN);
  b.t.push(t);
  if (b.t.length > CHART_CAP) {
    b.t.shift();
    for (const k of ['position', 'velocity', 'effort', 'current']) b[k].shift();
  }
}
```

`ros.js` 里三处 `chartPush(...)`（原 `subscribeAll` 的 `motion` 与 `status` 处理器）改为从
`core/store.js` import。旧的 `chartTrim()`（`app.js:1052-1061`）**删除**——上限已在写入侧统一处理。

- [ ] **Step 2: 抽屉与 tab 的标记**

每个屏的抽屉是屏模板的一部分，形状固定：

```html
      <section class="col-drawer">
        <details class="drawer">
          <summary>观察<span class="spacer"></span><span class="sum drawer-sum">--</span></summary>
          <!-- tab 行放在 summary **之外**：放进去点 tab 会连带触发展开/收起 -->
          <div class="tabs" data-drawer-tabs>
            <button class="tab" data-drawer-tab="curve">曲线</button>
            <button class="tab" data-drawer-tab="events">事件日志</button>
            <button class="tab" data-drawer-tab="history">命令历史</button>
          </div>
          <div class="drawer-body">
            <div data-drawer-panel="curve"><div data-panel="curve"></div></div>
            <div data-drawer-panel="events" hidden><div data-panel="events"></div></div>
            <div data-drawer-panel="history" hidden><div data-panel="history"></div></div>
          </div>
        </details>
      </section>
```

各屏的 tab 集合按 spec 的分发表：

| 屏 | 本任务写什么 |
|---|---|
| `run` | 加 `col-drawer`（`curve` `events` `history` 三个 tab） |
| `teach` | 加 `col-drawer`（`curve` `events` 两个 tab） |
| `device` | 加 `col-drawer`（只有 `events`）——单 tab 时 tab 行整体隐藏 |
| `diag` | 建屏模板：**没有抽屉**，`curve` `history` `events` 三个槽位直接放主区；`imu` 槽位在 Task 11 加 |

单 tab 抽屉隐藏 tab 行（`panels.css`）：

```css
.drawer .tabs { display: flex; gap: var(--gap-sm); }
.drawer .tabs > .tab:only-child { display: none; }
```

**注意**：抽屉里的 `data-panel` 与主区共用同一套面板 id，`main.js` 的挂载循环按
`root.querySelectorAll('[data-panel]')` 遍历，因此抽屉未展开时面板**也要挂载**（渲染到隐藏容器，
成本可接受，且展开时无需重新挂载）。这一点必须写清楚，否则容易出现「展开抽屉是空的」。

- [ ] **Step 3: `main.js` 加 `initDrawers()`**

```js
// 屏内抽屉：开合状态与当前 tab 按屏记忆（web_host.fold.<屏>），与 spec 的约定一致。
function initDrawers(screen) {
  const key = FOLD_OF_SCREEN(screen);
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(key) || '{}'); } catch (e) { saved = {}; }
  const save = () => localStorage.setItem(key, JSON.stringify(saved));

  const details = document.querySelector('#screenRoot details.drawer');
  if (!details) return;
  if (saved.drawerOpen !== undefined) details.open = !!saved.drawerOpen;
  details.addEventListener('toggle', () => { saved.drawerOpen = details.open; save(); });

  const tabs = [...document.querySelectorAll('#screenRoot [data-drawer-tab]')];
  const panels = [...document.querySelectorAll('#screenRoot [data-drawer-panel]')];
  const activate = (name) => {
    for (const p of panels) p.hidden = p.dataset.drawerPanel !== name;
    for (const t of tabs) t.classList.toggle('on', t.dataset.drawerTab === name);
    saved.tab = name; save();
  };
  for (const t of tabs) t.onclick = () => activate(t.dataset.drawerTab);
  activate(saved.tab || (tabs[0] && tabs[0].dataset.drawerTab));
}
```

并在 `showScreen()` 里落一处改动（Task 1 的版本还没有这一步）：

```js
  // 挂载面板之后
  initDrawers(screen);
  markDirty(DIRTY.ALERTS);
```

**tab 行必须放在 `<summary>` 之外**（上面的标记已如此）：放进 summary 里点 tab 会连带触发展开
收起，靠 `stopPropagation` 绕开浏览器默认行为更脆。`.drawer-sum` 由 `initDrawers` 填当前 tab 名（用 class 不用 id —— 抽屉标记在
run/teach/device 三份屏模板里各出现一次，用 id 会重复，见全局约束），这样抽屉收起时
标题栏仍有信息（与旧 `renderFoldSummaries` 的用意一致）。

- [ ] **Step 4: 写三个面板**

| 面板 | 搬移来源 | id |
|---|---|---|
| `curve` | `drawChart`（`app.js:1071-1168`）、`fillChartMotorSelect`（`app.js:1062-1070`）、以及 `init()` 里的曲线控件绑定（`app.js:2598-2610`） | `chart` `chartCh` `chartMotor` `chartWindow` `chartHint` `btnChartPause` `btnChartClear` |
| `events` | `renderEvents`（`app.js:754-761`）、`exportEvents`（`app.js:1025-1034`） | `eventLog` `sumEvents` `btnExportEvents` `btnExportEventsCsv` `btnClearEvents` |
| `history` | `renderHistory`（`app.js:1003-1014`）、`exportHistory`（`app.js:1035-1039`） | `histRows` `sumHist` `btnExportHist` `btnClearHist` |

三处统一改动：

1. `state.events` → `store.events`、`history` → `store.history`、`chartBuf` → `store.chartBuf`、
   `chart` → `store.chart`。
2. `download()` 从 `core/format.js` import。
3. 面板的 `render` 只在相关脏键上工作：`curve` 看 `DIRTY.MOTION | DIRTY.STATUS`（画布重绘，
   `intervalMs: 200`，与旧 `setInterval(drawChart, 200)` 同拍）、`events` 看 `DIRTY.EVENTS`、
   `history` 看 `DIRTY.HISTORY`。

`curve` 的 `unmount()` 无需释放 canvas（没有 WebGL），但**必须清掉自己注册的 `ResizeObserver`**
（画布宽度跟随容器）。

- [ ] **Step 5: 手动验证**

1. 运行屏点开抽屉 → 曲线画出四条通道、切换电机与时间窗正常、暂停/清空正常；
2. 抽屉切到事件日志 → 有事件；导出 JSON 与 CSV 都能下载；清空后为空；
3. 关闭抽屉再切屏再切回 → 抽屉开合状态与上次的 tab 都被记住；
4. **抽屉关闭状态下事件仍在累积**：关掉抽屉，让事件进来，再打开 —— 新事件在（证明面板在
   隐藏容器里也挂着）；
5. 切到设备屏 → 抽屉只有「事件日志」一个 tab，且不显示 tab 行（只有一个时隐藏）。

- [ ] **Step 6: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/curve.js ros2_ws/src/web_host/web/js/panels/events.js \
        ros2_ws/src/web_host/web/js/panels/history.js ros2_ws/src/web_host/web/js/core/store.js \
        ros2_ws/src/web_host/web/js/core/ros.js ros2_ws/src/web_host/web/js/main.js \
        ros2_ws/src/web_host/web/index.html ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 曲线、事件日志、命令历史与屏内抽屉

曲线缓冲搬进 store 并在写入侧封顶 1500 样本/通道（30 s × 50 Hz）：没有曲线面板时
缓冲也必须收敛，否则切屏一整天后内存只涨不落。抽屉开合与当前 tab 按屏记忆。"
```

---

### Task 8: 设备屏（全部参数与写事务）

**Files:**
- Create: `ros2_ws/src/web_host/web/js/core/device.js`
- Create: `ros2_ws/src/web_host/web/js/panels/deviceParams.js`
- Modify: `ros2_ws/src/web_host/web/js/core/ros.js`（掉线清场里复位 `store.conn.autoLoaded`）
- Modify: `ros2_ws/src/web_host/web/index.html`（`device` 屏 + 面板模板 + 抽屉）
- Modify: `ros2_ws/src/web_host/web/js/main.js`（连接建立后触发一次 `loadParams()`）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `store.paramCache`、`store.sysStatus`、`store.devParamSeq`、`store.lastError`、`store.selectedId`、`core/ros.js` 的 `pub()`/`waitSysStatus()`/`waitDevParamSeq()`、`core/store.js` 的 `paramRowIds()`、`devDetailTitle()`、`core/format.js` 的 `devSlotNames()`/`devRecoveryText()`/`fmtParam()`/`PARAM_FIELD`/`SYS_CMD`/`DEV_INIT_STATE`。
- Produces: `core/device.js` 导出 `loadParams()`、`applyParams()`、`saveParamsToFlash()`、`setLimitFromPosition(id, side)`、`setMotorZero(id)`、`clearMotorFault(id)`、`flipDirection(id, cur)`、`hasDirtyParamInput()`；面板 `deviceParams`。

**为什么参数事务单独成 `core/device.js`**：这些函数既被设备屏的按钮调用，也被**连接建立后
的自动读取**调用（旧 `app.js:315-318` 的 `waitSysStatus().then(() => loadParams())`）。自动读取
不能住在面板里——`store.paramCache` 是使能面板列电机列表的依据，切到「运行」屏时设备屏并未
挂载。spec 的 `core/` 文件清单里没有它，Task 12 会把这一条同步回 spec。

- [ ] **Step 1: 写 `core/device.js`**

整段搬这些函数（**逐字，只改 `state` → `store`、`pub()` 从 `core/ros.js` import**）：

| 函数 | 旧行号 |
|---|---|
| `paramReq` / `nextParamReq` | `app.js:1827-1833` |
| `sendParamSet` / `chunkedParamSet` | `app.js:1834-1860` |
| `sendSystemCommand` | `app.js:1861-1872` |
| `waitSysSeq` | `app.js:1873-1884` |
| `SYS_ERR_TEXT` / `lastErrorText` | `app.js:1885-1900` |
| `writeResult` / `setParamResult` | `app.js:1901-1909` |
| `hasDirtyParamInput` | `app.js:2063-2073` |
| `waitSysStatus` / `waitDevParamSeq` | `app.js:2224-2278` |
| `loadParams` | `app.js:2239-2262` |
| `applyParams` | `app.js:2279-2350` |
| `saveParamsToFlash` | `app.js:2351-2386` |
| `setLimitFromPosition` | `app.js:2387-2448` |
| `setMotorZero` | `app.js:2449-2490` |
| `clearMotorFault` | `app.js:2491-2524` |
| `flipDirection` | `app.js:1764-1779` |

`writeResult(id, ok, text)` 改为写 store 上的结果槽（`store.results = new Map()`）并 `markDirty`，
DOM 由面板负责：

```js
// 结果文本进 store 不进 DOM：同一个事务的结果可能来自任一屏的按钮。
export function writeResult(key, ok, text) {
  store.results.set(key, {ok, text, t: Date.now()});
  markDirty(DIRTY.PARAMS);
}
export function setParamResult(ok, text) { writeResult('paramResult', ok, text); }
```

`store.results` 在 `core/store.js` 里加初始化 `results: new Map()`（本任务新增的字段）。**注意 `DIRTY.PARAMS` 会被
多个面板监听**，因此 `params` 脏帧里设备屏与使能屏都会重绘——这正是旧代码 `renderParamRows()`
一次重画两张表的行为，语义不变。

- [ ] **Step 2: 建 `device` 屏与面板模板**

`device` 屏（无臂视图，参数表吃满宽度）：

```html
  <template data-screen="device">
    <div class="screen device">
      <section class="col-full"><div data-panel="deviceParams"></div></section>
      <section class="col-drawer">
        <details class="drawer">
          <summary>事件<span class="spacer"></span></summary>
          <div class="drawer-body">
            <div data-drawer-panel="events"><div data-panel="events"></div></div>
          </div>
        </details>
      </section>
    </div>
  </template>
```

面板模板标记搬自旧 `index.html:129-160`（`<section class="bottom">` 那块），id 全部保留：
`sumDev` `paramSummary` `btnParamLoad` `btnParamApply` `btnParamSave` `chkAllParams` `paramResult` `paramRows`。

```html
  <template data-panel-template="deviceParams">
    <details class="card" data-fold="dev" open>
      <summary>全部参数 <span class="spacer"></span><span class="sum" id="sumDev">未读取</span></summary>
      <div id="paramSummary" class="summary tiny">连接后点「读取」</div>
      <div class="ctrl-row wrap">
        <button id="btnParamLoad" class="tiny-btn">读取</button>
        <button id="btnParamApply" class="tiny-btn primary">下发修改</button>
        <button id="btnParamSave" class="tiny-btn">保存 W25</button>
        <span class="spacer"></span>
        <label class="check tiny"><input type="checkbox" id="chkAllParams"> 显示全部</label>
      </div>
      <div id="paramResult" class="result tiny"></div>
      <div class="table-wrap">
        <table class="params">
          <thead>
            <tr>
              <th class="num">ID</th><th>型号</th><th>状态</th><th>来源</th>
              <th class="num">pos_min<small>rad</small></th><th class="num">pos_max<small>rad</small></th>
              <th class="num">当前位置<small>rad</small></th>
              <th>限位设置</th>
              <th class="num">vel_max<small>rad/s</small></th><th class="num">torque_max<small>A</small></th>
              <th>限位保护</th><th>方向</th><th></th>
            </tr>
          </thead>
          <tbody id="paramRows">
            <tr class="empty"><td colspan="13">连接后点「读取」载入当前参数</td></tr>
          </tbody>
        </table>
      </div>
    </details>
  </template>
```

- [ ] **Step 3: 写 `js/panels/deviceParams.js`**

搬 `renderDevRows`（`app.js:1925-2050`，约 125 行，含行内输入框、设零、方向、限位保护、
来源列、状态列、悬停原值）、`updateDevCurPos`（`app.js:2051-2062`）、`renderParamSummary`
（`app.js:2169-2223`）。改动：

1. `state.paramCache` → `store.paramCache`、`state.sysStatus` → `store.sysStatus`、
   `state.devInit` → `store.devInit`、`state.lastError` → `store.lastError`。
2. 事务函数从 `core/device.js` import。
3. `paramRowIds()` → `paramRowIds($('chkAllParams').checked)`。
4. `setParamResult` 的显示改读 `store.results.get('paramResult')`。
5. **保留防抹除保护**：`render` 里先判 `hasDirtyParamInput()`，为真就跳过重绘。旧代码是在
   `devParams` 订阅处理器里判的（`app.js:432-434`），现在订阅层不认识面板，判断必须挪到这里。
   漏判的后果与使能表一样：操作员正在敲的值被下一帧重建抹掉。

```js
export function mount(root, ctx) {
  $('chkAllParams').onchange = () => render(new Set([DIRTY.PARAMS, DIRTY.SELECTED]));
  $('btnParamLoad').onclick = () => loadParams();
  $('btnParamApply').onclick = () => applyParams();
  $('btnParamSave').onclick = () => saveParamsToFlash();

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.PARAMS) || dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.TOPOLOGY)
          || dirty.has(DIRTY.CONN))) return;
    if (hasDirtyParamInput()) return;         // 有未下发的手改就不重建
    renderParamSummary();
    renderDevRows();
    renderResultFromStore();
  };
  render(new Set([DIRTY.PARAMS, DIRTY.CONN]));
  return {render, intervalMs: 200, unmount() {}};
}
```

`renderResultFromStore()` 本身很短，写进本文件：

```js
// 结果文本住在 store 里（core/device.js 的 writeResult 写它），因为同一批事务
// 的按钮分布在设备屏与运行屏两处。这里只负责把它画出来。
function renderResultFromStore() {
  const r = store.results.get('paramResult');
  const el = $('paramResult');
  el.classList.toggle('ok', !!(r && r.ok));
  el.classList.toggle('err', !!(r && !r.ok));
  el.textContent = r ? r.text : '';
}
```

`「当前位置」列的就地刷新`：旧代码用独立的 `setInterval(updateDevCurPos, 200)`（`app.js:1714`），
新结构里改为 `render` 内对已有行做局部更新——`intervalMs: 200` 已经保证 5 Hz，不需要另开定时器。
**不要在 render 里重建整表再更新当前位置**，那会抹掉正在编辑的输入框（这正是 `updateDevCurPos`
当初被单独拆出来的原因，`app.js:2051` 上方的注释记着）。

- [ ] **Step 4: 连接建立后自动读取参数**

`main.js` 里把 `loadParams` 接到连接事件上（**不是**接到设备屏的 `mount` 上）：

```js
// 连上后自动读取一次：使能面板的电机列表来自 paramCache，设备屏不挂载时也要有。
// 先等首帧 system_status 确认固件侧已在总线上（订阅刚建时 DDS 发现可能还没完成，
// REPORT 太早发出可能白等）；真没固件时由 loadParams 自己的超时兜底。
onDirty((dirty) => {
  if (dirty.has(DIRTY.CONN) && store.conn.connected && !store.conn.autoLoaded) {
    store.conn.autoLoaded = true;
    waitSysStatus().then(() => loadParams());
  }
  ...
});
```

`store.conn.autoLoaded` 在 `ros.js` 的掉线清场里要一起复位（与 `store.sysStatus.seen = false` 同处），
否则重连后不会重新读取。

- [ ] **Step 5: 手动验证（需真机或 firmware HIL）**

1. 设备屏读表：来源列、状态列、7 位有效数字、未保存台标黄都正常；
2. 改一项限值 → 「下发修改」→ `paramResult` 有结果，行状态随之更新；
3. 「保存 W25」→ 保存进行中显示、`save_in_progress` 为真时按钮禁用；
4. 行内「设零」（需失能态）与「方向」翻转各试一次，事件日志出现 `CONFIG_RESULT`；
5. 未核对的台（`CONFIG_CONFLICT`）点「策略」被拒，事件日志 reason=2；
6. 断连后所有写按钮禁用（`body.offline` + 按钮 `disabled`）。

- [ ] **Step 6: 提交**

```bash
git add ros2_ws/src/web_host/web/js/core/device.js ros2_ws/src/web_host/web/js/panels/deviceParams.js \
        ros2_ws/src/web_host/web/js/core/store.js ros2_ws/src/web_host/web/js/main.js \
        ros2_ws/src/web_host/web/index.html ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 设备屏与参数写事务

参数读写事务单独成 core/device.js：既被设备屏按钮调用，也被连接建立后的自动读取调用 ——
使能面板的电机列表来自 paramCache，设备屏不挂载时也要有。

防抹除保护从订阅处理器挪进面板 render：订阅层已不认识面板，正在编辑的输入框仍不能被
下一帧重建抹掉。"
```

---

### Task 9: 示教屏

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/teach.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（`teach` 屏补齐槽位 + 面板模板 + 抽屉）
- Modify: `ros2_ws/src/web_host/web/js/core/ros.js`（`trajectoryState` 订阅改为写 `store.traj` 并 `markDirty(DIRTY.TRAJ)`）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `panels/command.js` 的 `currentTargets()`、`core/ros.js` 的 `pub()`/`primeTrajectorySnapshot()`、`core/format.js` 的 `TRAJ_MAX_POINTS`/`TRAJ_MAX_MOTORS`/`TRAJ_STATE_TIMEOUT_PAD_MS`、`core/store.js` 的 `targetOf()`。
- Produces: 面板 `teach`；`store.traj` 的字段 `{activeTrajectoryId, waitingState, segmentIndex, segmentCount}`。

- [ ] **Step 1: `trajectoryState` 订阅改为写 store**

`ros.js` 里现在的处理器直接调 `teachOnTrajectoryCompleted(m)`（旧 `app.js:436-446`）——订阅层反向
依赖面板，必须拆掉。改为：

```js
  sub('trajectoryState', (m) => {
    // 轨迹状态是 10Hz best-effort 回执。回执里的 trajectory_id 就是**这一段**的 id：
    // 长示教被切成多条独立轨迹串行下发，页面等这一段的 completed 才发下一段。
    store.traj.last = m;
    markDirty(DIRTY.TRAJ);
  });
```

「等这一段完成」的判定搬进 `teach` 面板的 `render`：`activeTrajectoryId`、`waitingState` 两个标志
本来就是面板自己的播放状态机（`store.traj` 里保留它们，因为回执要从订阅侧比对）。

- [ ] **Step 2: 建面板模板与补齐 teach 屏**

标记搬自旧 `index.html:293-315`，id 全部保留：`btnTeachRecord` `btnTeachAdd` `btnTeachExport`
`btnTeachImport` `btnTeachPlay` `btnTeachPause` `btnTeachStep` `chkTeachLoop` `teachStatus` `teachRows`。
按 spec 补一个分段进度条（新增 id）：

```html
      <div class="ctrl-row wrap">
        <button id="btnTeachRecord" class="tiny-btn primary">记录当前命令</button>
        <button id="btnTeachAdd" class="tiny-btn">+ 空点位</button>
        <button id="btnTeachExport" class="tiny-btn">导出 JSON</button>
        <button id="btnTeachImport" class="tiny-btn">导入 JSON</button>
        <span class="spacer"></span>
        <button id="btnTeachPlay" class="primary">播放</button>
        <button id="btnTeachPause" class="secondary">暂停</button>
        <button id="btnTeachStep" class="secondary">单步</button>
        <label class="check"><input type="checkbox" id="chkTeachLoop"> 循环</label>
        <span class="muted tiny" id="teachStatus">空闲</span>
      </div>
      <!-- 分段进度：长示教切成多条独立轨迹串行下发，段号必须看得见 -->
      <div class="ctrl-row wrap tiny muted" id="teachSegment">段 --/--</div>
```

`teach` 屏补齐 `col-side`（`readout` + `policy`）、`col-command`（`command`）、`col-teach`（`teach`）
四个槽位。`col-drawer`（`curve` + `events`）已在 Task 7 就位，**不要重复添加**。

- [ ] **Step 3: 写 `js/panels/teach.js`**

搬 `app.js:1283-1711` 的示教全段（约 430 行）与 `app.js:1015-1024` 的 `download`（已进 format）。
函数清单与行号：

`teach`（状态对象，`app.js:1284-1289`）、`setTeach`（1290-1295）、`teachAddRow`（1296-1306）、
`teachRenumber`（1307-1312）、`teachReadRows`（1313-1335）、`teachStopTimer`（1336-1342）、
`teachRowsToTrajectory`（1343-1363）、`segmentRowsToTrajectory`（1364-1382）、`rowsMove`（1383-1396）、
`pickSegmentEnd`（1397-1437）、`buildTrajectorySegments`（1438-1474）、`teachPlay`（1475-1501）、
`teachSendTrajectorySegment`（1502-1539）、`teachOnTrajectoryCompleted`（1540-1573）、
`teachPause`（1574-1586）、`teachStep`（1587-1621）、`teachRecordCmd`（1622-1629）、
`teachExport`（1630-1638）、`fmtTeachNum`（1639-1644）、`targetsToText`（1645-1653）、
`rowsFromTrajectory`（1654-1679）、`teachImport`（1680-1711）。

改动四处：

1. `state.motors` → `store.motors`；`history` → `store.history`；`teach.*` 仍是本面板私有状态
   （它不跨屏），但 `activeTrajectoryId` / `waitingState` 写进 `store.traj` 供订阅侧比对。
2. `teachRecordCmd` 里的「取当前命令」改为 `import { currentTargets } from './command.js'` ——
   这是**面板依赖面板**的唯一一处，是必要的：示教记录的正是下发快照里的目标值。若 `command`
   面板未挂载（理论上 teach 屏总挂着它），`currentTargets()` 读 `$('cmdRows')` 会拿到空表，
   因此 `currentTargets()` 要在面板未挂载时返回空数组而不是抛错。
3. `teachOnTrajectoryCompleted` 不再由订阅直接调用，改由 `render` 在 `DIRTY.TRAJ` 时检查回执。
   **必须保留原判据**（`trajectory_id` 等于当前段的 id 且 `completed` 为真），注释照搬。
4. `unmount()` 必须 `teachStopTimer()` 并 `cancelTrajectory()`（若正在播放）。理由与下发面板停连发
   相同：切屏后播放状态机不能在后台继续发轨迹。

**分段串行的三条不变量**（搬移时不许改动，它们是台架实测换来的）：每段 ≤256 点、每段时间从
t=0 重新起算、段界重复一行保证位置连续、每段声明整条时间线的目标集合。`app.js:1438-1474` 与
`app.js:1502-1539` 里的注释（含 09-22 与 10-08 两次调整的原因）**逐字保留**。

- [ ] **Step 4: 手动验证（需真机）**

1. 短示教（≤256 点）播放完成，末点保持；点「暂停」取消并软停；
2. 长示教（如 2148 点）切成多段串行下发，`teachSegment` 显示「段 3/9」，每段等回执才发下一段；
3. 导入两种 JSON（本页行数组、`MotorTrajectory` 话题载荷）都能识别；
4. **播放中切屏** → 播放停止、轨迹取消（终端确认不再有 `motor_trajectory` 发布）。

- [ ] **Step 5: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/teach.js ros2_ws/src/web_host/web/js/core/ros.js \
        ros2_ws/src/web_host/web/index.html ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 示教屏与分段进度

trajectory_state 订阅不再反向调用面板：只写 store.traj 并标脏，「等这一段完成」的判定
搬进面板的播放状态机。分段串行的三条不变量与两次调整的原因注释逐字保留。

新增：分段进度条（段 n/N）；unmount 停止播放并取消轨迹。"
```

---

### Task 10: 仿真屏

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/sim.js`（从 `web/simulation.js` 改造）
- Modify: `ros2_ws/src/web_host/web/index.html`（`sim` 屏 + 面板模板）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: 无 store 依赖（仿真启动器走 `/api/simulation/*` HTTP 接口，与 rosbridge 无关）。
- Produces: 面板 `sim`。

- [ ] **Step 1: 建 `sim` 屏与面板模板**

标记搬自旧 `index.html:180-238`，id 全部保留：`simSummary` `simMode` `simTransport` `simModel`
`simVcanFields` `simVcanName` `simSocketFields` `simInterface` `simDriverCanField` `simDriverInterface`
`simSlcanFields` `simTty` `simDriverTtyField` `simDriverTty` `simHostBusHint` `simPhysicalGuard`
`simBusIsolated` `simFirmwareHint` `btnSimStart` `btnSimStop` `btnSimRefresh` `simState` `simResult` `simLog`。

```html
  <template data-screen="sim">
    <div class="screen sim"><section class="col-full"><div data-panel="sim"></div></section></div>
  </template>
```

- [ ] **Step 2: 把 `simulation.js` 改造成 `panels/sim.js`**

`simulation.js` 已经是结构清晰的 IIFE（`ui` 映射 + 22 个函数 + 一个 1.8 s 轮询）。改造只有三处：

1. 包成 `export function mount(root, ctx)`，`ui` 映射与全部内部函数移入 `mount` 作用域内
   （或保留在模块作用域并延迟初始化 `ui`——**推荐移入 `mount`**，因为切屏会反复挂载，
   模块级 `ui` 会指向已被卸载的 DOM）。
2. `window.setInterval(refreshStatus, 1800)`（旧 `simulation.js:334`）改由 `mount` 持有句柄，
   `unmount()` 里 `clearInterval`。**这是必须的**：旧代码是页面级单例，切屏后轮询会被重复注册。
3. 文件头部的注释（「仿真启动器」相关说明）与全部行为逐字保留。

- [ ] **Step 3: `main.js` 的挂载方式**

`sim` 屏的面板不在 `#screenRoot` 的常驻区域里，也不与 rosbridge 相关；挂载循环已经统一处理，
不需要额外接线。但要确认 `mountPanel` 的**动态 import 路径**对 `panels/sim.js` 有效。

- [ ] **Step 4: 手动验证**

1. 仿真屏列出模型、CAN 接口、串口；
2. 选 `vcan` + 主机闭环 → 启动 → `simLog` 出日志、`simState` 变「启动进程运行中」；
3. 选 `SocketCAN` 时出现物理总线隔离确认勾选框，不勾选不能启动（前端与后端双重拦截）；
4. **切屏再切回** → 状态与日志仍在（`store` 无关，靠后端 `/api/simulation/status` 恢复），
   `simLog` 不重复追加；切屏后 1.8 s 轮询不叠加（DevTools Network 里 `/api/simulation/status`
   的频率仍是每 1.8 s 一次）。

- [ ] **Step 5: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/sim.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 仿真启动器改为可挂载面板

simulation.js 的 ui 映射与全部逻辑移入 mount 作用域，1.8s 轮询由 mount 持有句柄并在
unmount 清理 —— 旧代码是页面级单例，切屏会重复注册轮询。"
```

---

### Task 11: 诊断屏与 IMU 面板

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/imu.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（`diag` 屏 + `imu` 面板模板）
- Modify: `ros2_ws/src/web_host/web/css/panels.css`

**Interfaces:**
- Consumes: `store.imu`、`store.temp`、`store.tempLast`、`store.chartBuf`。
- Produces: 面板 `imu`；`diag` 屏（组装 `curve` `history` `imu` `events` 四个已存在的面板）。

- [ ] **Step 1: 写 `js/panels/imu.js`**

搬 `drawImu3d`（`app.js:1170-1228`）、`drawHorizon`（`app.js:1229-1282`）、`renderImu`
（`app.js:762-782`），以及 IMU Hz 的 1 s 定时器（`app.js:1729-1735`）。

两处改动：

1. `state.imu` / `state.temp` / `state.connected` → `store.imu` / `store.temp` / `store.conn.connected`。
2. Hz 定时器由 `mount` 持有并在 `unmount` 清除；两个 canvas 的重绘由 `render` 在
   `DIRTY.IMU` 时驱动（`intervalMs: 200`，与旧 `setInterval(renderImu, 200)` 同拍）。

`unmount()` 不需要释放 WebGL 资源——两个 canvas 都是 2D context。但要 `clearInterval`。

- [ ] **Step 2: 建 `diag` 屏与 `imu` 面板模板**

`imu` 面板标记搬自旧 `index.html:332-351`，id 全部保留（`imuHz` `valRoll` `valPitch` `valYaw`
`valTemp` `imuAge` `imu3d` `valRollBig` `valPitchBig` `valYawBig` `valQuat` `attitudeHorizon`）。

```html
  <template data-screen="diag">
    <div class="screen diag">
      <section class="col-curve"><div data-panel="curve"></div></section>
      <section class="col-history"><div data-panel="history"></div></section>
      <section class="col-imu"><div data-panel="imu"></div></section>
      <section class="col-events"><div data-panel="events"></div></section>
    </div>
  </template>
```

诊断屏**没有抽屉**：曲线、命令历史、IMU、事件日志都是主区常驻（spec 的分发表如此）。
因此 `curve` / `events` / `history` 三个面板在本屏的 `data-panel` 占位直接放在主区，
`main.js` 的挂载循环照常处理，不需要 `data-drawer-panel` 包壳。

- [ ] **Step 3: 手动验证**

1. 诊断屏四块面板都在，曲线最大；
2. IMU 有数据时两个 canvas 都在画（3D 姿态随机体转动、人工地平仪随 roll/pitch 倾斜），
   三组角度数值一致（左侧小数值、右侧大数值、四元数）；
3. 无 IMU 数据时显示 `--`（不是 0），`imuHz` 显示 `-- Hz`；
4. 切走再切回，两个 canvas 仍能继续画（说明定时器与 rAF 没有随 unmount 一起废掉）；
5. DevTools 里确认切屏后不再有 IMU 定时器在跑。

- [ ] **Step 4: 提交**

```bash
git add ros2_ws/src/web_host/web/js/panels/imu.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/css/panels.css
git commit -m "feat(web_host): 诊断屏与 IMU 面板

诊断屏四个面板都是主区常驻，没有抽屉。IMU 的 1s Hz 定时器由 mount 持有并在 unmount 清除。"
```

---

### Task 12: 说明浮层、删除旧前端与文档同步

**Files:**
- Create: `ros2_ws/src/web_host/web/js/panels/notes.js`
- Delete: `ros2_ws/src/web_host/web/app.js`
- Delete: `ros2_ws/src/web_host/web/arm3d.js`
- Delete: `ros2_ws/src/web_host/web/simulation.js`
- Delete: `ros2_ws/src/web_host/web/style.css`
- Modify: `ros2_ws/src/web_host/web/index.html`（`notes` 面板模板 + 外壳里挂载点）
- Modify: `ros2_ws/src/web_host/web/js/main.js`（挂载 `notes` 到 `#helpBody`）
- Modify: `ros2_ws/src/web_host/README.md`（整篇按新结构改写）
- Modify: `docs/superpowers/specs/2026-10-09-web-host-frontend-restructure-design.md`（把 `core/device.js` 补进文件清单）
- Modify: `CLAUDE.md`（web_host 小节：构建注意里的 `arm3d.js` 例改为新文件、页面结构描述）
- Modify: `docs/adr/README.md`（0010 行的「当前适用性」补一句实现已落地）

**Interfaces:**
- Consumes: `ctx.ui`。
- Produces: 面板 `notes`（挂载到 `#helpBody`，不进任何屏）。

- [ ] **Step 1: 写 `js/panels/notes.js`**

面板标记搬自旧 `index.html:317-330`（`<section class="notes">` 那 9 条），以及旧
`index.html:354-356` 的页脚。内容**逐字保留**——那是写给操作员看的，不是实现细节。

```html
  <template data-panel-template="notes">
    <ul class="notes">
      <!-- 9 条安全与操作注记，逐字搬自旧 index.html:320-329 -->
    </ul>
    <div class="foot muted tiny">
      rosbridge ws·9090 ｜ 页面 http·8080 ｜ 仅限内网，rosbridge 无鉴权
    </div>
  </template>
```

`notes.js` 的 `mount()` 只做一件事：把模板克隆进 `#helpBody`。没有 render、没有定时器。

```js
// 说明：长解释集中在这里与各控件的 title，面板上只留数字与状态色。
// 挂载到说明浮层（#helpBody），不进任何屏，也不进路由。
export function mount(root) {
  const tpl = document.querySelector('template[data-panel-template="notes"]');
  root.append(tpl.content.cloneNode(true));
  return {render() {}, unmount() { root.replaceChildren(); }};
}
```

`main.js` 的 `boot()` 里加 `mountPanel('notes', $('helpBody'))`。

**注意**：`notes` 的挂载点 `#helpBody` 不在 `#screenRoot` 里，因此它与 `wing` 一样不随屏挂载/
卸载——契约测试的 `SHELL_ONLY_PANELS` 已豁免这两个。

- [ ] **Step 2: 删除旧前端**

```bash
cd /home/ly/robot_ws/ros2_ws/src/web_host/web
git rm app.js arm3d.js simulation.js style.css
```

删除前先确认三件事：

1. `grep -rn "app\.js\|arm3d\.js\|simulation\.js\|style\.css" web/ ../README.md` 已无活引用
   （`index.html` 里的 `<script src>` 在 Task 1 就换成了 `js/main.js`）；
2. 新页面的「功能对等清单」17 项已在台架上逐条走过（本任务 Step 4）；
3. `git status` 里没有别人的未提交改动被卷进来。

- [ ] **Step 3: 更新文档**

- `web_host/README.md`：按新结构整篇改写——文件结构表、五屏与 14 面板的分发表、`?model=`/`?joints=`
  URL 参数（保留）、构建注意（把 `arm3d.js` / `simulation.js` 的例子换成 `js/panels/` 与 `css/`
  这类新路径）、验证命令里的资源清单（`arm3d.js` → `js/main.js`）。
- `spec`：`core/` 清单补上 `device.js`（理由见 Task 8）；把「不设 `screens/` 目录」这条写进决策。
- `CLAUDE.md`：web_host 小节里「`web/` 下**新增文件**（如 `arm3d.js`、`vendor/`）」的例子改为
  「`js/panels/`、`css/`」，并补一句「前端为外壳/屏/面板三层，DOM 单一来源在 `index.html`，
  契约测试 `test_web_assets.py` 校验 `$('id')` 引用与 `data_files` 覆盖」。
- `docs/adr/README.md`：0010 行的「当前适用性」保持不变（决策未变），只把日期列的表头日期更新。

- [ ] **Step 4: 全量验证**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select web_host
colcon test --packages-select web_host && colcon test-result --verbose
```

期望：`test_web_assets.py` 12 项与 `test_agent_manager.py` 全绿。

再做 spec「验证标准」的 1–8 条，其中台架部分按**功能对等清单 17 项**逐条打勾：

1. 顶栏连接（地址记忆、刷新自动连、自动重连）
2. 控制板与 Agent（by-id 优先枚举、扫描、启动并连接、断开、状态文案）
3. 状态 chips 五项
4. 关节索引（点击选中、与臂上小球双向联动）
5. 臂视图（URDF+STL、幽灵、九态着色、轨迹期不画幽灵、拖拽缩放点选、加载失败浮标、两个 URL 参数）
6. 选中关节读数 9 项
7. 使能与增益（逐台下发、代数递增、本行筛选、显示全部、CONFIG_RESULT 可见）
8. 下发（增删行、填入在线原位、单发、100 Hz、点名提示、NaN 拒收、切屏停连发）
9. 曲线（四通道、多电机、三档窗口、暂停、清空）
10. 事件日志（列表、导出 JSON、导出 CSV、清空）
11. 命令历史（表格、导出、清空）
12. 示教（点位表、记录、单步、循环、播放/暂停、两种 JSON 导入、导出、≤256 点、
    超长串行分段、段界位置连续、完成后末点保持、暂停取消、分段进度可见）
13. 设备参数（总览条与不一致明细、读取、下发修改、保存 W25、显示全部、来源列、状态列、
    悬停原值、设零、方向、限位保护、限位取自位置、7 位有效数字、未保存标黄）
14. 仿真（三种控制端 × 三种传输、模型与端点枚举、物理总线确认双重、启动/停止/刷新、日志）
15. IMU（数值、3D 姿态、人工地平仪、Hz）
16. 折叠记忆（抽屉与状态翼）
17. 说明（9 条注记、单位约定、话题清单、页脚）

多窗口：两个标签页分别停在 `#/run` 与 `#/diag`，各自数据正常。

- [ ] **Step 5: 提交**

```bash
cd /home/ly/robot_ws
git rm ros2_ws/src/web_host/web/app.js ros2_ws/src/web_host/web/arm3d.js \
       ros2_ws/src/web_host/web/simulation.js ros2_ws/src/web_host/web/style.css
git add ros2_ws/src/web_host/web/js/panels/notes.js ros2_ws/src/web_host/web/index.html \
        ros2_ws/src/web_host/web/js/main.js ros2_ws/src/web_host/README.md \
        docs/superpowers/specs/2026-10-09-web-host-frontend-restructure-design.md \
        docs/adr/README.md CLAUDE.md
git commit -m "refactor(web_host): 删除旧单页前端并同步文档

五屏与说明浮层全部搬迁完成，功能对等清单 17 项已在台架逐条走过，删除 app.js /
arm3d.js / simulation.js / style.css。README 与 CLAUDE.md 的构建注意改为新路径。

spec 补记两处实施期决策：core/ 增加 device.js（参数事务被设备屏与连接自动读取共用）、
不设 screens/ 目录（屏由 index.html 的 <template> 定义）。"
```

---

## 完成标准

全部 12 个任务完成后：

- `web/` 下只剩 `index.html`、`css/{tokens,shell,panels}.css`、`js/{main.js,core/*,panels/*}`、
  `vendor/`、`roslib.min.js`；
- `web_host/README.md` 描述的结构与实际文件一致；
- `colcon build` + `colcon test --packages-select web_host` 全绿；
- spec 的「验证标准」8 条与「功能对等清单」17 项都已在台架上走过；
- ADR 0010 的三条硬约束仍然成立：无构建链、DOM 单一来源、测试零 Node 依赖。
