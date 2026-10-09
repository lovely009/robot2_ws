# web_host 前端重构设计（控制台范式）

- 日期：2026-10-09
- 状态：待批准
- 范围：`ros2_ws/src/web_host/web/` 的全部前端静态资产（HTML / CSS / JS）与 `web_host/README.md`；
  新增一个 pytest 静态契约测试。不改后端 Python 模块接口，不改任何话题、消息或固件契约，不新增功能。
- ADR：[0010 web_host 前端的无构建链分层](../../adr/0010-web-host-frontend-layering-without-build-chain.md)

## 背景

现状是单页 + 8 个抽屉：`index.html` 363 行标记、`app.js` 2613 行脚本、`style.css` 450 行样式，
外加已经是 ES module 的 `arm3d.js`（207 行，three.js 臂视图）与 `simulation.js`（335 行，仿真
启动器）。15 块功能面板按「页面有地方放」平铺。

与实际使用者确认为四个日常场景：真机台架调试（核对 → 使能 → 下发 → 看反馈）、示教录制与回放、
仿真验证、参数整定与故障处理。据此确认的痛点有四条，本轮全部处理：

1. **常用操作埋在抽屉里** —— 下发、使能/增益、示教、参数都在底部 `.drawer` 内，每次要展开与滚动。
2. **没有导航** —— 8 个抽屉平铺，找东西靠记忆。
3. **信息密度均匀** —— 控制板选串口、臂视图、9 个读数、事件列表同时在场，常态信息与深度信息一样重。
4. **窄屏只能顺序堆叠** —— 固定三栏在窄屏退化为一条更长的页。

同时确认的范围与约束：**保持无构建链**（原生 ES module + import map，不引框架）；**测试走 Python
侧静态契约**（不引 Node 测试框架，理由见 ADR 0010 决策 8）。

## 决策

### 骨架：外壳 / 屏 / 面板

```text
┌────────────────────────────────────────────────────────────────┐
│ 顶栏（全局）：品牌 · 状态 chips · rosbridge 地址 · 控制板串口 ·   │
│              扫描 · 启动并连接 · 断开 · Agent 状态 · 说明 ?      │
├────────┬──────────────────────────────────────┬────────────────┤
│ 左导航 │ 主区：当前屏（含各自的次级抽屉）        │ 状态翼          │
│ 5 项   │                                      │ 展开 168px      │
│ 56–72px│                                      │ ↔ 收起 34px     │
└────────┴──────────────────────────────────────┴────────────────┘
```

- **外壳**：三处常驻。连接与 Agent 控制留在顶栏（它们是每一屏都要用的），不随屏切换。
- **屏**：路由单位，共 5 个。屏不是模块——它的标记是 `index.html` 里的
  `<template data-screen="run">`，id 在 `core/router.js` 的 `SCREENS` 常量与左导航项的
  `data-goto` 中重复出现，由测试校验三处一致。
- **面板**：可挂载组件，共 14 个，各自一个 `<template data-panel-template="...">` 加一个
  `js/panels/<id>.js`。面板模板**定义一次**，被多个屏引用时按屏克隆。

### 五屏与面板分发

分发原则是「干活时手边要有」，而不是「按工具分类」。曲线与事件日志因此成为**跨屏复用的同一个
组件**：数据在 store，挂载到多处，切屏不丢缓冲。

| 屏 | `#/` | 主区面板 | 次级抽屉（默认收起） |
|---|---|---|---|
| 运行 | `run` | `jointList` `arm` `readout` `policy` `command` | `curve` `events` `history` |
| 示教 | `teach` | `jointList` `arm` `teach` `policy` `command` | `curve` `events` |
| 设备 | `device` | `deviceParams` | `events` |
| 仿真 | `sim` | `sim` | — |
| 诊断 | `diag` | `curve` `history` `imu` `events` | — |

- 「运行」是台架调试主战场，核对 → 使能 → 下发 → 看反馈四步全在一屏内；事件日志与曲线放在它的
  抽屉里，下发被拒不用切屏就能查原因。
- 「设备」屏**不放臂视图**，13 列参数表吃满宽度；设零、方向翻转、限位保护、清故障是
  `deviceParams` 的行内操作，不单独成面板。
- 「示教」屏保留 `command`：示教的「记录当前命令」记录的正是下发快照里的目标值。
- 「诊断」屏是深挖用的窄屏，曲线放最大，命令历史与 IMU 在旁。
- 「说明」不进导航（见下文浮层）。

### 状态翼（W3：可折叠 + 记忆）

展开态 168px（内容宽）：

1. 臂缩略（实际姿态 + 幽灵目标），点击跳「运行」屏；
2. 全局计数：在线 n/N、运行 n、冲突 n、故障 n、运动流 Hz；
3. **告警清单**：每条可点，跳转到该告警所属的屏并选中那台电机（例如「ID4 参数未核对」→ 设备屏 +
   选中 ID4）；
4. 最近一条事件（单行，超长截断）。

收起态 34px：竖排状态点 + 在线比 + 告警计数 + 展开钮。折叠状态存 `localStorage`，沿用现有
`web_host.fold` 的记忆机制。

`localStorage` 键：状态翼 `web_host.wing`；屏内抽屉开合与当前 tab `web_host.fold.<屏 id>`。

### 视觉度量（色板不变，只换度量）

保留现有色板与气质（`--bg:#0a0e16`、`--accent:#3d9bff`，等宽数字、状态用色点、面板不写句子）。
改的是度量：正文 12px → 12.5px，行内边距 3px/6px → 7px/11px，面板内边距 → 12px，圆角 5px → 9px，
栅格间距 → 8px。分层的落点是**间距与分组**，不靠加颜色——现有密度下折叠区与常驻区视觉等价，
只能靠颜色区分，那会与现有原则冲突。

### 文件与模块边界

```text
ros2_ws/src/web_host/
  web/
    index.html          外壳 + 14 个面板 <template> + 5 个屏 <template>（约 700 行）
    css/
      tokens.css        色板（沿用现状）+ 度量 token + 断点
      shell.css         顶栏 / 左导航 / 状态翼 / 浮层 / 响应式
      panels.css        面板通用：卡片、KV 栅格、表格、按钮、结果文本、日志
    js/
      main.js           入口：建 store、初始化 ros 与 agent、装路由与外壳、起 rAF 循环
      core/
        ros.js          rosbridge 连接 / 重连 / 发布 / 订阅注册表
        store.js        共享状态 + 脏标记 + 订阅
        router.js       SCREENS 常量、hash 路由、屏挂载与卸载
        agent.js        micro-ROS Agent 进程管理（现有 HTTP API 调用搬移）
        format.js       $ / fmt / 单位与时间文本
      panels/           14 个面板模块（见上文表）
    vendor/             three + urdf-loader（不动）
    roslib.min.js       （不动）
  web_host/             后端 Python：不改
  test/
    test_agent_manager.py   保留
    test_web_assets.py      新增（静态契约）
```

`web/` 下新增 `css/` 与 `js/` 两个**真实目录**（不是符号链接）：`_web_data_files()` 的
`os.walk(followlinks=True)` 会正常下钻，`web/` 内不得出现目录符号链接这条约束不变。

### 面板契约

```js
// js/panels/<id>.js —— 文件名即面板 id，与 index.html 的
// <template data-panel-template="<id>"> 一致
export function mount(root, ctx) {
  // root: 已克隆并插入 DOM 的面板根元素
  // ctx : { store, ros, agent, fmt, ui }
  //   ui.selectMotor(id)   选中关节（写 store.selected）
  //   ui.gotoScreen(id)    路由跳转
  //   ui.toast(text, level)
  return {
    // 由 main.js 的单一 rAF 循环调用；dirty 是自上次渲染后变脏的 key 集合
    render(dirty) {},
    // 解绑事件、释放 canvas / three 资源；**不得清空 store 数据**
    unmount() {},
  };
}
```

### store 契约

```js
export const store = {
  conn:     { connected: false, url: '', agent: 'stopped', lastMsgAt: 0 },
  topology: [],                 // motor_topology
  motion:   {                   // motion_states
    latest: new Map(),          // id -> { pos, vel, eff, cur, temp, t, source }
    ring:   new Map(),          // id -> 环形缓冲，容量按最大时间窗 30 s × 50 Hz = 1500 点
    hz: 0,
  },
  status:   new Map(),          // motor_status_array：lifecycle / fault / 测量年龄
  events:   [],                 // motor_events，定长上限
  history:  [],                 // 已发命令快照，定长上限
  params:   new Map(),          // device_param_report：rec_* / dev_* / dev_init_state
  traj:     { active: false, id: 0, segmentIndex: 0, segmentCount: 0, completed: 0 },
  imu:      { roll, pitch, yaw, quat, temp, receivedAt, hz },
  policy:   new Map(),          // 页面最后一次下发的 enable/kp/kd（固件不回读）
  selected: null,               // 选中电机 id
};
export function markDirty(...keys) {}   // 脏标记
export function onDirty(fn) {}          // main.js 的 rAF 循环消费
```

脏标记键：`conn` `topology` `motion` `status` `events` `history` `params` `traj` `imu`
`policy` `selected` `alerts`。

### 路由与多窗口

hash 路由 `#/run` `#/teach` `#/device` `#/sim` `#/diag`；空或未知 hash 落 `#/run`。切屏 =
卸载当前屏的全部面板 → 克隆新屏模板进主区 → 挂载其面板。刷新停在当前屏，URL 可直接分享。

多窗口就是开第二个标签页指向不同 hash（`#/run` 盯臂、`#/diag` 盯曲线），各自建立自己的
rosbridge 会话——rosbridge 支持多客户端，两侧互不干扰。

左导航 `1`–`5` 快捷键切屏；`Esc` 关浮层。键盘事件在 `body` 上监听，输入框聚焦时不响应。

### 屏内分层

每屏 = **主区**（常驻）+ **次级抽屉**（默认收起，`<details>`）。抽屉展开后占下半屏，收起时不占
高度。这是「信息分层」的具体落点：常态只看到主区。

抽屉内部：运行屏与示教屏各有多个抽屉面板，用 **tab 切换**（同一时刻只渲染一个）；设备屏只有
事件日志一个，不需要 tab。抽屉的开合状态与当前 tab 都按屏记忆。

挂载点与归属：`wing` 由外壳挂载（不属任何屏），`notes` 由说明浮层挂载。其余 12 个面板都由屏模板
里的 `data-panel` 占位声明，随屏挂载与卸载。

### 响应式

| 宽度 | 行为 |
|---|---|
| ≥1360 | 三区全开 |
| 1080–1360 | 翼默认收起（可手动展开） |
| 820–1080 | 左导航收成 56px 图标导轨；翼默认收起 |
| <820 | 单列：左导航变底部标签栏，翼变顶栏下一行的状态带，连接控件收进「连接」浮层，臂视图 `clamp()` 缩小 |

折叠状态的优先级：**用户显式选择 > 断点默认**；但 `<820` 强制收起（空间确实没有，此时翼只以
状态带形态存在）。

### 断连与错误

断连（rosbridge 未连或超时）时：顶栏状态点变红；停止 rAF 里与数据相关的渲染分支；屏内容置灰并
保留最后一帧数值，标注数据陈旧时间；所有写操作按钮（下发、使能、参数、设零、方向、保存）
禁用，不等超时。这条与 `motor_hardware_interface` 的 fail-fast 语义一致：页面不猜、不降级、
不静默重试。

Agent 与 rosbridge 是两件事，页面上继续分开表达（「启动并连接」只启动所选串口的 Agent，
「断开」只停本页管理的 Agent 子进程，两者都不改变 rosbridge 连接）。

### 说明浮层

现有「说明」抽屉里那 9 条安全注记 + 话题清单 + 页脚，原样搬进一个浮层，由顶栏「说明 ?」打开。
标记仍是面板模板（`notes`），挂载到浮层容器。不进导航、不进路由，`Esc` 或点遮罩关闭。

## 功能对等清单

重构是**纯搬迁**，不新增功能。以下 17 项行为必须在搬迁后逐条存在；实施计划里每项对应一个核对项。

| # | 区域 | 必须保留的行为 |
|---|---|---|
| 1 | 顶栏连接 | rosbridge 地址（localStorage 记忆、刷新自动连/重连）、自动拼 `location.hostname:9090` |
| 2 | 控制板与 Agent | `/dev/serial/by-id/` 优先枚举 + `ttyACM*`/`ttyUSB*` 补缺、扫描、启动并连接、断开、Agent 状态文本（进程状态 ≠ 固件会话已建立） |
| 3 | 状态 chips | 在线 n/N、运行 n、冲突 n、故障 n、运动流 Hz |
| 4 | 关节索引 | 一行一台（ID / 总线 / 型号 / 位置 / 状态），点击选中，与臂上小球双向联动 |
| 5 | 臂视图 | URDF + STL 渲染、实际姿态、幽灵目标、着色 = 生命周期九态、轨迹播放期间不画幽灵、拖拽/缩放/点选、加载失败浮标、`?model=` `?joints=` 两个 URL 参数 |
| 6 | 选中关节读数 | 位置 / 速度 / 力矩 / 电流 / 温度 / 跟随误差 / 命令源 / 故障 / 测量年龄 |
| 7 | 使能与增益 | 逐台 `enable`/`kp`/`kd` 单独下发、策略代数严格递增、本行筛选（默认只显示选中台）、显示全部、受理结果看事件日志的 `CONFIG_RESULT` |
| 8 | 下发 | 命令快照表（增行 / 填入在线电机原位 / 清空）、单发、100 Hz 连发、目标含未初始化或离线台时点名提示、NaN/Inf 整包拒收提示、连发停止后的软停语义不变 |
| 9 | 曲线 | 位置 / 速度 / 力矩 / 电流四通道、多电机叠加、时间窗 5/10/30 s、暂停、清空 |
| 10 | 事件日志 | 列表渲染、导出 JSON、导出 CSV、清空 |
| 11 | 命令历史 | 表格（时间 / seq / 电机 / 状态）、导出 JSON、清空 |
| 12 | 示教 | 点位表（等待 ms、`ID:pos,vel,eff`、空行=等待）、记录当前命令、加空点位、单步、循环、播放/暂停、导入（本页行数组或 `MotorTrajectory` 话题载荷）、导出、单条 ≤256 点、超长切多条**串行**下发（等 `trajectory_state.completed`）、段界重复一行保证位置连续、段尾挑运动最安静的行、完成后末点保持、暂停发送空轨迹取消、**分段进度可见** |
| 13 | 设备参数 | 总览条（已核对 / 未初始化 / 离线计数 + 不一致项明细）、读取（`system_command(REPORT)`）、下发修改（写事务 + SDO 回读核验，约 1 s，收尾自动回一帧报告）、保存 W25、显示全部筛选、来源列、状态列、位置与力矩悬停显示设备端原值（cnt / mA）、行内设零、方向翻转、限位保护开关、限位取自当前位置、7 位有效数字显示、未保存台标黄 |
| 14 | 仿真 | 控制端三选（主机闭环 / 外部控制器 / STM32 HIL）、传输三选（vcan / SocketCAN / SLCAN）、模型枚举、端点设备枚举、物理总线隔离确认（前端与后端双重）、启动 / 停止 / 刷新设备、子进程状态与最近日志 |
| 15 | IMU | Roll/Pitch/Yaw/温度/更新年龄、3D 姿态、人工地平仪、Hz |
| 16 | 折叠记忆 | 屏内抽屉与状态翼的开合状态存 `localStorage`，刷新保持 |
| 17 | 说明 | 9 条安全注记、单位约定、话题清单、页脚（端口与「仅限内网，rosbridge 无鉴权」） |

## 测试

新增 `ros2_ws/src/web_host/test/test_web_assets.py`（pytest，纯标准库解析，零 Node 依赖）。
断言清单：

1. `index.html` 可解析（`html.parser`），全部 `id` 唯一。
2. 扫描 `web/js/**/*.js` 中的 `$('x')` 与 `getElementById('x')` 字面量，每个 `x` 都存在于
   `index.html` 的 id 集合。**这是重构最常犯的错：改名漏改。**
3. 扫描 `web/js/**/*.js`，不得出现 `id="` / `id='`（禁止 JS 生成带 id 的 DOM）。动态行只用 class。
4. `core/router.js` 的 `SCREENS` 常量、`index.html` 的 `data-screen` 模板名集合、左导航项的
   `data-goto` 集合三者相等，且 `DEFAULT_SCREEN` 在集合内。
5. 每个屏模板里出现的 `data-panel="x"` 都有对应的 `<template data-panel-template="x">`。
6. 每个 `<template data-panel-template="x">` 都有 `js/panels/x.js`。
7. 每个 `js/panels/*.js` 至少被一个屏模板引用（白名单：`notes` 只在浮层、`wing` 只在外壳）。
8. 每个 JS 的相对 import 目标文件存在；从 `main.js` 出发的 import 图无环。
9. `index.html` 的每个 `<link href>`、`<script src>`、importmap 映射目标在磁盘上存在。
10. 用 `runpy` 执行 `setup.py` 并 monkeypatch `setuptools.setup` 收集 `data_files`，断言其覆盖的
    文件集合等于磁盘上 `web/` 的文件集合。**防「构建成功但页面 404」**（README 已记录该坑）。

现有 `test/test_agent_manager.py` 保留不动。不引入 JS 测试框架——静态契约能覆盖的正是结构类错误，
行为类错误留给台架回归。

## 迁移

一次性替换，不保留新旧共存（理由见 ADR 0010 Consequences）。按批提交，每批结束时页面可运行：

1. `docs:` ADR 0010 + 本设计（含计划）
2. `feat(web_host):` 外壳 + 路由 + store + `core/` 拆分（五屏可切，主区为空）
3. `test(web_host):` `test_web_assets.py`（对第 2 批的结构断言通过）
4. `feat(web_host):` 运行屏（`jointList` `arm` `readout` `policy` `command` + 三个抽屉面板）
5. `feat(web_host):` 设备屏（`deviceParams`）
6. `feat(web_host):` 示教屏（`teach`）
7. `feat(web_host):` 仿真屏（`sim`）
8. `feat(web_host):` 诊断屏 + 状态翼 + 说明浮层（`curve` `history` `imu` `events` `wing` `notes`）
9. `refactor(web_host):` 删除 `app.js` / `arm3d.js` / `simulation.js` / 旧 `style.css`，
   更新 `README.md`、`docs/adr/README.md` 索引与 `CLAUDE.md` 的 web_host 小节

`arm3d.js` 与 `simulation.js` 不是被删除后重写，而是分别改写为 `panels/arm.js` 与
`panels/sim.js`（three.js 与仿真管理器逻辑照搬，只改挂载方式）。

## 验证标准

1. `colcon build --symlink-install --packages-select web_host` 通过，安装目录
   `install/web_host/share/web_host/web/` 下 `css/`、`js/core/`、`js/panels/` 三层文件齐全
   （不是「构建成功但页面 404」）。
2. `colcon test --packages-select web_host` 通过，`test_web_assets.py` 全绿，
   `test_agent_manager.py` 不回退。
3. 无连接打开页面：不抛异常、顶栏显示未连接、五屏都能切换、写操作按钮全部禁用、rAF 不空转。
4. 台架手动回归（需真机 + micro-ROS Agent，按「功能对等清单」17 项逐条勾）：
   - 连接后关节索引、臂视图、读数、chips 有数据；
   - 使能 → 下发 → 事件日志出现受理结果 → 曲线看到跟随；
   - 参数屏读取 → 改一项限值 → 下发 → 保存 W25，未保存台标黄与来源列正确；
   - 示教：短轨迹（≤256 点）与长轨迹（>256 点切多段串行）都能播完，段进度可见；
   - 仿真屏能起 `simulation.launch.py` 并出日志。
5. 多窗口：两个标签页分别停在 `#/run` 与 `#/diag`，各自数据正常、互不干扰。
6. 窄屏三档断点（1360 / 1080 / 820 附近）目视检查布局不破。
7. 切屏后旧屏面板不再渲染：在 rAF 循环里统计每帧调用 `render` 的面板数，切屏后应只剩新屏的
   数量（可临时打点到 console 核对，不留在最终代码里）。
8. 刷新后停在原屏、抽屉与状态翼的开合状态与刷新前一致。

## 非目标

- 不改 `static_server.py` / `agent_manager.py` / `rosbridge_shim.py` / `simulation_manager.py`
  的接口与行为。
- 不改任何话题名、消息定义、`can_bridge_msgs`、固件或 micro-ROS Agent 契约。
- 不引入任何框架、npm、打包步骤或 JS 测试框架。
- 不做主题切换（浅色/深色）、不做移动端手势优化、不做国际化。
- 不新增功能：不新增话题订阅、不新增写操作、不改变任何安全语义与阈值。
- 不动 `web/vendor/` 与 `roslib.min.js` 的版本。
- 不改写 `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` 等历史文档中
  对 `web/app.js` 的引用（按 `docs/adr/README.md` 的维护规则，历史正文不重写）。
