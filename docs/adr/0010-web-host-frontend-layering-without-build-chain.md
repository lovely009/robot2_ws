# web_host 前端的无构建链分层

**Status**: accepted

## Context

`web_host/web/` 是纯静态资产：`index.html` 363 行、`app.js` 2613 行、`style.css` 450 行、
`arm3d.js` 207 行、`simulation.js` 335 行、`vendor/`（three 0.160.1 + urdf-loader 0.13.1）与
`roslib.min.js`。由 `web_host/static_server.py` 用标准库 `ThreadingHTTPServer` 托管，
`setup.py` 的 `_web_data_files()` 以 `os.walk('web', followlinks=True)` 逐目录列 `data_files`；
浏览器侧用原生 import map 解析 three 与 urdf-loader，没有 npm/打包步骤。

实际使用中暴露的问题是**信息组织**而非功能缺失：单页把 15 块面板平铺成一条长页加 8 个
`<details>` 抽屉，顺序按「页面有地方放」而非「干活时手边要有」；常用操作（下发、使能、示教、
参数）埋在抽屉里；没有导航；密度均匀，常态信息与深度信息视觉等价；窄屏只能顺序堆叠。日常
使用集中在四件事：真机台架调试（核对 → 使能 → 下发 → 看反馈）、示教录制与回放、仿真验证、
参数整定与故障处理。

约束：「无 Node 构建依赖」已写进 `web_host/README.md` 的构建说明与 `CLAUDE.md` 的构建小节，
页面由 colcon 直接安装。引入构建链会同时改动 `setup.py`、构建文档与部署方式。

## Decision

1. **范围**：只重构 `web_host` 的前端静态资产。不改 `static_server` / `agent_manager` /
   `rosbridge_shim` / `simulation_manager` 的接口，不改任何话题、消息定义或固件契约，不新增功能。
2. **保持无构建链**：原生 ES module + 浏览器 import map；不引框架，不加 npm/Vite。`vendor/`
   仍只承载 three 与 urdf-loader。
3. **三层结构**：外壳（顶栏 / 左导航 / 状态翼）、屏（路由单位）、面板（可挂载组件）。屏只声明
   「挂哪些面板」，面板只负责渲染与交互。二者都不持有跨屏数据。
4. **状态集中**：所有运行时数据进 `core/store.js`（拓扑、`motion_states` 环形缓冲、事件、
   命令历史、参数、告警）。订阅在入口全局建立，与屏是否可见无关；面板卸载只停止渲染，不丢
   数据。曲线缓冲与事件列表因此跨屏存活。
5. **DOM 单一来源**：`index.html` 持有全部标记——外壳、每块面板一个 `<template>`、每个屏一个
   `<template>`。面板模板定义一次，被多屏引用时按屏克隆。**JS 不得生成带 `id` 的元素**，动态
   生成的行只用 class。这条是第 8 条可测性的前提。
6. **路由**：hash 路由 `#/run` `#/teach` `#/device` `#/sim` `#/diag`。屏不是模块：屏 id 由
   `core/router.js` 的 `SCREENS` 常量、`index.html` 的 `<template data-screen="...">` 名与左
   导航项的 `data-goto` 三处共同表达，由测试校验一致。多窗口并行 = 开两个标签页指向不同 hash，
   各自建立 rosbridge 会话。
7. **渲染**：单一 rAF 循环 + 脏标记；未挂载的面板不参与渲染。现有代码的常驻定时渲染全部撤掉。
8. **测试**：前端不引 JS 测试框架。可测性由 Python 侧静态契约测试承担
   （`test/test_web_assets.py`）：DOM id 引用完整性、屏与面板模板和路由表一致、import 图存在
   且无环、静态资产被 `data_files` 覆盖。这四条恰好覆盖重构最常犯的错——改名漏改、新增文件
   忘记重跑 colcon。

## Considered Options

- **只整理现有单页，不换范式**：改动最小，但「信息分层」在不换骨架的前提下无法落地——折叠区
  与常驻区在同一密度下视觉等价，最后只能靠加颜色区分，与现有「数字用等宽、状态用色点、面板不
  写句子」的原则冲突。
- **vendor 轻量框架（Preact + htm）**：组件化写法更干净，但路由与 store 仍要自写，vendor 版本
  要自己维护，且与「vendor 只放 three 与 urdf-loader」的现状不一致；收益不抵偏离。
- **引入 Vite + React**：开发体验最好，但要改 `setup.py`、构建文档与部署方式，且与已写入
  README 的「无 Node 依赖」约定正面冲突。
- **面板工作台范式**（用户自由拖摆面板、布局编码进 URL）：最贴合多窗口并行盯盘，但每个视图都要
  用户自己搭一次，且拖拽网格布局 + 面板生命周期是几种方案里最重的自写代码。

## Consequences

正面：

- 屏与面板的边界把文件规模拉回可单次读完的量级（2613 行单文件是当前主要维护成本）。
- 数据与渲染分离后，切屏不再丢缓冲，不可见屏也不再空转渲染。
- DOM 单一来源使静态契约测试成立，无需引入 Node。
- 新增面板或屏的流程固定：写面板模板 → 写面板模块 → 在屏模板里加一行引用。

负面与代价：

- **一次性替换**：重构期间 `web/` 页面不能用于台架，直到五屏全部搬迁完成。保留新旧共存的两份
  前端被认为比一次性替换更容易出错，故不采用。
- `index.html` 从 363 行涨到约 700 行（含全部模板）。以「单一 DOM 来源」换取可测性，是本决策
  的明确取舍。
- 面板跨屏复用要求所有共享数据都在 store；任何面板私自缓存都会在多屏挂载时出现不一致。
- 静态契约测试只能校验结构与引用完整性，不能替代台架上的行为回归。

## References

- 设计稿随 2026-10-14 文档收敛删除；层边界以本 ADR 为准。
