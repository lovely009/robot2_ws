# Architecture Source-of-Truth Reconciliation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every current architecture entry point accurately describe the seven-package ROS 2 workspace and establish the accepted `robot_dynamics` dependency boundary.

**Architecture:** Add one workspace ADR as the authoritative decision, then make current-facing summaries point to the same seven-package reality. Preserve historical ADR bodies and implementation plans; only their explicitly current status annotations may change.

**Tech Stack:** Markdown, ROS 2 `package.xml` manifests, POSIX shell/Python 3 validation commands, Git.

**Spec:** `docs/superpowers/specs/2026-09-23-architecture-source-of-truth-design.md`

## Global Constraints

- Change documentation only; do not modify code, `package.xml`, `CMakeLists.txt`, launch, YAML, URDF, or controller files.
- Keep all existing package dependencies and runtime paths unchanged.
- Preserve `motor_drive` as ROS-free and keep `robot_dynamics` outside ros2_control's real-time loop.
- Use ADR 0008 because the architecture index already reserves the unqualified 0007 reference for the firmware ADR.
- Write explanatory prose in Chinese while retaining package names, protocol names, ROS 2 APIs, and Pinocchio in English.
- Do not run `colcon`; validate package count, links, wording, and changed-file scope directly.

## Review Focus

- **Actual package inventory:** exactly seven `ros2_ws/src/*/package.xml` manifests must exist and all seven names must appear in the current workspace summaries; Task 2 Step 5 checks this.
- **Stale current-state wording:** no current entry point or ADR applicability annotation may still describe `dev/robot_ws` as six packages; Task 3 Step 4 checks this.
- **Historical integrity:** ADR 0001/0005 decision bodies and the old implementation plan must remain unchanged; only named applicability/status lines may change; Task 3 Step 5 checks the scoped diff.
- **ADR identity and reachability:** workspace ADR 0008 and firmware ADR 0007 must both have distinct, valid links from the index; Task 1 Step 5 checks both targets.
- **Documentation-only scope:** no source, manifest, build, launch, configuration, model, or generated file may enter the implementation commits; Task 3 Step 6 enforces an allowlist.

---

### Task 1: Establish the authoritative workspace ADR

**Files:**
- Create: `docs/adr/0008-robot-dynamics-dependency-boundary.md`
- Modify: `docs/adr/README.md:5-13`

**Interfaces:**
- Consumes: the approved boundary in `docs/superpowers/specs/2026-09-23-architecture-source-of-truth-design.md` and actual dependencies from `ros2_ws/src/*/package.xml`.
- Produces: an accepted ADR named `0008-robot-dynamics-dependency-boundary.md` and a discoverable index row used by all later documentation.

- [ ] **Step 1: Capture the failing ADR reachability check**

Run:

```bash
test -f docs/adr/0008-robot-dynamics-dependency-boundary.md
```

Expected: exit status 1 because ADR 0008 does not exist yet.

- [ ] **Step 2: Create ADR 0008 with the accepted decision**

Create `docs/adr/0008-robot-dynamics-dependency-boundary.md` with this complete content:

````markdown
# robot_dynamics 独立依赖边界

**Status**: accepted

## Context

`robot_dynamics` 是 `dev/robot_ws` 七包工作区中的独立运动学/动力学计算包。当前版本只验证 ROS 2 Humble 提供的 Pinocchio 可被 ament_cmake 查找、链接和测试；现有真机控制、独立电机仿真、Gazebo 模型与 Web 上位机均不需要通过该包运行。

若把 Pinocchio 直接加入 `motor_drive`、`motor_hardware_interface` 或 `robot_arm_gazebo`，会把通用计算依赖带入底层驱动、实时控制适配或模型资产包，模糊现有职责边界。

## Decision

- `robot_dynamics` 作为通用机器人运动学/动力学计算入口，当前只直接依赖 Pinocchio。
- `robot_dynamics` 不依赖 `motor_drive`、`motor_hardware_interface`、`motor_sim` 或 Gazebo，也不进入 ros2_control 实时循环。
- 当前不提供 ROS 节点、topic、service、参数、launch 或 URDF 加载逻辑。
- FK、IK、RNEA、重力补偿、轨迹前馈和 URDF 模型加载仅在出现具体需求时增加。
- 后续若控制或仿真包需要消费 `robot_dynamics`，或者依赖方向需要改变，必须另立 ADR。

当前工作区包级边界为：

```text
can_bridge_msgs                         固件与 ROS 2 消息契约
motor_hardware_interface -> motor_drive 真机 ros2_control 适配 -> 无 ROS 底层驱动
motor_sim                               与机器人无关的独立电机仿真
robot_arm_gazebo                        机器人模型资产与 Gazebo 集成
robot_dynamics -> Pinocchio             独立运动学/动力学计算边界
web_host -> can_bridge_msgs             Web 上位机消息
web_host -> robot_arm_gazebo            运行期模型资源
```

## Considered Options

- **直接把 Pinocchio 加入现有控制或模型包**：初始文件更少，但会扩大实时控制或模型资产包的依赖与职责，否决。
- **只保留设计稿，不建立 ADR**：代码可以工作，但当前架构索引无法给出稳定边界，否决。
- **新增独立包（已选）**：多一个最小 ament 包，但依赖和后续 API 有清晰归属。

## Consequences

- `motor_drive` 继续保持 ROS 无关，现有真机、仿真和 Gazebo 运行路径不变。
- Pinocchio 缺失时只影响 `robot_dynamics` 的配置与构建，不提供静默降级或源码下载。
- 后续动力学 API 可以独立测试；跨包集成必须显式记录新的依赖决策。
````

- [ ] **Step 3: Update the ADR index date, current package count, and ADR 0008 row**

In `docs/adr/README.md`:

1. Change the table heading date from `2026-09-11` to `2026-09-23`.
2. Change both current-state references from `六包` to `七包` in rows 0001 and 0005.
3. Keep the firmware ADR 0007 row unchanged.
4. Append this row immediately after firmware ADR 0007:

```markdown
| [0008 robot_dynamics 独立依赖边界](0008-robot-dynamics-dependency-boundary.md) | accepted | **当前有效**：Pinocchio 只由独立计算包承载，不进入现有控制、仿真或 Gazebo 运行路径。 |
```

- [ ] **Step 4: Check ADR formatting and links**

Run:

```bash
git diff --check -- docs/adr/README.md docs/adr/0008-robot-dynamics-dependency-boundary.md
python3 - <<'PY'
from pathlib import Path
index = Path('docs/adr/README.md').read_text()
assert '(../../micro_ros_firmware/docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md)' in index
assert '(0008-robot-dynamics-dependency-boundary.md)' in index
assert Path('micro_ros_firmware/docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md').is_file()
assert Path('docs/adr/0008-robot-dynamics-dependency-boundary.md').is_file()
print('ADR links: PASS')
PY
```

Expected: `git diff --check` exits 0 and Python prints `ADR links: PASS`.

- [ ] **Step 5: Verify package claims against manifests**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
from xml.etree import ElementTree as ET
expected = {
    'can_bridge_msgs', 'motor_drive', 'motor_hardware_interface',
    'motor_sim', 'robot_arm_gazebo', 'robot_dynamics', 'web_host',
}
actual = {
    ET.parse(path).getroot().findtext('name')
    for path in Path('ros2_ws/src').glob('*/package.xml')
}
assert actual == expected, (actual, expected)
print('seven-package inventory: PASS')
PY
```

Expected: `seven-package inventory: PASS`.

- [ ] **Step 6: Commit the ADR and index**

```bash
git add docs/adr/README.md docs/adr/0008-robot-dynamics-dependency-boundary.md
git commit -m "docs: record robot dynamics architecture boundary"
```

### Task 2: Synchronize current architecture entry points

**Files:**
- Modify: `README.md:3-16`
- Modify: `AGENTS.md:6-19`
- Modify: `CLAUDE.md:5-24,109-123`
- Modify: `CONTEXT.md:5-12`

**Interfaces:**
- Consumes: the seven-package inventory and accepted boundary established by Task 1.
- Produces: consistent current-facing repository guidance for humans and agents.

- [ ] **Step 1: Confirm the current summaries are stale**

Run:

```bash
grep -nE '六个 ament 包|六包工作区|仿真 6 包' README.md AGENTS.md CLAUDE.md
```

Expected: matches in all three files, proving the current summaries still describe six packages.

- [ ] **Step 2: Update the root README package inventory**

In `README.md`:

1. Change `六个 ament 包` to `七个 ament 包` on the introductory line.
2. Insert this table row between `robot_arm_gazebo` and `web_host`:

```markdown
| `robot_dynamics` | 独立 Pinocchio 运动学/动力学计算边界（当前仅验证依赖） |
```

3. Change `真机 + 仿真 6 包` to `真机 + 仿真 + 动力学 7 包` in the `dev/robot_ws` branch description.

- [ ] **Step 3: Update agent-facing repository guidance**

In `AGENTS.md`:

1. Replace the `ros2_ws/` region description with:

```markdown
| `ros2_ws/` | ROS 2 Humble 七包工作区：消息、真机、仿真、动力学与 Web 上位机 | 本文件 + `CONTEXT.md` + 各包 `README.md` |
```

2. Replace lines 14-19 with:

```markdown
真机 `motor_drive`（无 ROS 底层驱动，双传输 `socketcan`/`serial_slcan`，协议 `vaxor`/`cerb`）
+ `motor_hardware_interface`（ros2_control 适配 + 真机入口）+ 仿真 `motor_sim` +
`robot_arm_gazebo`；`robot_dynamics` 是独立 Pinocchio 运动学/动力学计算边界；
`can_bridge_msgs` 提供固件/ROS 2 共用消息，`web_host` 提供 rosbridge Web 上位机。
**电机拓扑唯一来源是 YAML**：数量/类型/ID/关节名/总线/传输由
`motor_hardware_interface/config/motor_driver.yaml` 决定，增删改不修改 C++、launch
或 controller 配置。
```

- [ ] **Step 4: Update Claude guidance and the domain glossary**

In `CLAUDE.md`:

1. Change `六个 ament 包` to `七个 ament 包`.
2. Add this bullet between `robot_arm_gazebo` and `web_host`:

```markdown
- `robot_dynamics` —— 独立 Pinocchio 运动学/动力学计算边界（当前仅验证依赖）
```

3. Change the branch description to `真机 + 仿真 + 动力学 7 包`.
4. Immediately after the existing four-package architecture diagram's closing fence, add:

```markdown

独立计算边界：`robot_dynamics -> Pinocchio`。它不依赖电机驱动、ros2_control 适配、
`motor_sim` 或 Gazebo，也不进入现有实时控制路径；跨包集成须另立 ADR。
```

In `CONTEXT.md`, append this row to the layering table after the simulation row:

```markdown
| 运动学/动力学层 | `robot_dynamics`：独立 Pinocchio 计算边界，不属于电机驱动实时链路 |
```

- [ ] **Step 5: Verify every current summary contains the exact seven-package inventory**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
packages = (
    'can_bridge_msgs', 'motor_drive', 'motor_hardware_interface', 'motor_sim',
    'robot_arm_gazebo', 'robot_dynamics', 'web_host',
)
for name in ('README.md', 'CLAUDE.md'):
    text = Path(name).read_text()
    missing = [package for package in packages if f'`{package}`' not in text]
    assert not missing, (name, missing)
for name in ('README.md', 'AGENTS.md', 'CLAUDE.md'):
    text = Path(name).read_text()
    assert '七包' in text or '七个 ament 包' in text, name
assert '不属于电机驱动实时链路' in Path('CONTEXT.md').read_text()
print('current architecture summaries: PASS')
PY
git diff --check -- README.md AGENTS.md CLAUDE.md CONTEXT.md
```

Expected: Python prints `current architecture summaries: PASS`; `git diff --check` exits 0.

- [ ] **Step 6: Commit the synchronized entry points**

```bash
git add README.md AGENTS.md CLAUDE.md CONTEXT.md
git commit -m "docs: synchronize seven-package architecture"
```

### Task 3: Correct current annotations without rewriting history

**Files:**
- Modify: `docs/adr/0001-motor-canfd-layering-and-config-single-source.md:3`
- Modify: `docs/adr/0005-dev-real-branch-and-motor-drive-rename.md:3`
- Modify: `docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md:4`

**Interfaces:**
- Consumes: ADR 0008 and the synchronized current summaries from Tasks 1-2.
- Produces: accurate present-day annotations while preserving historical decisions and plans.

- [ ] **Step 1: Record the narrow pre-change anchors**

Run:

```bash
grep -n '当前 `dev/robot_ws`.*六包' \
  docs/adr/0001-motor-canfd-layering-and-config-single-source.md \
  docs/adr/0005-dev-real-branch-and-motor-drive-rename.md
grep -n -- '- 状态：待用户审阅' \
  docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md
```

Expected: one current-applicability match in each ADR and one status match in the implemented Pinocchio design.

- [ ] **Step 2: Correct only the two ADR applicability annotations**

Make these exact replacements:

```text
docs/adr/0001... line 3:
当前 `dev/robot_ws` 是六包工作区。
→
当前 `dev/robot_ws` 是七包工作区。

docs/adr/0005... line 3:
当前 `dev/robot_ws` 已重新包含六包，
→
当前 `dev/robot_ws` 已演进为七包工作区，
```

Do not change either ADR's title, `Status`, context, decision, options, or consequences.

- [ ] **Step 3: Mark the completed Pinocchio design as implemented**

In `docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md`, replace only:

```markdown
- 状态：待用户审阅
```

with:

```markdown
- 状态：已实施
```

Do not update the historical implementation plan under `docs/superpowers/plans/2026-09-22-pinocchio-dependency.md`.

- [ ] **Step 4: Verify no current entry point still claims a six-package workspace**

Run:

```bash
if grep -nE '六个 ament 包|六包工作区|重新包含六包|仿真 6 包' \
  README.md AGENTS.md CLAUDE.md CONTEXT.md docs/adr/*.md; then
  echo 'stale current package count found' >&2
  exit 1
fi
```

Expected: no output and exit status 0. Historical decision-body phrases such as `四包` and `两包` remain valid and are intentionally not searched.

- [ ] **Step 5: Verify historical edits are limited to annotations**

Run:

```bash
git diff --unified=0 -- \
  docs/adr/0001-motor-canfd-layering-and-config-single-source.md \
  docs/adr/0005-dev-real-branch-and-motor-drive-rename.md \
  docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md
```

Expected: exactly three one-line replacements: two six-to-seven applicability corrections and one status change. No historical decision-body hunk appears.

- [ ] **Step 6: Enforce the documentation-only changed-file allowlist**

Run:

```bash
python3 - <<'PY'
import subprocess
allowed = {
    'README.md', 'AGENTS.md', 'CLAUDE.md', 'CONTEXT.md',
    'docs/adr/README.md',
    'docs/adr/0001-motor-canfd-layering-and-config-single-source.md',
    'docs/adr/0005-dev-real-branch-and-motor-drive-rename.md',
    'docs/adr/0008-robot-dynamics-dependency-boundary.md',
    'docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md',
    'docs/superpowers/specs/2026-09-23-architecture-source-of-truth-design.md',
    'docs/superpowers/plans/2026-09-23-architecture-source-of-truth.md',
}
base = 'fe65593'
changed = set(subprocess.check_output(
    ['git', 'diff', '--name-only', f'{base}..HEAD'], text=True
).splitlines())
changed.update(subprocess.check_output(
    ['git', 'diff', '--name-only'], text=True
).splitlines())
changed.update(subprocess.check_output(
    ['git', 'ls-files', '--others', '--exclude-standard'], text=True
).splitlines())
unexpected = changed - allowed
assert not unexpected, sorted(unexpected)
print('documentation-only scope: PASS')
PY
git diff --check
```

Expected: `documentation-only scope: PASS`; `git diff --check` exits 0.

- [ ] **Step 7: Commit the annotation corrections**

```bash
git add \
  docs/adr/0001-motor-canfd-layering-and-config-single-source.md \
  docs/adr/0005-dev-real-branch-and-motor-drive-rename.md \
  docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md \
  docs/superpowers/plans/2026-09-23-architecture-source-of-truth.md
git commit -m "docs: reconcile architecture status annotations"
```

- [ ] **Step 8: Run the final repository-state verification**

Run:

```bash
git status --short
git log --oneline -4
```

Expected: clean status. The latest three implementation commits are:

```text
docs: reconcile architecture status annotations
docs: synchronize seven-package architecture
docs: record robot dynamics architecture boundary
```

The preceding commit is the approved design:

```text
docs: design architecture source reconciliation
```
