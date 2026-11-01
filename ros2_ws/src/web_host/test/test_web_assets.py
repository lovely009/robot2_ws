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

# 不属任何屏的面板：notes 由说明浮层挂载
SHELL_ONLY_PANELS = {'notes'}


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


def test_js_does_not_use_unimported_core_constants():
    """面板用了 core 模块的常量却没 import —— 只在运行时炸，而且被 main.js 吞掉。

    实锤：`feedforward.js` 用了 `DIRTY.BENCH` 却从没 import 过 `DIRTY`。mount 照常成功
    （那一行在 render 里，脏帧才走到），而 main.js 的渲染循环把 render 的异常
    `try/catch` 掉只打 console——于是这个面板的脏帧刷新一直是死的，且没人发现
    （2026-10-19 查实并修复）。同一个循环在**挂载**期没有保护时更狠：一个面板抛异常会
    把它之后的占位一起吞掉，表现成"整屏静默缺块"。

    只查全大写常量（`DIRTY` / `TOPICS` / `LIFECYCLE` …）：它们是"用了就一定得 import"
    的裸标识符，误报面最小。查之前先去注释，免得注释里提到名字就误报。
    """
    exported = set()
    for path in sorted((WEB / 'js' / 'core').glob('*.js')):
        exported |= set(re.findall(
            r'export\s+(?:const|let|var|function|class)\s+([A-Za-z_$][\w$]*)',
            path.read_text(encoding='utf-8')))
    constants = {n for n in exported if re.fullmatch(r'[A-Z][A-Z0-9_]*', n)}
    assert constants, 'core 模块里没解析到全大写导出——本测试的前提没了'

    problems = []
    for path in sorted((WEB / 'js').rglob('*.js')):
        if 'vendor' in path.parts:
            continue
        src = path.read_text(encoding='utf-8')
        code = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
        code = re.sub(r'//[^\n]*', '', code)
        imported = set()
        for m in re.finditer(r'import\s*\{([^}]*)\}', src):
            for part in m.group(1).split(','):
                part = part.strip()
                if part:
                    imported.add(part.split(' as ')[-1].strip())
        for name in sorted(constants):
            # 前面不是 `.` 也不是标识符字符 = 当裸标识符在用（排除 obj.NAME）
            if not re.search(rf'(?<![\w.]){name}\b', code):
                continue
            if name in imported:
                continue
            if re.search(rf'\b(?:const|let|var|function|class)\s+{name}\b', code):
                continue
            problems.append(f'{path.relative_to(WEB).as_posix()} 用了 {name} 却没 import')
    assert not problems, '引用了未导入的 core 常量：\n' + '\n'.join(problems)


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


# 7. 每个面板模块都被某屏引用（notes 由外壳挂载，豁免）
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


# 11. setup.py 声明的 web/ 文件必须真的落进 colcon 安装树（防「构建成功但页面 404」）
def test_data_files_cover_web_tree(monkeypatch):
    """setup.py 声明的 web/ 文件都要以**真实文件**出现在 colcon 安装树里。

    比的是源树与 colcon 安装树这**两棵不同的树**：只往 web/ 加了文件、却没重跑
    colcon build 时，安装树里就没有它（symlink-install 下还可能只剩悬空符号链接），
    故用 is_file()（跟随符号链接，悬空即缺失）。这是「构建成功但页面 404」的根因，
    也是本测试唯一要防的失效。安装树不存在 → skip，让未构建的工作区仍能跑本文件。
    只查「源树 ⊆ 安装树」这一个方向；反方向（安装树里的历史残留）跨次构建不稳定，
    不查。
    """
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
    declared = {(PKG / f).resolve() for _dest, files in captured['data_files'] for f in files}
    # 源侧：declared 里落在 web/ 下的文件，取相对 web/ 的路径
    source = sorted(p.relative_to(WEB) for p in declared if WEB in p.parents)

    try:
        from ament_index_python.packages import get_package_share_directory
        install_web = Path(get_package_share_directory('web_host')) / 'web'
    except (ImportError, LookupError):
        pytest.skip('未构建 web_host：先 colcon build --symlink-install --packages-select web_host')

    # is_file() 跟符号链接：源文件不在安装树时链接悬空，算缺失
    missing = sorted(str(rel) for rel in source if not (install_web / rel).is_file())
    assert not missing, (
        f'web/ 下这些文件没有装进 {install_web}（新增文件后必须重跑 '
        f'colcon build --symlink-install --packages-select web_host）：{missing}')


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


# 13. 内置兜底表必须覆盖**默认模型**的全部电机
def test_builtin_joint_map_covers_default_model():
    """内置表只是 `/api/model-joints` 不可达时的兜底，但兜底也必须是对的。

    它曾经是**唯一**来源且只写了第一链 1..9，默认模型又早已是 18 台的
    half_body_robot → 开箱即坏：10-18 关节角恒 0、小球点不中，像"电机没注册"。
    权威来源现在在模型配置（见 test_model_joint_api_is_wired_across_layers），
    这里只保证最后一道兜底不会又写成半个模型。
    """
    store_src = (WEB / 'js' / 'core' / 'store.js').read_text(encoding='utf-8')

    m = re.search(
        r'export const JOINT_OF_MOTOR\s*=\s*JOINT_OVERRIDE\s*\|\|\s*\{(.*?)\n\};',
        store_src, re.S)
    assert m, ('store.js 里找不到 JOINT_OF_MOTOR 的兜底表——'
               '期望 `export const JOINT_OF_MOTOR = JOINT_OVERRIDE || {…};`')
    mapped = {int(k) for k in re.findall(r'(\d+)\s*:', m.group(1))}
    assert mapped, 'JOINT_OF_MOTOR 兜底表解析为空'

    # 默认模型名从同一个文件取，避免两处各写一份而漂移
    mm = re.search(r"const DEFAULT_MODEL = '([^']+)'", store_src)
    assert mm, 'store.js 里找不到默认模型名 DEFAULT_MODEL'
    model_name = mm.group(1)

    driver = (PKG.parent / 'robot_description' / 'models' / model_name /
              'config' / 'motor_driver.yaml')
    if not driver.is_file():
        pytest.skip(f'默认模型 {model_name} 的 motor_driver.yaml 不在源码树里')
    declared = {int(v) for v in
                re.findall(r'^\s*id:\s*(\d+)\s*$',
                           driver.read_text(encoding='utf-8'), re.M)}
    assert declared, f'{driver} 里没解析到任何电机 id'

    missing = sorted(declared - mapped)
    assert not missing, (
        f'默认 JOINT_OF_MOTOR 漏了默认模型 {model_name} 的电机 {missing}：'
        f'它们在臂视图里不可见、不可点选。请在 store.js 的默认表里补齐，'
        f'或先确认"?joints= 覆盖"是否仍是有意设计。')


# 14. 「电机后端」面板的接口控件必须与「仿真」抽屉**同值**
def test_motor_backend_interface_mirrors_the_simulation_drawer():
    """面板里的 CAN 接口 / 设备下拉是同一份值的第二个入口，不是第二份配置。

    这个契约是纯运行时行为，上面 13 条一条都盖不到：删掉任一方向的绑定，构建、
    测试、页面加载全部照常，只在浏览器里表现成"面板里改了不生效"或"面板下拉只剩
    占位项"——正是本文件要防的那类静默失效。所以按源码形状断言（与第 13 条同一
    取舍）：改机制可以，但三个入口的两向绑定与候选同源都得一起搬过去。
    """
    src = (WEB / 'js' / 'panels' / 'sim.js').read_text(encoding='utf-8')
    # (宿主 ui 名, 镜像 ui 名, 候选来源数组)
    pairs = [
        ('transport', 'backendTransport', None),
        ('simInterface', 'backendInterface', 'cans'),
        ('simTty', 'backendTty', 'serials'),
    ]

    # 镜像 → 宿主：三个帧对必须在**注册 change 监听的那个循环**里
    loop = 'for (const [primary, mirror] of ['
    start = src.find(loop)
    assert start != -1, f'sim.js 里找不到镜像绑定循环：{loop!r}'
    block = src[start:start + 900]
    assert 'addEventListener' in block, (
        '镜像绑定循环里没有注册 change 监听——面板改动不会写回宿主')

    missing = []
    for primary, mirror, source in pairs:
        if f'[ui.{primary}, ui.{mirror}]' not in block:
            missing.append(f'镜像绑定循环里缺 [ui.{primary}, ui.{mirror}]')
        # 宿主 → 镜像：syncBackendInterface 里逐个赋值
        if not re.search(rf'ui\.{mirror}\.value\s*=\s*ui\.{primary}\.value', src):
            missing.append(f'ui.{mirror} 没有从 ui.{primary} 取值的赋值')
        # 候选同源：镜像下拉的 option 必须与宿主同一份来源，否则只剩占位项
        if source and not re.search(
                rf'replaceOptions\(\s*ui\.{mirror}\s*,\s*{source}\b', src):
            missing.append(f'ui.{mirror} 的候选没有从 {source} 填充')

    assert not missing, (
        '「电机后端」面板与「仿真」抽屉的同值契约被破坏（面板显示会与实际生效值'
        f'分叉，且页面不报错）：{missing}')


# ---------------------------------------------------------------------------
# 台架服务与 unit 名跨包对账；运动入口固定由 ros2_control 客户端消费。
# ---------------------------------------------------------------------------

SRC = PKG.parent                      # ros2_ws/src
BENCH_NODE = SRC / 'robot_bringup' / 'bench_manager' / 'node.py'
SYSTEMD_UNITS = SRC / 'robot_bringup' / 'bench_manager' / 'systemd_units.py'
AGENT_JS = WEB / 'js' / 'core' / 'agent.js'
FORMAT_JS = WEB / 'js' / 'core' / 'format.js'
FEEDFORWARD_JS = WEB / 'js' / 'panels' / 'feedforward.js'


def _skip_without(path):
    if not path.is_file():
        pytest.skip(f'{path} 不在源码树里（单独部署 web_host？）')


def test_frontend_service_table_matches_bench_manager():
    """前端 SVC 表与 `bench_manager` 注册的 `/bench/*` 服务必须**一一对应**。

    多一条 = 页面上有个调用必然失败；少一条 = 后端的能力页面够不着（本次新增的三个
    重力前馈服务就是后者）。类型名同样对账：roslibjs 按 serviceType 构造请求，错了
    会在服务端反序列化失败，而页面只看到"服务无响应"。
    """
    _skip_without(BENCH_NODE)
    node_src = BENCH_NODE.read_text(encoding='utf-8')
    # 只认 `(类型, '/bench/名字', self._on_…)` 这个形状：`/bench/status` 是话题
    # （`create_publisher`），不长这样。
    registered = {
        (name, f'bench_msgs/srv/{srv}')
        for srv, name in re.findall(
            r"\(\s*([A-Z]\w+),\s*'(/bench/[a-z_]+)',\s*self\._on_", node_src)
    }
    assert registered, 'node.py 的 on_configure 里没解析到任何服务注册'

    declared = {
        (name, srv) for name, srv in re.findall(
            r"\[\s*'(/bench/[a-z_]+)'\s*,\s*'(bench_msgs/srv/\w+)'\s*\]",
            AGENT_JS.read_text(encoding='utf-8'))
    }
    assert declared, 'agent.js 里没解析到 SVC 表'

    assert declared == registered, (
        '前端 SVC 表与 bench_manager 的服务注册不一致：'
        f'前端多出 {sorted(declared - registered)}，'
        f'缺 {sorted(registered - declared)}')


def test_frontend_unit_names_match_bench_manager():
    """四个 unit 名是同一份契约的三处副本（后端常量、前端常量、文档）。"""
    _skip_without(SYSTEMD_UNITS)
    backend = dict(re.findall(
        r'^(UNIT_\w+)\s*=\s*"([^"]+)"', SYSTEMD_UNITS.read_text(encoding='utf-8'),
        re.M))
    assert backend, 'systemd_units.py 里没解析到 UNIT_* 常量'
    frontend = dict(re.findall(
        r'^export const (UNIT_\w+)\s*=\s*\'([^\']+)\'',
        AGENT_JS.read_text(encoding='utf-8'), re.M))
    assert frontend, 'agent.js 里没解析到 UNIT_* 常量'
    assert frontend == backend, (
        f'前端 unit 名与后端不一致：前端 {frontend}，后端 {backend}')


def test_downlink_topics_use_ros2_control_client():
    """运动低频入口固定；设备策略与参数继续走 bridge facade。"""
    js = FORMAT_JS.read_text(encoding='utf-8')
    expected = {
        'setpoint': '/control/setpoint',
        'trajectory': '/control/trajectory',
        'convertedTrajectory': '/control/trajectory',
        'commandStatus': '/control/status',
        'trajectoryState': '/control/trajectory_state',
        'stop': '/control/stop',
        'jointTrajectory': '/arm_controller/joint_trajectory',
        'policy': '/motor/policy_array',
        'sysCmd': '/motor/system_command',
    }
    for key, topic in expected.items():
        m = re.search(rf"^\s*{key}:\s*{{name:\s*'([^']+)'", js, re.M)
        assert m and m.group(1) == topic, f'{key} 未迁移到 {topic}'
    assert not re.search(r"^\s*(?:paramSet|firmwareConfig):", js, re.M)
    assert "stop: {name: '/control/stop', type: 'std_msgs/msg/Empty'}" in js


def test_frontend_has_no_legacy_motion_route_switch():
    for path in js_files():
        text = path.read_text(encoding='utf-8')
        for obsolete in ('setDownlinkRoute', 'getDownlinkRoute', 'command_streamer',
                         '/motor_command/', 'OVERLAY_COMMAND_TOPIC', 'DOWNLINK_ROUTE'):
            assert obsolete not in text, f'{path}: 仍引用 {obsolete}'


def test_gravity_controls_still_use_bench_services():
    src = FEEDFORWARD_JS.read_text(encoding='utf-8')
    assert 'bench.startGravityFeedforward' in src
    assert 'bench.stopGravityFeedforward' in src
    assert 'bench.setGravityFeedforwardGain' in src
    assert 'store.commandStatus' in src
    assert 'gravity_enabled' in src and 'effort_gain' in src and 'data.mode' in src
