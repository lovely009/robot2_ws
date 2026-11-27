"""网页模板、模块引用、安装资产和跨包接口检查。"""
import json
import os
import re
import runpy
from html.parser import HTMLParser
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
WEB = PKG / 'web'
FRONTEND_SRC = PKG / 'frontend' / 'src'
INDEX = WEB / 'panel_templates.html'


class IndexParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = []
        self.panel_templates = []
        self.assets = []
        self._in_importmap = False
        self._importmap = []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if a.get('id'):
            self.ids.append(a['id'])
        if a.get('data-panel-template'):
            self.panel_templates.append(a['data-panel-template'])
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
    assert not INDEX.exists(), f'{INDEX} 应当已退休移除'
    parser = IndexParser()
    parser.feed((PKG / 'frontend/index.html').read_text(encoding='utf-8'))
    for component in (PKG / 'frontend/src').rglob('*.vue'):
        parser.feed(component.read_text(encoding='utf-8'))
    return parser


def js_files():
    files = sorted(FRONTEND_SRC.rglob('*.ts'))
    assert files, f'{FRONTEND_SRC} 下没有任何 .ts'
    return files


def runtime_panels():
    src = (PKG / 'frontend/src/runtime.ts').read_text(encoding='utf-8')
    match = re.search(r'const panelIds: PanelId\[\] = \[([^\]]+)\]', src)
    assert match, 'Vue runtime 缺少面板清单'
    return re.findall(r'"([^"\n]+)"', match.group(1))


def test_all_ids_unique(index):
    dupes = sorted({i for i in index.ids if index.ids.count(i) > 1})
    assert not dupes, f'index.html 里重复的 id：{dupes}'


def test_js_dom_refs_exist(index):
    known = set(index.ids)
    ref = re.compile(r"\$\(\s*'([^']+)'\s*\)|getElementById\(\s*'([^']+)'\s*\)")
    missing = {}
    for f in js_files():
        for m in ref.finditer(f.read_text(encoding='utf-8')):
            name = m.group(1) or m.group(2)
            if name not in known:
                missing.setdefault(f.relative_to(FRONTEND_SRC).as_posix(), set()).add(name)
    assert not missing, f'TS 引用了模板/组件里不存在的 id：{missing}'


def test_js_does_not_use_unimported_core_constants():
    """共享常量需要显式导入。"""
    exported = set()
    for path in sorted((FRONTEND_SRC / 'core').glob('*.ts')):
        exported |= set(re.findall(
            r'export\s+(?:const|let|var|function|class)\s+([A-Za-z_$][\w$]*)',
            path.read_text(encoding='utf-8')))
    constants = {n for n in exported if re.fullmatch(r'[A-Z][A-Z0-9_]*', n)}
    assert constants, 'core 模块里没解析到全大写导出——本测试的前提没了'

    problems = []
    for path in sorted(FRONTEND_SRC.rglob('*.ts')):
        if path.name.endswith('.d.ts'):
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
            # 前面不是 `.` 也不是标识符字符、后面不是 `:`（对象字面量键）= 当裸标识符在用
            if not re.search(rf'(?<![\w.]){name}\b(?!\s*:)', code):
                continue
            if name in imported:
                continue
            if re.search(rf'\b(?:const|let|var|function|class)\s+{name}\b', code):
                continue
            problems.append(f'{path.relative_to(FRONTEND_SRC).as_posix()} 用了 {name} 却没 import')
    assert not problems, '引用了未导入的 core 常量：\n' + '\n'.join(problems)


def test_vue_panels_match_components(index):
    vue_panels = set()
    for component in (PKG / 'frontend/src/components').glob('*Panel.vue'):
        m = re.search(r'data-workbench-panel="([^"]+)"', component.read_text(encoding='utf-8'))
        if m:
            vue_panels.add(m.group(1))
    assert set(runtime_panels()) == vue_panels


def test_panel_modules_exist():
    missing = sorted(p for p in set(runtime_panels())
                     if not (FRONTEND_SRC / 'panels' / f'{p}.ts').is_file())
    assert not missing, f'缺少面板模块 frontend/src/panels/*.ts：{missing}'


def test_panel_modules_are_referenced(index):
    used = set(runtime_panels()) | {'jointRow', 'motion_preview'}
    unused = sorted(f.stem for f in (FRONTEND_SRC / 'panels').glob('*.ts')
                    if f.stem not in used)
    assert not unused, f'面板模块没有被任何屏引用：{unused}'


def test_relative_imports_exist():
    pat = re.compile(r"""(?:^|\n)\s*(?:import|export)[^'"\n]*?from\s*['"]([^'"]+)['"]""")
    missing = []
    for f in js_files():
        for m in pat.finditer(f.read_text(encoding='utf-8')):
            spec = m.group(1)
            if spec.startswith('.'):
                base = (f.parent / spec).resolve()
                if not (base.is_file() or base.with_suffix('.ts').is_file()
                        or base.with_suffix('.d.ts').is_file()
                        or base.with_suffix('.js').is_file()
                        or (base / 'index.ts').is_file()):
                    missing.append(f'{f.relative_to(FRONTEND_SRC)} -> {spec}')
    assert not missing, f'import 目标文件不存在：{missing}'


def test_index_assets_exist(index):
    missing = []
    for ref in index.assets:
        if ref.startswith(('http://', 'https://', '//', 'data:')):
            continue
        target = PKG / 'frontend' / ref.lstrip('/') if ref.startswith('/src/') else WEB / ref.lstrip('/')
        if not target.is_file():
            missing.append(ref)
    assert not missing, f'index.html 引用了不存在的文件：{missing}'


def test_data_files_cover_web_tree(monkeypatch):
    """检查声明的网页资产是否存在于安装目录。"""
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


def test_importmap_targets_exist(index):
    raw = ''.join(index._importmap).strip()
    assert raw, 'index.html 缺少 importmap'
    imports = json.loads(raw).get('imports', {})
    assert imports, 'importmap 里没有 imports'
    for _key, value in imports.items():
        if value.startswith(('http://', 'https://', '//')):
            continue
        assert (WEB / value.lstrip('/')).exists(), f'importmap 目标不存在：{value}'


SRC = PKG.parent                      # ros2_ws/src
BENCH_NODE = SRC / 'robot_bringup' / 'bench_manager' / 'node.py'
SYSTEMD_UNITS = SRC / 'robot_bringup' / 'bench_manager' / 'systemd_units.py'
AGENT_JS = FRONTEND_SRC / 'core' / 'agent.ts'


def _skip_without(path):
    if not path.is_file():
        pytest.skip(f'{path} 不在源码树里（单独部署 web_host？）')


def test_frontend_service_table_matches_bench_manager():
    """核对浏览器与管理器的服务名及消息类型。"""
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
    """前端实际引用的 unit 名必须与后端一致，无需复制未使用的常量。"""
    _skip_without(SYSTEMD_UNITS)
    backend = dict(re.findall(
        r'^(UNIT_\w+)\s*=\s*"([^"]+)"', SYSTEMD_UNITS.read_text(encoding='utf-8'),
        re.M))
    assert backend, 'systemd_units.py 里没解析到 UNIT_* 常量'
    frontend = dict(re.findall(
        r'^export const (UNIT_\w+)\s*=\s*\'([^\']+)\'',
        AGENT_JS.read_text(encoding='utf-8'), re.M))
    assert frontend, 'agent.js 里没解析到 UNIT_* 常量'
    assert frontend.items() <= backend.items(), (
        f'前端 unit 名与后端不一致：前端 {frontend}，后端 {backend}')
