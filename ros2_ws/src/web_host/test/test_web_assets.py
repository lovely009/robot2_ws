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

    install_web = PKG.parents[1] / 'install' / 'web_host' / 'share' / 'web_host' / 'web'
    if not install_web.is_dir():
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

    m = re.search(r'if \(!q\) \{\s*return \{(.*?)\};', store_src, re.S)
    assert m, 'store.js 里找不到 JOINT_OF_MOTOR 的默认表'
    mapped = {int(k) for k in re.findall(r'(\d+)\s*:', m.group(1))}
    assert mapped, 'JOINT_OF_MOTOR 默认表解析为空'

    # 默认模型名从同一个文件取，避免两处各写一份而漂移
    mm = re.search(r"URLQ\.get\('model'\)\s*\|\|\s*'([^']+)'", store_src)
    assert mm, 'store.js 里找不到默认模型名'
    model_name = mm.group(1)

    driver = (PKG.parent / 'robot_sim' / 'models' / model_name /
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
