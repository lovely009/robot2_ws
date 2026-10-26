"""正式工作台缺少构建产物时，包安装必须给出明确错误。"""
from pathlib import Path
import runpy

import pytest
import setuptools

SETUP = Path(__file__).resolve().parents[1] / 'setup.py'


def test_frontend_has_only_one_application_entry() -> None:
    package = SETUP.parent
    entries = sorted(path.relative_to(package) for path in package.glob('*/src/main.ts'))
    assert entries == [Path('frontend/src/main.ts')], (
        f'仅保留正式 Vue 工作台入口，不再保留独立旧原型：{entries}')


def test_install_rejects_missing_workbench_build(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    (tmp_path / 'frontend').mkdir()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: None)
    with pytest.raises(RuntimeError, match='npm.*build'):
        runpy.run_path(str(SETUP), run_name='__main__')


def test_install_includes_workbench_assets(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    (tmp_path / 'frontend').mkdir()
    destination = tmp_path / 'web/workbench/assets'
    destination.mkdir(parents=True)
    (destination.parent / 'index.html').write_text('<html></html>')
    (destination / 'workbench.js').write_text('export {}')
    captured = {}
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: captured.update(kwargs))
    runpy.run_path(str(SETUP), run_name='__main__')
    declared = {file for _, files in captured['data_files'] for file in files}
    assert 'web/workbench/index.html' in declared
    assert 'web/workbench/assets/workbench.js' in declared


def test_installed_web_contains_all_runtime_modules() -> None:
    from ament_index_python.packages import get_package_share_directory

    source = SETUP.parent / 'web/js'
    installed = Path(get_package_share_directory('web_host')) / 'web/js'
    missing = sorted(str(path.relative_to(source)) for path in source.rglob('*.js')
                     if not (installed / path.relative_to(source)).is_file())
    assert not missing, (
        f'运行时模块未安装：{missing}；新增静态文件后必须重新构建 web_host，'
        '源码浏览器测试通过不能替代安装目录验证')
