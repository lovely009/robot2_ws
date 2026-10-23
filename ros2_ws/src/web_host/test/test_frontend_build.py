"""正式工作台缺少构建产物时，包安装必须给出明确错误。"""
from pathlib import Path
import runpy

import pytest
import setuptools

SETUP = Path(__file__).resolve().parents[1] / 'setup.py'


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
