"""动作库文件层（web_host/actions.py）单测：枚举/摘要/读取/导入与安全校验。

纯文件逻辑，用 tmp_path 构造 web_root 与兄弟目录 action_library/，不碰 ROS。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from web_host.actions import (
    action_summary,
    actions_directory,
    list_actions,
    read_action,
    save_action,
)


def _payload(ids: Any = (1, 2), steps: int = 3) -> dict:
    points = []
    for k in range(steps):
        for motor_id in ids:
            points.append({
                'time_from_start_ms': k * 100,
                'motor_id': motor_id,
                'position': 0.1 * k,
                'velocity': 0.0,
                'torque': 0.0,
            })
    return {'target_ids': list(ids), 'points': points}


@pytest.fixture
def web_root(tmp_path: Path) -> Path:
    web = tmp_path / 'web'
    web.mkdir()
    return web


def test_summary_reports_ids_and_duration() -> None:
    summary = action_summary(_payload(ids=(1, 2), steps=3))
    assert summary['target_ids'] == [1, 2]
    assert summary['target_count'] == 2
    assert summary['point_count'] == 6
    assert summary['duration_ms'] == 200


@pytest.mark.parametrize('bad', [
    [],                                     # 不是对象
    {'points': []},                         # 缺 target_ids
    {'target_ids': []},                     # 空集合
    {'target_ids': [1], 'points': []},      # 无路点
    {'target_ids': [1], 'points': [{'motor_id': 1}]},   # 缺时间字段
])
def test_summary_rejects_non_trajectory(bad: Any) -> None:
    with pytest.raises(ValueError):
        action_summary(bad)


def test_list_and_read_roundtrip(web_root: Path) -> None:
    directory = actions_directory(web_root)
    directory.mkdir()
    (directory / 'demo.json').write_text(json.dumps(_payload()), encoding='utf-8')
    (directory / 'broken.json').write_text('{not json', encoding='utf-8')

    by_name = {entry['name']: entry for entry in list_actions(web_root)}
    assert by_name['demo.json']['point_count'] == 6
    assert by_name['demo.json']['target_ids'] == [1, 2]
    assert 'error' in by_name['broken.json']    # 坏文件保留条目、不炸整个列表

    assert json.loads(read_action(web_root, 'demo.json'))['target_ids'] == [1, 2]
    with pytest.raises(FileNotFoundError):
        read_action(web_root, 'missing.json')


def test_list_returns_empty_without_directory(web_root: Path) -> None:
    assert list_actions(web_root) == []


def test_save_requires_overwrite_flag(web_root: Path) -> None:
    text = json.dumps(_payload())
    entry = save_action(web_root, 'new.json', text)
    assert entry['name'] == 'new.json'
    with pytest.raises(FileExistsError):
        save_action(web_root, 'new.json', text)
    entry = save_action(web_root, 'new.json', text, overwrite=True)
    assert entry['point_count'] == 6
    assert json.loads(read_action(web_root, 'new.json'))['target_ids'] == [1, 2]


@pytest.mark.parametrize('name', [
    '', '..json', '../evil.json', 'a/b.json', 'noext', '.hidden.json',
    'a b.json', 'x' * 80 + '.json', '中文.json',
])
def test_names_are_rejected(web_root: Path, name: str) -> None:
    with pytest.raises(ValueError):
        save_action(web_root, name, json.dumps(_payload()))
    with pytest.raises(ValueError):
        read_action(web_root, name)


def test_symlinked_action_is_readable(tmp_path: Path) -> None:
    # symlink-install 的内置动作指向源码；读取须跟随链接，不能将解析后的目标误判为越界。
    web = tmp_path / 'web'
    web.mkdir()
    source = tmp_path / 'source-library'
    source.mkdir()
    (source / 'demo.json').write_text(json.dumps(_payload()), encoding='utf-8')
    directory = actions_directory(web)
    directory.mkdir()
    (directory / 'demo.json').symlink_to(source / 'demo.json')

    assert json.loads(read_action(web, 'demo.json'))['target_ids'] == [1, 2]
    assert list_actions(web)[0]['point_count'] == 6


def test_save_rejects_bad_payload(web_root: Path) -> None:
    with pytest.raises(ValueError):
        save_action(web_root, 'bad.json', '{"hello": 1}')
    with pytest.raises(ValueError):
        save_action(web_root, 'bad.json', '{not json')
    with pytest.raises(ValueError):
        save_action(web_root, 'bad.json', '   ')


@pytest.mark.parametrize('field,value', [
    ('target_ids', [None]), ('target_ids', [True]), ('target_ids', [1.5]),
    ('time_from_start_ms', None), ('time_from_start_ms', float('inf')),
    ('time_from_start_ms', -1), ('time_from_start_ms', True),
])
def test_invalid_numbers_reject_import_and_do_not_hide_library(web_root, field, value):
    payload = _payload()
    if field == 'target_ids':
        payload[field] = value
    else:
        payload['points'][0][field] = value
    text = json.dumps(payload)
    with pytest.raises(ValueError):
        save_action(web_root, 'bad.json', text)
    save_action(web_root, 'good.json', json.dumps(_payload()))
    (actions_directory(web_root) / 'bad.json').write_text(text)
    entries = {entry['name']: entry for entry in list_actions(web_root)}
    assert 'error' in entries['bad.json']
    assert entries['good.json']['target_ids'] == [1, 2]


def test_failed_overwrite_preserves_symlink_and_original(web_root, tmp_path, monkeypatch):
    import os
    original = json.dumps(_payload())
    source = tmp_path / 'source.json'
    source.write_text(original)
    directory = actions_directory(web_root)
    directory.mkdir()
    link = directory / 'demo.json'
    link.symlink_to(source)
    def disk_full(*_):
        raise OSError(28, 'No space left on device')
    monkeypatch.setattr(os, 'fsync', disk_full)
    with pytest.raises(OSError):
        save_action(web_root, 'demo.json', json.dumps(_payload(ids=(3,))), overwrite=True)
    assert link.is_symlink()
    assert source.read_text() == original
    assert sorted(directory.iterdir()) == [link]
    assert sorted(tmp_path.glob('*.json')) == [source]


def test_overwrite_updates_symlink_target_not_link(web_root, tmp_path):
    source = tmp_path / 'source.json'
    source.write_text(json.dumps(_payload()))
    directory = actions_directory(web_root)
    directory.mkdir()
    link = directory / 'demo.json'
    link.symlink_to(source)
    save_action(web_root, 'demo.json', json.dumps(_payload(ids=(3,))), overwrite=True)
    assert link.is_symlink()
    assert json.loads(source.read_text())['target_ids'] == [3]


def test_concurrent_new_imports_allow_only_one_complete_file(web_root, monkeypatch):
    import os
    import threading
    from concurrent.futures import ThreadPoolExecutor
    directory = actions_directory(web_root)
    directory.mkdir()
    barrier = threading.Barrier(2)
    exists, link = Path.exists, os.link
    def racing_exists(path):
        answer = exists(path)
        if path == directory / 'race.json':
            barrier.wait(timeout=5)
        return answer
    def racing_link(source, target):
        barrier.wait(timeout=5)
        return link(source, target)
    monkeypatch.setattr(Path, 'exists', racing_exists)
    monkeypatch.setattr(os, 'link', racing_link)
    def save(motor):
        try:
            save_action(web_root, 'race.json', json.dumps(_payload(ids=(motor,))))
            return motor
        except FileExistsError:
            return None
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(save, (1, 2)))
    winners = [motor for motor in results if motor is not None]
    assert len(winners) == 1
    assert json.loads(read_action(web_root, 'race.json'))['target_ids'] == winners
    assert list(directory.iterdir()) == [directory / 'race.json']


@pytest.mark.parametrize('field,value', [
    ('target_ids', [65536]), ('time_from_start_ms', 4294967296),
])
def test_action_numbers_fit_wire_fields(web_root, field, value):
    payload = _payload()
    if field == 'target_ids':
        payload[field] = value
    else:
        payload['points'][0][field] = value
    with pytest.raises(ValueError):
        save_action(web_root, 'overflow.json', json.dumps(payload))


def test_wire_integer_upper_bounds_remain_valid():
    payload = {'target_ids': [65535], 'points': [{'time_from_start_ms': 4294967295}]}
    assert action_summary(payload)['target_ids'] == [65535]


def test_atomic_save_keeps_existing_and_new_file_permissions(web_root, tmp_path):
    import stat
    directory = actions_directory(web_root)
    directory.mkdir()
    reference = directory / 'reference.json'
    reference.write_text(json.dumps(_payload()))
    source = tmp_path / 'source.json'
    source.write_text(json.dumps(_payload()))
    source.chmod(0o640)
    link = directory / 'existing.json'
    link.symlink_to(source)
    save_action(web_root, 'existing.json', json.dumps(_payload(ids=(3,))), overwrite=True)
    save_action(web_root, 'new.json', json.dumps(_payload()))
    assert stat.S_IMODE(source.stat().st_mode) == 0o640
    assert stat.S_IMODE((directory / 'new.json').stat().st_mode) == stat.S_IMODE(reference.stat().st_mode)
