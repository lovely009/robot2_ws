"""动作库：`action_library/*.json` 的枚举、读取与保存（动作屏播放的数据源）。

文件是 MotorTrajectory 话题载荷（`{target_ids, points:[{time_from_start_ms,
motor_id, position, velocity, torque|effort}]}`）。本模块只管文件层；播放入口
位于前端 `js/panels/actionLibrary.js`，通过受监督的主机控制路径执行。

读写都落在 share 安装树的 `action_library/`：内置文件在 `--symlink-install`
下是指回源码的符号链接（覆盖即写回源码），导入的新文件写在安装树里。
"""

import json
import os
import re
import stat
import uuid
from pathlib import Path

# 文件名白名单：仅允许单层、非隐藏的 JSON 文件名。
_NAME_RE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.\-]{0,63}\.json$")
_MAX_TEXT_BYTES = 8 * 1024 * 1024


def actions_directory(web_root: str) -> Path:
    """动作库目录 = web_root 的兄弟目录 `action_library/`。"""
    return Path(web_root).parent / "action_library"


def bvh_directory(web_root: str) -> Path:
    """工作区安装优先读取源码 BVH；独立部署仍使用安装动作库。"""
    package = Path(web_root).absolute().parent
    # colcon 支持 isolated install 与 merge install 两种布局。
    if package.name == 'web_host' and package.parent.name == 'share':
        prefix = package.parent.parent
        if prefix.name == 'web_host':
            prefix = prefix.parent
        if prefix.name == 'install':
            source = prefix.parent / 'src' / 'web_host' / 'action_library'
            if (source.parent / 'package.xml').is_file() and source.is_dir():
                return source
    return actions_directory(web_root)


def _checked_path(directory: Path, name) -> Path:
    # 不 resolve：symlink-install 的内置动作允许指向源码目录。
    if not isinstance(name, str) or not _NAME_RE.fullmatch(name):
        raise ValueError("非法动作文件名（单层、字母数字._-、以 .json 结尾）")
    return directory / name


def action_summary(document) -> dict:
    """MotorTrajectory 载荷 → 列表用摘要；不是该格式就抛 ValueError。"""
    if not isinstance(document, dict):
        raise ValueError("不是 JSON 对象")
    ids = document.get("target_ids")
    points = document.get("points")
    if (not isinstance(ids, list) or not ids
            or not isinstance(points, list) or not points):
        raise ValueError("不是 MotorTrajectory 载荷（缺 target_ids/points）")
    times = []
    for point in points:
        if not isinstance(point, dict) or "time_from_start_ms" not in point:
            raise ValueError("路点缺 time_from_start_ms")
        value = point["time_from_start_ms"]
        if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("路点时间必须为 uint32 整数")
        times.append(value)
    if any(type(motor_id) is not int or not 0 <= motor_id <= 0xFFFF for motor_id in ids):
        raise ValueError("电机编号必须为 uint16 整数")
    return {
        "target_ids": ids,
        "target_count": len(ids),
        "point_count": len(points),
        "duration_ms": max(times) - min(times),
    }


def list_actions(web_root: str) -> list[dict]:
    """枚举动作库并附摘要；解析失败的文件保留条目并携带 error。"""
    directory = actions_directory(web_root)
    if not directory.is_dir():
        return []
    actions = []
    for path in sorted(directory.glob("*.json")):
        entry = {"name": path.name}
        try:
            entry.update(action_summary(json.loads(path.read_text(encoding="utf-8"))))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError,
                OSError) as exc:
            entry["error"] = str(exc)
        actions.append(entry)
    return actions


def read_action(web_root: str, name) -> str:
    """读取动作文件原文（保留原格式，供前端解析）。"""
    path = _checked_path(actions_directory(web_root), name)
    if not path.is_file():
        raise FileNotFoundError(name)
    return path.read_text(encoding="utf-8")


def save_action(web_root: str, name, text, overwrite: bool = False) -> dict:
    """导入动作到动作库（校验 JSON 与载荷格式）；同名需显式 overwrite。"""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("动作内容为空")
    if len(text.encode("utf-8")) > _MAX_TEXT_BYTES:
        raise ValueError("动作文件超过 8 MB 上限")
    summary = action_summary(json.loads(text))
    directory = actions_directory(web_root)
    directory.mkdir(parents=True, exist_ok=True)
    path = _checked_path(directory, name)
    # 覆盖仍写回 symlink-install 的源文件；暂存完成前不改动旧内容。
    destination = path.resolve() if overwrite else path
    try:
        mode = stat.S_IMODE(destination.stat().st_mode) if overwrite else None
    except FileNotFoundError:
        mode = None
    temporary = None
    try:
        candidate = destination.with_name(f".action-{uuid.uuid4().hex}")
        with candidate.open("x", encoding="utf-8") as handle:
            temporary = candidate
            if mode is not None:
                os.fchmod(handle.fileno(), mode)
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        if overwrite:
            os.replace(temporary, destination)
        else:
            # 完整文件以硬链接发布，目标已存在时原子拒绝，不能先检查再覆盖。
            os.link(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {"name": name, **summary}
