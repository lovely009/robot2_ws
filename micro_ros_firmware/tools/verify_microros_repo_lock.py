#!/usr/bin/env python3
"""校验 micro-ROS 内源仓库提交锁与本地构建缓存是否一致。

micro_ros_platformio cfee17f 不支持项目级内源仓库清单覆盖，26 个仓库按分支
浮动拉取（见 microros_repos.lock 头部说明）。本脚本把最近一次验证构建的提交
记录在 microros_repos.lock 中，并与 .pio 缓存里的实际 git 检出核对：

- 默认模式：lock 中每个仓库都必须在缓存中存在且 url/分支/SHA 完全一致，
  否则以非零状态退出，提示干净重建会引入漂移。
- --write：按 lock 中已登记的仓库名刷新 url/分支/SHA（不新增仓库，避免把
  extra_packages 里的自有消息包误锁）。

仅调用本地 git，不访问网络；路径全部基于工程目录推导，不记录机器绝对路径。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml

LOCK_FILENAME = "microros_repos.lock"
DEFAULT_ENV = "stm32h723vgt6"
LOCK_HEADER = """\
# micro-ROS 内源仓库提交锁（humble，26 个，对应当前已验证构建组合）。
# 来源：micro_ros_platformio@cfee17f 的 microros_utils/repositories.py。
# 注意：上游库只支持 extra_packages.repos 追加仓库，没有内置仓库的提交覆盖
# 接口，因此本文件不被 pio 构建强制消费，仅用于审计与漂移守门；
# 干净重建（clean_microros 后）仍按分支浮动拉取，随后应运行：
#   python3 tools/verify_microros_repo_lock.py
# 校验失败说明上游分支已漂移，需要评估后用 --write 更新本文件并重新验证。
"""


def find_src_roots(project_dir: Path, environment: str) -> list[Path]:
    """返回 micro-ROS dev/mcu 两套源码根目录（可能尚未生成）。"""
    build_dir = (
        project_dir
        / ".pio"
        / "libdeps"
        / environment
        / "micro_ros_platformio"
        / "build"
    )
    return [build_dir / "dev" / "src", build_dir / "mcu" / "src"]


def _git(repo_dir: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo_dir), *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def read_repo_state(repo_dir: Path) -> dict[str, str]:
    """读取单个 git 检出的 url/分支/SHA。"""
    return {
        "url": _git(repo_dir, "config", "--get", "remote.origin.url"),
        "branch": _git(repo_dir, "rev-parse", "--abbrev-ref", "HEAD"),
        "sha": _git(repo_dir, "rev-parse", "HEAD"),
    }


def collect_cache(project_dir: Path, environment: str) -> dict[str, dict[str, str]]:
    """收集缓存中全部 git 仓库的状态，键为目录名（仓库名）。"""
    cache: dict[str, dict[str, str]] = {}
    for src_root in find_src_roots(project_dir, environment):
        if not src_root.is_dir():
            continue
        for repo_dir in sorted(p for p in src_root.iterdir() if p.is_dir()):
            try:
                cache[repo_dir.name] = read_repo_state(repo_dir)
            except subprocess.CalledProcessError:
                # 非 git 目录（如复制进来的 extra_packages）直接跳过
                continue
    return cache


def load_lock(lock_path: Path) -> dict:
    data = yaml.safe_load(lock_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("repositories"), dict):
        raise ValueError(f"{lock_path} 缺少 repositories 段")
    return data


def write_lock(lock_path: Path, distro: str, repositories: dict) -> None:
    # sort_keys=False：保留调用方给定的仓库顺序，--write 刷新时不重排整个锁文件；
    # 内层字段统一成 branch/sha/url，与手写锁的书写顺序一致。
    ordered = {
        name: {
            "branch": state["branch"],
            "sha": state["sha"],
            "url": state["url"],
        }
        for name, state in repositories.items()
    }
    payload = yaml.safe_dump(
        {"distro": distro, "repositories": ordered},
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
    )
    lock_path.write_text(LOCK_HEADER + payload, encoding="utf-8")


def evaluate(
    lock_repos: dict[str, dict[str, str]],
    cache: dict[str, dict[str, str]],
) -> list[str]:
    """返回差异说明列表，空列表表示完全一致。"""
    errors: list[str] = []
    for name in sorted(lock_repos):
        expected = lock_repos[name]
        actual = cache.get(name)
        if actual is None:
            errors.append(f"{name}: 缓存中缺失（需要先完成一次 micro-ROS 构建）")
            continue
        for field in ("url", "branch", "sha"):
            if expected.get(field) != actual.get(field):
                errors.append(
                    f"{name}.{field}: lock={expected.get(field)} 缓存={actual.get(field)}"
                )
    return errors


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, default=project_dir)
    parser.add_argument("--environment", default=DEFAULT_ENV)
    parser.add_argument(
        "--lock", type=Path, default=project_dir / LOCK_FILENAME,
        help="提交锁路径（默认：工程根目录下的 microros_repos.lock）",
    )
    parser.add_argument(
        "--write", action="store_true",
        help="按锁中已登记的仓库名从缓存刷新状态，而不是只做校验",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    lock_data = load_lock(args.lock)
    cache = collect_cache(args.project_dir, args.environment)
    if not cache:
        print("未找到 micro-ROS 构建缓存，请先完成一次 pio run")
        return 1

    if args.write:
        refreshed = {}
        missing = [name for name in lock_data["repositories"] if name not in cache]
        if missing:
            print("以下锁内仓库不在缓存中，无法刷新：" + ", ".join(sorted(missing)))
            return 1
        for name in lock_data["repositories"]:
            refreshed[name] = cache[name]
        write_lock(args.lock, lock_data.get("distro", "humble"), refreshed)
        print(f"已刷新 {args.lock}，共 {len(refreshed)} 个仓库")
        return 0

    errors = evaluate(lock_data["repositories"], cache)
    if errors:
        print("micro-ROS 内源仓库与提交锁不一致：")
        for line in errors:
            print(f"  - {line}")
        print("干净重建将消费上游分支最新提交；确认变更后可用 --write 更新锁文件。")
        return 1
    print(f"全部 {len(lock_data['repositories'])} 个内源仓库与 {args.lock.name} 一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
