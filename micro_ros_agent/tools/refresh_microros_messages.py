#!/usr/bin/env python3
"""增量刷新固件的 micro-ROS 自定义消息，不下载或重建依赖仓库。

PlatformIO 的 micro_ros_platformio 只检查 ``libmicroros.a`` 是否存在，修改
``extra_packages`` 后普通 ``pio run`` 不会自动重生成类型。本工具复用已经存在的
MCU/dev 构建环境，只复制权威消息包、执行增量 colcon build，并重新打包静态库。
缓存不完整时直接失败；不会退化为联网 clone，也不会调用 clean_microros。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--environment",
        default="stm32h723vgt6",
        help="PlatformIO 环境名（默认：stm32h723vgt6）",
    )
    return parser.parse_args()


def require(path: Path, purpose: str) -> None:
    if not path.exists():
        raise SystemExit(
            f"缺少 {purpose}: {path}\n"
            "为避免隐式联网，本工具不会创建 micro-ROS 基础环境。请先确认现有 "
            "PlatformIO 缓存完整，或由维护者在允许联网的环境中执行首次 pio run。"
        )


def main() -> None:
    args = parse_args()
    # 本工具位于 micro_ros_agent/tools/（与固件同仓库）；权威消息包与
    # PlatformIO 缓存都锚定在固件目录。
    firmware_root = Path(__file__).resolve().parents[2] / "micro_ros_firmware"
    platformio_root = (
        firmware_root
        / ".pio"
        / "libdeps"
        / args.environment
        / "micro_ros_platformio"
    )
    package_link = firmware_root / "extra_packages" / "can_bridge_msgs"
    require(package_link, "extra_packages/can_bridge_msgs 权威消息入口")
    require(platformio_root / "build" / "dev" / "install" / "setup.sh", "micro-ROS dev 环境")
    require(platformio_root / "build" / "mcu" / "src", "micro-ROS MCU 源码缓存")
    require(platformio_root / "platformio_toolchain.cmake", "PlatformIO CMake toolchain")

    # 本脚本独立运行时不经 pio 进程，PATH 里没有 PlatformIO 注入的 ARM 工具链；
    # toolchain 文件用裸名调用 arm-none-eabi-gcc-ar，缺路径会在静态库归档阶段
    # 报 “Error running link command: No such file or directory”。显式补上。
    toolchain_bin = (
        Path.home() / ".platformio" / "packages" / "toolchain-gccarmnoneeabi" / "bin"
    )
    if toolchain_bin.is_dir() and str(toolchain_bin) not in os.environ.get("PATH", "").split(
        os.pathsep
    ):
        os.environ["PATH"] = str(toolchain_bin) + os.pathsep + os.environ.get("PATH", "")

    resolved_package = package_link.resolve(strict=True)
    expected_package = (firmware_root.parent / "micro_ros_agent" / "src" / "can_bridge_msgs").resolve()
    if resolved_package != expected_package:
        raise SystemExit(
            "extra_packages/can_bridge_msgs 未指向权威消息包：\n"
            f"  actual:   {resolved_package}\n"
            f"  expected: {expected_package}"
        )

    sys.path.insert(0, str(platformio_root))
    from microros_utils.library_builder import Build  # pylint: disable=import-outside-toplevel

    python_activate = Path.home() / ".platformio" / "penv" / "bin" / "activate"
    require(python_activate, "PlatformIO Python 环境")

    builder = Build(
        library_folder=str(platformio_root),
        packages_folder=str(firmware_root / "extra_packages"),
        distro="humble",
        python_env=str(python_activate),
    )
    builder.check_env()

    # 有意不调用 download_dev_environment()/download_mcu_environment()：这两个步骤
    # 可能访问网络。这里只覆盖本仓库维护的 extra package 并增量重建现有工作区。
    builder.download_extra_packages()
    builder.build_mcu_environment(
        str(platformio_root / "metas" / "colcon.meta"),
        str(platformio_root / "platformio_toolchain.cmake"),
    )
    builder.package_mcu_library()

    generated_header = (
        platformio_root
        / "libmicroros"
        / "include"
        / "can_bridge_msgs"
        / "msg"
        / "detail"
        / "motor_topology_entry__struct.h"
    )
    require(generated_header, "重新生成的 can_bridge_msgs 头文件")
    header_text = generated_header.read_text(encoding="utf-8")
    if "PROTOCOL_CERB" not in header_text or "PROTOCOL_CREB" in header_text:
        raise SystemExit("生成结果仍包含旧协议常量，拒绝报告成功")

    print("micro-ROS 自定义消息已从权威包增量刷新；未下载依赖。")


if __name__ == "__main__":
    main()
