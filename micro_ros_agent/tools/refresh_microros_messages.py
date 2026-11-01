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


HOST_ROS_PREFIX = "/opt/ros"

# 在 source 过主机 ROS 的 shell 里直接运行本脚本时，/opt/ros/<distro> 会经由
# 这些变量进入 MCU 的 colcon 工作区。MCU 构建用 --packages-ignore-regex=.*_cpp
# 排除了 rosidl_typesupport_cpp，但 symlink-install 的 rosidl_default_generators
# 仍会 find_package(rosidl_typesupport_cpp)；一旦从主机 ROS 找到，它又会用
# AMENT_PREFIX_PATH（已被 check_env 清掉 /opt/ros）查 ament 资源，最终报
# "No 'rosidl_typesupport_cpp' found"。colcon 命令内部会 source 工作区
# setup.sh 重新生成这些变量，因此整体删除是安全的。
ENV_KEYS_TO_DROP = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "PYTHONPATH",
    "ROS_PACKAGE_PATH",
)


def isolate_from_host_ros(env: dict[str, str]) -> list[str]:
    """清除主机 ROS 注入的构建变量，返回被清理的变量名。"""
    cleaned: list[str] = []
    for key in ENV_KEYS_TO_DROP:
        if key in env:
            del env[key]
            cleaned.append(key)

    for key in ("PATH", "LD_LIBRARY_PATH"):
        value = env.get(key)
        if value is None:
            continue
        entries = value.split(os.pathsep)
        kept = [item for item in entries
                if item and not item.startswith(HOST_ROS_PREFIX)]
        if kept != entries:
            if kept:
                env[key] = os.pathsep.join(kept)
            else:
                del env[key]
            cleaned.append(key)
    return cleaned


def purge_host_ros_caches(build_folder: Path) -> list[Path]:
    """删除曾被主机 ROS 污染的 CMakeCache.txt（仅生成缓存，不碰源码）。"""
    purged: list[Path] = []
    for cache in sorted(build_folder.glob("*/CMakeCache.txt")):
        try:
            text = cache.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if HOST_ROS_PREFIX in text:
            cache.unlink()
            purged.append(cache)
    return purged


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

    # check_env 只清 AMENT_PREFIX_PATH 和 PATH 中的 ROS 项，CMAKE_PREFIX_PATH 等
    # 仍可能指向 /opt/ros；同时旧 CMakeCache 会把 rosidl_typesupport_cpp_DIR
    # 固化成主机路径，即使环境干净也会被直接采用。两者都必须先清理，
    # 否则 colcon 的增量 configure 会再次失败。
    cleaned = isolate_from_host_ros(builder.env)
    if cleaned:
        print("已隔离主机 ROS 构建环境: " + ", ".join(cleaned))
    purged = purge_host_ros_caches(Path(builder.build_folder) / "mcu" / "build")
    if purged:
        print("已删除被主机 ROS 污染的 CMake 缓存:")
        for cache in purged:
            print(f"\t - {cache}")

    # 有意不调用 download_dev_environment()/download_mcu_environment()：这两个步骤
    # 可能访问网络。这里只覆盖本仓库维护的 extra package 并增量重建现有工作区。
    builder.download_extra_packages()
    # 第三个参数是 board_microros_user_meta：官方流程（extra_script.py）用
    # builder.run(meta, toolchain, microros_user_meta) 把它附加到 colcon 的
    # --metas 列表后面，覆盖包内 metas/colcon.meta 的默认值。漏掉它的后果是
    # 库被按**默认**配置重建——本固件要 RMW_UXRCE_MAX_PUBLISHERS=12 /
    # MAX_SUBSCRIPTIONS=6 / MAX_SERVICES=5，默认分别是 4 / 4 / 1：重建后
    # tools/check_microros_config.py 会在下一次 pio run 拦下（且只有到运行期
    # 才会以 WAIT_AGENT 无限重连的形式暴露）。
    user_meta = firmware_root / "microros_user.meta"
    require(user_meta, "microros_user.meta 库级配置覆盖")
    builder.build_mcu_environment(
        str(platformio_root / "metas" / "colcon.meta"),
        str(platformio_root / "platformio_toolchain.cmake"),
        str(user_meta),
    )
    builder.package_mcu_library()

    detail_dir = (
        platformio_root / "libmicroros" / "include" / "can_bridge_msgs" / "msg" / "detail"
    )
    generated_header = detail_dir / "motor_topology_entry__struct.h"
    require(generated_header, "重新生成的 can_bridge_msgs 头文件")
    header_text = generated_header.read_text(encoding="utf-8")
    if "PROTOCOL_CERB" not in header_text or "PROTOCOL_CREB" in header_text:
        raise SystemExit("生成结果仍包含旧协议常量，拒绝报告成功")

    # 新增消息同样要落到库里——只刷新了协议常量而新类型缺失，说明 colcon 用了
    # 旧的 install 目录（增量构建没看到新 .msg），此时 pio run 会在编译期报
    # "no member named 'param_set_array'" 之类的错，指向性远不如这里直接点名。
    for struct_name in ("control_lease", "param_set_array", "system_command", "system_status",
                        "motor_control_state", "motor_control_state_array"):
        require(detail_dir / f"{struct_name}__struct.h", f"新消息 {struct_name} 的生成头")

    # V3 的 13 个 Agent* 类型必须全部生成；任何缺失都说明本次 colcon 复用了
    # 旧 install/build 缓存，而不是复制进来的权威 .msg。
    agent_structs = (
        "agent_wire_contract",
        "agent_command",
        "agent_motion",
        "agent_trajectory_request",
        "agent_config_request",
        "agent_config_result",
        "agent_trajectory_receipt",
        "agent_trajectory_state",
        "agent_motor_status",
        "agent_status",
        "agent_device_info",
        "agent_operation_detail",
        "agent_info",
    )
    for struct_name in agent_structs:
        require(detail_dir / f"{struct_name}__struct.h",
                f"V3 消息 {struct_name} 的生成头")

    contract_text = (detail_dir / "agent_wire_contract__struct.h").read_text(encoding="utf-8")
    if "SCHEMA_VERSION" not in contract_text:
        raise SystemExit("AgentWireContract 生成头缺少 SCHEMA_VERSION，拒绝报告成功")

    # 标准 sensor_msgs/Temperature 走同一条 MCU 类型生成链；V3 的
    # /micro_ros_up/imu/temperature 依赖它，不能只在主机的 Humble 里存在。
    # rosidl_generator_c 把结构体放在 msg/detail/ 下，公开入口是 msg/temperature.h。
    require(
        platformio_root / "libmicroros" / "include" / "sensor_msgs" / "msg"
        / "detail" / "temperature__struct.h",
        "sensor_msgs/Temperature 的生成头",
    )

    print("micro-ROS 自定义消息已从权威包增量刷新；未下载依赖。")


if __name__ == "__main__":
    main()
