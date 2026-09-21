# PlatformIO 构建前校验（pre script，由 platformio.ini 的 extra_scripts 引入）。
#
# 目的：挡住一类**静默且难排查**的故障——libmicroros.a 是用旧配置构建的。
#
# 背景：RMW_UXRCE_MAX_SERVICES 决定 rmw 库内部 custom_services[] 的静态数组大小，
# 是**库编译期**的事实（见 platformio.ini 的「micro-ROS 库级配置覆盖」）。而
# micro_ros_platformio 的 library_builder 见到 libmicroros.a 就 early return，
# **不会**因 meta 变化自动重建。于是：把整个工程目录（含 .pio 缓存）拷给别人、
# 或从旧版本升级上来时，库仍是旧配置，pio run 却一路成功，直到烧录后节点在
# WAIT_AGENT 无限重连——看不到任何编译期线索。
#
# 校验的是 libmicroros/include/ 里的 config.h：它与库同源（打包时从
# build/mcu/install/include 复制），因此能代表库的实际编译配置。
# 更严格的 nm 校验（custom_services 符号大小）见 AGENTS.md，此处不引入工具链
# 依赖，保证在没装 ARM toolchain 的机器上也能跑。

Import("env")  # noqa: F821  (SCons 注入)

import os
import re

# 与 platformio.ini / microros_user.meta 保持一致（rclc 参数服务器的 5 个 service）。
REQUIRED_MAX_SERVICES = 5
REQUIRED_STREAM_HISTORY = 16


def _libmicroros_dir():
    libdeps = env["PROJECT_LIBDEPS_DIR"]  # .pio/libdeps
    pio_env = env["PIOENV"]               # 环境名，如 stm32h723vgt6
    return os.path.join(
        libdeps, pio_env, "micro_ros_platformio", "libmicroros"
    )


def _check():
    libmicroros = _libmicroros_dir()
    config_path = os.path.join(
        libmicroros, "include", "rmw_microxrcedds_c", "config.h"
    )
    if not os.path.exists(config_path):
        # 库尚未构建：首次构建会带上 microros_user.meta，配置天然正确。
        return
    try:
        with open(config_path, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError:
        return  # 读不了就不阻断构建——本校验是辅助，不是构建契约

    match = re.search(r"^#define\s+RMW_UXRCE_MAX_SERVICES\s+(\d+)", text, re.M)
    if match is None:
        return  # 上游改了 config.h 结构：不误报，交由人工确认
    value = int(match.group(1))
    if value >= REQUIRED_MAX_SERVICES:
        # RMW 的 CMake 配置只有在 input/output 同时非空时才采用专用值；
        # 否则 config.h 会回落到 RMW_UXRCE_STREAM_HISTORY（默认 4）。
        # config.h 还会保留 CMake 条件的 inactive 分支。判断 #else 后的
        # fallback 值即可验证当前预处理结果：旧的“只给 INPUT=12”会落到
        # 4/4，新配置会落到 16/16。
        fallback = text.split("#else", 1)[1] if "#else" in text else ""
        input_value = re.search(
            r"^#define\s+RMW_UXRCE_STREAM_HISTORY_INPUT\s+%d\s*$"
            % REQUIRED_STREAM_HISTORY,
            fallback,
            re.M,
        )
        output_value = re.search(
            r"^#define\s+RMW_UXRCE_STREAM_HISTORY_OUTPUT\s+%d\s*$"
            % REQUIRED_STREAM_HISTORY,
            fallback,
            re.M,
        )
        if not all((input_value, output_value)):
            print(
                "\n"
                + "=" * 72
                + "\nmicro-ROS 库配置不符：可靠轨迹输入缓冲未按预期生成\n"
                "  需要 input/output stream history = %d（各 8 KiB，512B MTU）。\n"
                "  当前生成头：%s\n\n"
                "这会让 MotorTrajectory 的可靠分片在到达 MCU 前失败或使会话异常。\n"
                "修复：运行 ../micro_ros_agent/tools/refresh_microros_messages.py，\n"
                "再重新执行 pio run；不要用 clean_microros。\n"
                % (REQUIRED_STREAM_HISTORY, config_path)
                + "=" * 72
                + "\n"
            )
            env.Exit(1)  # noqa: F821  (SCons)
        return

    print(
        "\n"
        + "=" * 72
        + "\nmicro-ROS 库配置不符：RMW_UXRCE_MAX_SERVICES = %d（需要 >= %d）\n"
        "  %s\n\n"
        "rclc 参数服务器固定要建 5 个 service（get/get_types/set/list/describe），\n"
        "此配置下初始化会在第 2 个 service 上失败 → param_bridge_create 失败 →\n"
        "ros_bridge 整体回滚 → 节点在 WAIT_AGENT 无限重连。\n"
        "该故障只在运行期暴露，编译期不会有任何提示，所以在这里拦下。\n\n"
        "修复（库不会自动重建，必须手动删掉整个 libmicroros/ 再构建）：\n"
        "    rm -rf %s\n"
        "    pio run\n"
        "  注意：这条路径会 git pull 各 micro-ROS 依赖，需要网络可达 GitHub；\n"
        "  离线重建方法见 AGENTS.md 的「高频踩坑」。\n"
        % (value, REQUIRED_MAX_SERVICES, config_path, libmicroros)
        + "=" * 72
        + "\n"
    )
    env.Exit(1)  # noqa: F821  (SCons)


_check()
