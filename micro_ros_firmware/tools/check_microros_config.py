# PlatformIO 构建前校验：按 microros_user.meta 核对缓存库的静态槽位与流缓冲配置。
# libmicroros.a 不会随 meta 变化自动重建；槽位不足会使实体初始化失败并反复重连。
# 校验随库打包的 rmw_microxrcedds_c/config.h，不依赖 ARM 工具链。

Import("env")  # noqa: F821  (SCons 注入)

import os
import re

# meta 里被本脚本校验的宏（顺序即报错顺序）。STREAM_HISTORY 单独处理——它的
# 生成逻辑带条件分支，不是简单比对数字。
SLOT_MACROS = (
    "RMW_UXRCE_MAX_PUBLISHERS",
    "RMW_UXRCE_MAX_SUBSCRIPTIONS",
    "RMW_UXRCE_MAX_SERVICES",
)
STREAM_HISTORY_MACRO = "RMW_UXRCE_STREAM_HISTORY"

# 每条宏"用超了"时具体会死在什么地方——点名到文件，比"配置不符"有用得多。
SLOT_CONSEQUENCE = {
    "RMW_UXRCE_MAX_PUBLISHERS": (
        "出站话题数超过静态槽位：新增 topic 的 rcl_publisher_init 会失败。\n"
        "  当前固件用 11 个（imu 2 + motor_bridge 8 + system_status 1）。"
    ),
    "RMW_UXRCE_MAX_SUBSCRIPTIONS": (
        "入站话题数超过静态槽位：新增 topic 的 rcl_subscription_init 会失败。\n"
        "  当前固件用 4 个。"
    ),
    "RMW_UXRCE_MAX_SERVICES": (
        "service 数超过静态槽位（rclc 参数服务器已于 2026-09-21 退役，\n"
        "  当前固件用 0 个；保留该余量是为了不动库配置，不是有人在用）。"
    ),
}

REPAIR_GUIDANCE = (
    "修复（库不会自动重建，必须显式刷新）：\n"
    "    ../micro_ros_agent/tools/refresh_microros_messages.py\n"
    "    pio run\n"
    "  该工具是**离线增量**刷新，不联网、不调用 clean_microros。\n"
    "  若必须走删目录重建（需网络可达 GitHub），方法见 AGENTS.md 的「高频踩坑」。"
)


def _libmicroros_dir():
    libdeps = env["PROJECT_LIBDEPS_DIR"]  # .pio/libdeps
    pio_env = env["PIOENV"]               # 环境名，如 stm32h723vgt6
    return os.path.join(
        libdeps, pio_env, "micro_ros_platformio", "libmicroros"
    )


def _meta_macro_values():
    """从 microros_user.meta 解析出各 RMW 宏的期望值。"""
    meta_name = str(
        env.GetProjectOption("board_microros_user_meta", "microros_user.meta")
    )
    meta_path = os.path.join(env["PROJECT_DIR"], meta_name)
    try:
        with open(meta_path, "r", encoding="utf-8") as handle:
            text = handle.read()
    except OSError as err:
        raise SystemExit(
            "无法读取 %s（board_microros_user_meta 指向的库配置覆盖）：%s\n"
            "该文件是库级配置的唯一真相源，本校验不会跳过。" % (meta_path, err)
        )
    return {
        match.group(1): int(match.group(2))
        for match in re.finditer(r"-D(RMW_UXRCE_[A-Z_]+)=(\d+)", text)
    }


def _fail(title, body):
    print("\n" + "=" * 72 + "\n" + title + "\n" + body + "\n" + "=" * 72 + "\n")
    env.Exit(1)  # noqa: F821  (SCons)


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

    expected = _meta_macro_values()

    # --- 三条静态槽位宏：meta 要求 vs 库里实际编出来的值 ---
    for macro in SLOT_MACROS:
        required = expected.get(macro)
        if required is None:
            continue
        match = re.search(r"^#define\s+%s\s+(\d+)" % macro, text, re.M)
        if match is None:
            continue  # 上游改了 config.h 结构：不误报，交由人工确认
        value = int(match.group(1))
        if value >= required:
            continue
        _fail(
            "micro-ROS 库配置不符：%s = %d（meta 要求 %d）" % (macro, value, required),
            "  %s\n\n"
            "%s\n\n"
            "该故障只在运行期暴露，编译期不会有任何提示，所以在这里拦下。\n"
            "这是库用旧配置构建过的证据——meta 改了但库没重建。\n\n%s"
            % (config_path, SLOT_CONSEQUENCE[macro], REPAIR_GUIDANCE),
        )

    # --- STREAM_HISTORY：结构校验（input/output 必须成对） ---
    required_stream = expected.get(STREAM_HISTORY_MACRO)
    if required_stream is None or "#else" not in text:
        return  # 上游改了 config.h 结构：不误报，交由人工确认
    # RMW 的 CMake 配置只有在 input/output 同时非空时才采用专用值；
    # 否则 config.h 会回落到 RMW_UXRCE_STREAM_HISTORY（默认 4）。
    # config.h 还会保留 CMake 条件的 inactive 分支。判断 #else 后的
    # fallback 值即可验证当前预处理结果：旧的“只给 INPUT=12”会落到
    # 4/4，新配置会落到 16/16。
    fallback = text.split("#else", 1)[1]
    if not all(
        re.search(
            r"^#define\s+%s_%s\s+%d\s*$" % (STREAM_HISTORY_MACRO, side, required_stream),
            fallback,
            re.M,
        )
        for side in ("INPUT", "OUTPUT")
    ):
        _fail(
            "micro-ROS 库配置不符：可靠轨迹输入缓冲未按预期生成",
            "  需要 input/output stream history = %d（各 8 KiB，512B MTU）。\n"
            "  当前生成头：%s\n\n"
            "这会让 MotorTrajectory 的可靠分片在到达 MCU 前失败或使会话异常。\n"
            "（上游要求 input/output 成对配置；只给 INPUT 会静默回退到默认 4。）\n\n%s"
            % (required_stream, config_path, REPAIR_GUIDANCE),
        )


_check()
