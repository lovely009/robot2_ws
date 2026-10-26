#!/usr/bin/env python3
"""等待电机总线模拟器就绪，再 exec micro-ROS Agent。"""

import os
import sys
import time


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: agent_after_sim_ready.py READY_FILE AGENT_TTY", file=sys.stderr)
        return 2
    ready_file, agent_tty = sys.argv[1:]
    deadline = time.monotonic() + 45.0
    while not os.path.isfile(ready_file):
        if time.monotonic() >= deadline:
            print(
                f"CAN motor simulator did not become ready: {ready_file}",
                file=sys.stderr,
            )
            return 1
        time.sleep(0.05)
    os.execvp(
        "ros2",
        [
            "ros2",
            "run",
            "micro_ros_agent",
            "micro_ros_agent",
            "serial",
            "--dev",
            agent_tty,
            "-v",
        ],
    )
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
