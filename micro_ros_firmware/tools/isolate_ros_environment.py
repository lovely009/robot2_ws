# 避免 micro-ROS 交叉编译从主机 ROS 环境查找依赖。
Import("env")  # noqa: F821（SCons 注入）

import os
import re


for environment in (os.environ, env["ENV"]):
    for key in (
        "PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "PYTHONPATH", "LD_LIBRARY_PATH",
    ):
        if key in environment:
            paths = [
                path for path in environment[key].split(os.pathsep)
                if not re.match(r"^/opt/ros/[^/]+(?:/|$)", path)
            ]
            if paths:
                environment[key] = os.pathsep.join(paths)
            else:
                del environment[key]
