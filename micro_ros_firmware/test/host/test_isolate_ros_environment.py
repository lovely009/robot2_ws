#!/usr/bin/env python3
"""验证固件构建不会从主机 ROS 路径查找依赖。"""

import os
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "isolate_ros_environment.py"


class IsolateRosEnvironmentTest(unittest.TestCase):
    def run_script(self, process_env, build_env):
        # 仅模拟 SCons 注入；执行实际构建脚本并检查环境副作用。
        with patch.dict(os.environ, process_env, clear=True):
            runpy.run_path(str(SCRIPT), init_globals={
                "Import": lambda _name: None,
                "env": {"ENV": build_env},
            })
            return dict(os.environ), build_env

    def test_path_isolated_without_ros_distro(self):
        process, build = self.run_script(
            {"PATH": "/opt/ros/humble/bin:/usr/bin"},
            {"PATH": "/arm/bin:/opt/ros/humble/bin:/usr/bin"},
        )
        self.assertEqual(process["PATH"], "/usr/bin")
        self.assertEqual(build["PATH"], "/arm/bin:/usr/bin")

    def test_all_ros_distributions_and_search_paths_are_isolated(self):
        values = {key: "/opt/ros/humble:/opt/ros/jazzy/lib:/custom"
                  for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                              "COLCON_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")}
        process, build = self.run_script(values, values.copy())
        for key in values:
            self.assertEqual(process[key], "/custom")
            self.assertEqual(build[key], "/custom")

    def test_non_ros_environment_is_preserved(self):
        values = {"PATH": "/arm/bin:/usr/bin", "OTHER": "keep",
                  "CMAKE_PREFIX_PATH": "/opt/ros-tools:/custom"}
        process, build = self.run_script(values, values.copy())
        self.assertEqual(process, values)
        self.assertEqual(build, values)

    def test_ros_only_search_paths_are_removed(self):
        values = {key: "/opt/ros/humble/bin:/opt/ros/jazzy/lib"
                  for key in ("PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                              "COLCON_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")}
        process, build = self.run_script(values, values.copy())
        self.assertEqual(process, {})
        self.assertEqual(build, {})

    def test_missing_paths_remain_absent(self):
        process, build = self.run_script({}, {})
        self.assertEqual(process, {})
        self.assertEqual(build, {})


if __name__ == "__main__":
    unittest.main()
