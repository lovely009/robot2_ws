"""MCU 消息刷新脚本的主机 ROS 构建隔离单元测试。"""
import tempfile
import unittest
from pathlib import Path

import refresh_microros_messages as refresh


class HostRosIsolationTest(unittest.TestCase):
    def test_drops_host_ros_environment(self):
        env = {
            "PATH": "/opt/ros/humble/bin:/usr/bin",
            "LD_LIBRARY_PATH": "/opt/ros/humble/lib",
            "CMAKE_PREFIX_PATH": "/opt/ros/humble",
            "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages",
            "HOME": "/home/tester",
        }
        cleaned = refresh.isolate_from_host_ros(env)

        self.assertEqual(env["PATH"], "/usr/bin")
        self.assertEqual(env["HOME"], "/home/tester")
        self.assertNotIn("LD_LIBRARY_PATH", env)
        for key in ("CMAKE_PREFIX_PATH", "PYTHONPATH"):
            self.assertNotIn(key, env)
        self.assertEqual(
            set(cleaned),
            {"PATH", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH"},
        )

    def test_keeps_non_host_ros_environment(self):
        env = {"PATH": "/usr/bin:/home/tester/.platformio/penv/bin"}
        cleaned = refresh.isolate_from_host_ros(env)

        self.assertEqual(env["PATH"], "/usr/bin:/home/tester/.platformio/penv/bin")
        self.assertEqual(cleaned, [])

    def test_purges_only_host_ros_caches(self):
        with tempfile.TemporaryDirectory() as tmp:
            build_folder = Path(tmp) / "build"
            dirty = build_folder / "pkg_dirty" / "CMakeCache.txt"
            clean = build_folder / "pkg_clean" / "CMakeCache.txt"
            stale = build_folder / "pkg_stale" / "CMakeCache.txt-NOTFOUND"
            for path in (dirty, clean, stale):
                path.parent.mkdir(parents=True)
            dirty.write_text("rosidl_typesupport_cpp_DIR=/opt/ros/humble/share\n")
            clean.write_text("rosidl_typesupport_cpp_DIR=NOTFOUND\n")
            stale.write_text("/opt/ros/humble\n")

            purged = refresh.purge_host_ros_caches(build_folder)

            self.assertEqual(purged, [dirty])
            self.assertFalse(dirty.exists())
            self.assertTrue(clean.exists())
            self.assertTrue(stale.exists())


if __name__ == "__main__":
    unittest.main()
