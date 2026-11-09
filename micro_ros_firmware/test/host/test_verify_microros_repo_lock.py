#!/usr/bin/env python3
"""verify_microros_repo_lock 的主机单元测试：临时目录内真实 git 仓库，不触网。"""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import verify_microros_repo_lock as verifier  # noqa: E402


def make_git_repo(path: Path, url: str, branch: str) -> str:
    path.mkdir(parents=True)
    subprocess.run(["git", "init", "-b", branch, str(path)], check=True,
                   capture_output=True)
    subprocess.run(["git", "-C", str(path), "config", "user.email", "t@example.com"],
                   check=True)
    subprocess.run(["git", "-C", str(path), "config", "user.name", "tester"],
                   check=True)
    (path / "README").write_text("first\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(path), "add", "README"], check=True)
    subprocess.run(["git", "-C", str(path), "commit", "-m", "init"], check=True,
                   capture_output=True)
    subprocess.run(["git", "-C", str(path), "remote", "add", "origin", url],
                   check=True)
    return verifier.read_repo_state(path)["sha"]


class RepoLockTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.project = Path(self._tmp.name)
        self.env = "stm32h723vgt6"
        dev_src = self.project / ".pio/libdeps" / self.env / \
            "micro_ros_platformio/build/dev/src"
        mcu_src = self.project / ".pio/libdeps" / self.env / \
            "micro_ros_platformio/build/mcu/src"
        self.dev_repo = dev_src / "ament_cmake"
        self.mcu_repo = mcu_src / "micro-CDR"
        make_git_repo(self.dev_repo, "https://github.com/ament/ament_cmake", "humble")
        make_git_repo(self.mcu_repo, "https://github.com/eProsima/micro-CDR", "ros2")
        # 非 git 目录必须被静默忽略（模拟复制进来的 extra_packages）
        (mcu_src / "can_bridge_msgs").mkdir(parents=True)
        self.lock_path = self.project / verifier.LOCK_FILENAME

    def tearDown(self):
        self._tmp.cleanup()

    def test_collect_cache_skips_non_git(self):
        cache = verifier.collect_cache(self.project, self.env)
        self.assertEqual(set(cache), {"ament_cmake", "micro-CDR"})
        self.assertEqual(cache["ament_cmake"]["branch"], "humble")
        self.assertEqual(cache["micro-CDR"]["branch"], "ros2")

    def test_write_then_verify_passes(self):
        cache = verifier.collect_cache(self.project, self.env)
        verifier.write_lock(self.lock_path, "humble", cache)

        loaded = verifier.load_lock(self.lock_path)
        self.assertEqual(loaded["distro"], "humble")
        self.assertEqual(set(loaded["repositories"]), {"ament_cmake", "micro-CDR"})
        self.assertEqual(verifier.evaluate(loaded["repositories"], cache), [])

        argv = ["verify", "--project-dir", str(self.project),
                "--environment", self.env, "--lock", str(self.lock_path)]
        with patch.object(sys, "argv", argv):
            self.assertEqual(verifier.main(), 0)

    def test_drifted_sha_fails_verification(self):
        cache = verifier.collect_cache(self.project, self.env)
        verifier.write_lock(self.lock_path, "humble", cache)

        (self.mcu_repo / "README").write_text("second\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(self.mcu_repo), "add", "README"], check=True)
        subprocess.run(["git", "-C", str(self.mcu_repo), "commit", "-m", "drift"],
                       check=True, capture_output=True)

        drifted = verifier.collect_cache(self.project, self.env)
        errors = verifier.evaluate(
            verifier.load_lock(self.lock_path)["repositories"], drifted)
        self.assertTrue(any("micro-CDR.sha" in line for line in errors))

        argv = ["verify", "--project-dir", str(self.project),
                "--environment", self.env, "--lock", str(self.lock_path)]
        with patch.object(sys, "argv", argv):
            self.assertEqual(verifier.main(), 1)

    def test_write_preserves_lock_order(self):
        # --write 刷新不得重排整个锁文件，避免真实漂移被全文件 diff 淹没
        cache = verifier.collect_cache(self.project, self.env)
        ordered = {"micro-CDR": cache["micro-CDR"],
                   "ament_cmake": cache["ament_cmake"]}
        verifier.write_lock(self.lock_path, "humble", ordered)

        argv = ["verify", "--project-dir", str(self.project),
                "--environment", self.env, "--lock", str(self.lock_path), "--write"]
        with patch.object(sys, "argv", argv):
            self.assertEqual(verifier.main(), 0)

        text = self.lock_path.read_text(encoding="utf-8")
        self.assertLess(text.index("micro-CDR"), text.index("ament_cmake"))
        self.assertLess(text.index("branch"), text.index("sha"))
        self.assertLess(text.index("sha"), text.index("url"))

    def test_missing_repo_and_refresh_write(self):
        cache = verifier.collect_cache(self.project, self.env)
        verifier.write_lock(self.lock_path, "humble", cache)
        import shutil
        shutil.rmtree(self.dev_repo)

        remaining = verifier.collect_cache(self.project, self.env)
        errors = verifier.evaluate(
            verifier.load_lock(self.lock_path)["repositories"], remaining)
        self.assertTrue(any("ament_cmake" in line and "缺失" in line for line in errors))

        # --write 遇到缓存缺仓库时必须失败，不能静默缩小锁定集合
        argv = ["verify", "--project-dir", str(self.project),
                "--environment", self.env, "--lock", str(self.lock_path), "--write"]
        with patch.object(sys, "argv", argv):
            self.assertEqual(verifier.main(), 1)


if __name__ == "__main__":
    unittest.main()
