"""验证 colcon 只发现重命名后的仿真包。"""

from pathlib import Path
import subprocess
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[3]


def test_integration_package_has_final_identity() -> None:
    """Gazebo 集成包必须一次性迁到最终目录、manifest 与构建类型。"""

    package_xml = ROOT / "src/humanoid_gazebo/package.xml"
    assert package_xml.exists()
    assert ElementTree.parse(package_xml).findtext("name") == "humanoid_gazebo"
    assert (ROOT / "src/humanoid_gazebo/CMakeLists.txt").exists()
    legacy_directory = ROOT / "src" / ("humanoid_" + "canfd_gazebo")
    assert not legacy_directory.exists()


def test_colcon_discovers_shared_motor_and_humanoid_packages() -> None:
    result = subprocess.run(
        ["colcon", "list", "--names-only"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert result.stdout.splitlines() == ["humanoid_gazebo", "motor_sim"]


def test_source_tree_contains_only_approved_simulation_packages() -> None:
    package_roots = sorted(
        package.parent.relative_to(ROOT).as_posix()
        for package in (ROOT / "src").glob("*/package.xml")
    )
    assert package_roots == ["src/humanoid_gazebo", "src/motor_sim"]


def test_runtime_sources_and_user_docs_have_no_removed_architecture_names() -> None:
    forbidden = (
        "Socket" + "CAN",
        "v" + "can",
        "motor_" + "hardware_interface",
        "motor_" + "drive",
        "humanoid_" + "canfd_gazebo",
        "driver_" + "loopback",
        "model_" + "sim.launch.py",
    )
    paths = [ROOT / "README.md", ROOT / "AGENTS.md", ROOT / "CONTEXT.md"]
    paths.extend(
        path
        for path in (ROOT / "src/humanoid_gazebo").rglob("*")
        if path.is_file()
        and "docs/protocol" not in path.as_posix()
        and "test" not in path.parts
        and "__pycache__" not in path.parts
    )
    text = "\n".join(path.read_text(errors="ignore") for path in paths)
    assert not [token for token in forbidden if token in text]
