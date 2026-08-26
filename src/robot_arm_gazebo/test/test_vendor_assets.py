from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_SHA = "18e6f603b3ebc2ec479fecb62d6be544b15755e9"


def test_exact_ur5e_vendor_manifest() -> None:
    vendor = PACKAGE_ROOT / "vendor" / "ur_description"
    assert (vendor / "LICENSE").is_file()
    assert UPSTREAM_SHA in (vendor / "UPSTREAM.md").read_text()
    assert len(list((vendor / "meshes/ur5e/visual").glob("*.dae"))) == 7
    assert len(list((vendor / "meshes/ur5e/collision").glob("*.stl"))) == 7


def test_only_approved_description_files_are_vendored() -> None:
    vendor = PACKAGE_ROOT / "vendor" / "ur_description"
    expected = {
        "LICENSE",
        "UPSTREAM.md",
        "config/initial_positions.yaml",
        "config/ur5e/default_kinematics.yaml",
        "config/ur5e/joint_limits.yaml",
        "config/ur5e/physical_parameters.yaml",
        "config/ur5e/visual_parameters.yaml",
        "urdf/inc/ur_common.xacro",
        "urdf/inc/ur_transmissions.xacro",
        "urdf/ur_macro.xacro",
    }
    expected.update(
        f"meshes/ur5e/{kind}/{name}.{extension}"
        for kind, extension in (("visual", "dae"), ("collision", "stl"))
        for name in (
            "base",
            "shoulder",
            "upperarm",
            "forearm",
            "wrist1",
            "wrist2",
            "wrist3",
        )
    )
    actual = {
        path.relative_to(vendor).as_posix()
        for path in vendor.rglob("*")
        if path.is_file()
    }
    assert actual == expected


def test_vendor_runtime_lookups_are_package_local() -> None:
    vendor = PACKAGE_ROOT / "vendor" / "ur_description"
    xacro_text = "\n".join(
        path.read_text() for path in (vendor / "urdf").rglob("*.xacro")
    )
    visual_parameters = (
        vendor / "config/ur5e/visual_parameters.yaml"
    ).read_text()
    assert "$(find ur_description)" not in xacro_text
    assert "package: ur_description" not in visual_parameters
    assert "package: robot_arm_gazebo" in visual_parameters
