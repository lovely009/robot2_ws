"""Validate the installed model assets without depending on robot_sim."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


SHARE = Path(get_package_share_directory('robot_description'))
SOURCE = Path(__file__).resolve().parents[1]
PREFIX = 'package://robot_description/'
# 原始 CAD 导出保留自身包名；公共运行件统一使用本包 URI。
URDFS = sorted(path.relative_to(SOURCE) for path in (SOURCE / 'models').glob('*/urdf/*.urdf')
               if path.parts[-3] not in {'Robot_body2', '全身实验版_仅双臂质量', 'robot_body'})
MODELS = sorted({path.parts[1] for path in URDFS})


@pytest.mark.parametrize('relative', URDFS, ids=str)
def test_installed_mechanical_model_and_meshes(relative):
    """A clean installation must resolve every runtime mesh and joint model."""
    installed = SHARE / relative
    assert installed.read_bytes() == (SOURCE / relative).read_bytes()
    root = ET.parse(installed).getroot()
    names = [joint.get('name') for joint in root.findall('joint')]
    assert names and len(names) == len(set(names))
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename')
        assert uri.startswith(PREFIX), uri
        asset = (SHARE / uri[len(PREFIX):]).resolve()
        assert asset.is_relative_to(SHARE.resolve())
        assert asset.is_file(), uri


def test_installed_models_have_no_simulation_entrypoints():
    assert not list((SHARE / 'models').glob('*/config/*hil*.yaml'))
    assert not list((SHARE / 'models').glob('*/config/*simulation*.yaml'))
    assert not list((SHARE / 'models').glob('*/mujoco/*.xml'))
    assert not list((SHARE / 'models').rglob('gazebo.launch'))
    assert not list((SHARE / 'models').rglob('*.urdf.xacro'))


def test_description_has_no_simulation_or_host_control_dependency():
    package = ET.parse(SHARE / 'package.xml').getroot()
    dependencies = {element.text for element in package
                    if element.tag.endswith('depend')}
    assert dependencies.isdisjoint({'robot_sim', 'web_host', 'robot_bringup',
                                   'motor_hardware_interface', 'controller_manager'})


def test_installed_driver_references_are_self_contained():
    assert ((SHARE / 'config/motor.yaml').read_bytes()
            == (SOURCE / 'config/motor.yaml').read_bytes())
    drivers = list((SHARE / 'models').glob('*/config/motor_driver.yaml'))
    assert drivers
    for driver in drivers:
        section = yaml.safe_load(driver.read_text())['motor_hardware']
        for field in ('motor_specs_file', 'transmissions_file'):
            referenced = (driver.parent / section[field]).resolve()
            assert referenced.is_relative_to(SHARE.resolve()), (driver, field)
            assert referenced.is_file(), (driver, field)
        source_driver = SOURCE / driver.relative_to(SHARE)
        assert source_driver.read_bytes() == driver.read_bytes()
