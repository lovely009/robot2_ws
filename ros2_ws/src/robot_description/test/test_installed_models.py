"""Validate the installed model assets without depending on robot_sim."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


SHARE = Path(get_package_share_directory('robot_description'))
SOURCE = Path(__file__).resolve().parents[1]
MODELS = sorted(path.name for path in (SOURCE / 'models').iterdir()
                if (path / 'urdf' / f'{path.name}.urdf').is_file())
PREFIX = 'package://robot_description/'


@pytest.mark.parametrize('model', MODELS)
def test_installed_mechanical_model_and_meshes(model):
    """A clean installation must resolve every runtime mesh and joint model."""
    relative = Path('models') / model / 'urdf' / f'{model}.urdf'
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


@pytest.mark.parametrize('model', MODELS)
def test_installed_xacro_does_not_require_sim_package(model, tmp_path):
    xacro = SHARE / 'models' / model / 'urdf' / f'{model}.urdf.xacro'
    if not xacro.is_file():
        # Some mechanical-only models have no simulation wrapper.
        assert (SHARE / 'models' / model / 'urdf' / f'{model}.urdf').is_file()
        return
    process = subprocess.run(['xacro', str(xacro), 'fixed_base:=false'],
                             cwd=tmp_path, capture_output=True, text=True)
    assert process.returncode == 0, process.stderr
    root = ET.fromstring(process.stdout)
    assert root.find("link[@name='world']") is None
    assert 'package://robot_sim/' not in process.stdout
    assert '$(find robot_sim)' not in xacro.read_text()


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
