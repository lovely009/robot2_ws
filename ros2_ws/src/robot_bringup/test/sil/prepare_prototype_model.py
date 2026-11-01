"""Prepare local-only half-body assets without changing the source robot model."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def prepare(root, output):
    source=root/'ros2_ws/src/robot_description/models/half_body_robot'
    model=output/'models/half_body_robot'
    (model/'config').mkdir(parents=True,exist_ok=True)
    (model/'meshes').mkdir(exist_ok=True)
    if not (model/'urdf').exists():
        (model/'urdf').symlink_to(source/'urdf',target_is_directory=True)
    driver=yaml.safe_load((source/'config/motor_driver.yaml').read_text())
    hardware=driver['motor_hardware']
    hardware['motor_specs_file']=str(root/'ros2_ws/src/robot_description/config/motor.yaml')
    hardware['transmissions_file']=str(source/'config/transmissions.yaml')
    hardware['buses']={'arm':{'transport':'socketcan','interface':'sil_never_open_arm'}}
    (model/'config/motor_driver.yaml').write_text(yaml.safe_dump(driver))
    spec=importlib.util.spec_from_file_location('simplify',source/'tools/simplify_mesh.py')
    api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)
    names={Path(n.get('filename')).name for n in ET.parse(source/'urdf/half_body_robot.urdf').findall('.//visual/geometry/mesh')}
    for name in sorted(names):
        target=model/'meshes'/name
        if target.exists():continue
        triangles=api.read_binary_stl(source/'meshes'/name)
        reduced=api.decimate(triangles,.004) if len(triangles)>10000 else triangles
        assert len(reduced)>0
        api.write_binary_stl(target,reduced)
    return output


def native_library(root,folder):
    import os
    import subprocess
    library=os.environ.get('ROBOT2_SIL_LIBRARY')
    if library:return Path(library)
    build=folder/'native-build'
    subprocess.run(['cmake','-S',str(root/'micro_ros_firmware/test/host'),'-B',str(build)],check=True)
    subprocess.run(['cmake','--build',str(build),'--target','robot2_sil','-j4'],check=True)
    return build/'librobot2_sil.so'
