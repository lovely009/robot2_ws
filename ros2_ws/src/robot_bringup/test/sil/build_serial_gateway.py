"""Build the serial client from existing PlatformIO sources; no downloads."""
import subprocess


def build(root,folder):
    cache=root/'micro_ros_firmware/.pio/libdeps/stm32h723vgt6/micro_ros_platformio/build/mcu/src'
    prefix=folder/'prefix'
    for source,target,options in [
        (cache/'micro-CDR',folder/'microcdr-build',['-DUCDR_BUILD_TESTS=OFF','-DUCDR_BUILD_EXAMPLES=OFF']),
        (cache/'Micro-XRCE-DDS-Client',folder/'client-build',[
            '-DUCLIENT_SUPERBUILD=OFF','-DUCLIENT_BUILD_TESTS=OFF','-DUCLIENT_BUILD_EXAMPLES=OFF',
            '-DUCLIENT_PROFILE_UDP=OFF','-DUCLIENT_PROFILE_TCP=OFF','-DUCLIENT_PROFILE_DISCOVERY=OFF'])]:
        if not source.is_dir():raise RuntimeError(f'Existing micro-ROS source cache required: {source}')
        subprocess.run(['cmake','-S',str(source),'-B',str(target),f'-DCMAKE_PREFIX_PATH={prefix}',
            f'-DCMAKE_INSTALL_PREFIX={prefix}',*options],check=True)
        subprocess.run(['cmake','--build',str(target),'-j4'],check=True)
        subprocess.run(['cmake','--install',str(target)],check=True)
    target=folder/'gateway-build'
    subprocess.run(['cmake','-S',str(root/'ros2_ws/src/robot_bringup/test/sil/serial'),'-B',str(target),
        f'-DCMAKE_PREFIX_PATH={prefix}'],check=True)
    subprocess.run(['cmake','--build',str(target),'-j4'],check=True)
