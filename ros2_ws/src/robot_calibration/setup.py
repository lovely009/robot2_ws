from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot dev',
    maintainer_email='dev@robot.local',
    description='Read-only startup and commissioning validation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'calibration_validator = robot_calibration.node:main',
        'calibration_check = robot_calibration.cli:main',
    ]},
)
