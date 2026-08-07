import os
from glob import glob

from setuptools import setup

package_name = 'can_motor_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # ament index resource + manifest + launch files + helper scripts
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='love009',
    maintainer_email='love009@example.com',
    description='CAN FD motor simulator (Protocol A/B) and interactive debug '
                'master for vcan-based motor drive debugging with ros2_socketcan',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_slave = can_motor_sim.motor_slave:main',
            'motor_master = can_motor_sim.motor_master:main',
        ],
    },
)
