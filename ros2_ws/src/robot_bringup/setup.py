from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ly',
    maintainer_email='ly@todo.todo',
    description='台架启动入口与会话编排管理器。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['bench_manager = bench_manager.node:main'],
    },
)
