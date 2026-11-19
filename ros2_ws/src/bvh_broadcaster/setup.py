from glob import glob
from setuptools import find_packages, setup

setup(
    name='bvh_broadcaster', version='0.2.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bvh_broadcaster']),
        ('share/bvh_broadcaster', ['package.xml', 'README.md', 'LICENSE', 'NOTICE']),
        ('share/bvh_broadcaster/launch', glob('launch/*.launch.py')),
        ('share/bvh_broadcaster/config', glob('config/*.rviz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ly', maintainer_email='ly@todo.todo', license='BSD-3-Clause',
    description='BVH TF playback and fixed-base arm IK preview for ROS 2', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'bvh_broadcaster = bvh_broadcaster.node:main',
        'retarget_bvh = bvh_broadcaster.retarget:main',
        'mocap_robot_preview = bvh_broadcaster.preview:main',
    ]},
)
