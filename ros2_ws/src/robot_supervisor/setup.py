from setuptools import find_packages, setup

setup(
    name='robot_supervisor', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_supervisor']),
        ('share/robot_supervisor', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='robot dev', maintainer_email='dev@robot.local',
    description='Robot readiness, exclusive control leases and managed ingress',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'robot_supervisor = robot_supervisor.node:main',
        'robot_control = robot_supervisor.client:main',
    ]},
)
