from setuptools import find_packages, setup

setup(
    name='robot_motion', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_motion']),
        ('share/robot_motion', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='robot dev', maintainer_email='dev@robot.local',
    description='Supervised standard trajectory actions executed by the MCU',
    license='Apache-2.0', tests_require=['pytest'],
)
