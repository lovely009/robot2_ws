from setuptools import find_packages, setup

package_name = 'motor_command'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ly',
    maintainer_email='ly@todo.todo',
    description='Host-side command controller: holds the bench setpoint and '
                'emits the command snapshot at a fixed rate.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_streamer = motor_command.command_streamer:main',
        ],
    },
)
