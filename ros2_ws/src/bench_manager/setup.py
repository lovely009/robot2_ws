from setuptools import find_packages, setup

package_name = 'bench_manager'

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
    description='Bench host manager: simulation / micro-ROS Agent / motor '
                'backend as ROS services.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bench_manager = bench_manager.node:main',
        ],
    },
)
