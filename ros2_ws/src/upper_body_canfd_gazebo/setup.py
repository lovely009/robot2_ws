from glob import glob
import os

from setuptools import find_packages, setup


package_name = "upper_body_canfd_gazebo"


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    maintainer="Robot Developer",
    maintainer_email="robot@example.com",
    description="14-DOF ROS 2 Humble + Gazebo Fortress upper-body model and ROS action tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ros_actions = upper_body_canfd_gazebo.ros_action_player:main",
        ],
    },
)
