import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'web_host'


def _web_data_files():
    """把 web/ 下的静态文件按目录逐层列成 data_files 条目。

    data_files 的每个条目要求是**文件**列表：塞目录进去（vendor/ 是三层嵌套）
    setuptools 会报 "doesn't exist or not a regular file"。

    followlinks=True 是必需的：colcon 在 build/web_host/ 里跑本文件，那里的
    web/ 是逐项指回 src 的符号链接，os.walk 默认不下钻目录符号链接，vendor/
    会被整个跳过（构建"成功"但页面 404，踩过）。代价是 web/ 里不能再放目录
    符号链接——臂视图的几何因此走 static_server 的挂载，而不是符号链接。
    """
    out = []
    for root, _dirs, files in os.walk('web', followlinks=True):
        if not files:
            continue
        out.append((os.path.join('share', package_name, root),
                    [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *_web_data_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ly',
    maintainer_email='ly@todo.todo',
    description='Humanoid robot web station: rosbridge websocket + static web UI.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_server = web_host.static_server:main',
            'rosbridge_shim = web_host.rosbridge_shim:main',
        ],
    },
)
