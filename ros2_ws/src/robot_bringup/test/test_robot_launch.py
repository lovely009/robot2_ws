"""工作台入口不引入独立托管节点，模型由会话管理器选择。"""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def load(name):
    path=Path(__file__).parents[1]/'launch'/name
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.generate_launch_description().entities


def test_default_robot_starts_bridge_and_workbench_without_required_model():
    actions=load('robot.launch.py')
    nodes=[a for a in actions if isinstance(a,Node)]
    assert [n.node_executable for n in nodes]==['micro_ros_bridge_node']
    includes=[a for a in actions if isinstance(a,IncludeLaunchDescription)]
    assert len(includes)==1
    includes[0].launch_description_source.get_launch_description(LaunchContext())
    assert 'bench.launch.py' in includes[0].launch_description_source.location
    assert not any(isinstance(a,DeclareLaunchArgument) and a.default_value is None for a in actions)


def test_bench_owns_sessions_and_exposes_normal_workbench():
    actions=load('bench.launch.py')
    assert {n.node_package for n in actions if isinstance(n,Node)}=={'robot_bringup'}
    includes=[a for a in actions if isinstance(a,IncludeLaunchDescription)]
    web=next(a for a in includes if 'control_mode' in dict(a.launch_arguments))
    assert dict(web.launch_arguments)['control_mode']=='bench'
