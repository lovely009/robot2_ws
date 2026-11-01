"""Causal scheduling regression, separate from full native MCU acceptance.

The validator is deliberately held by a barrier. The real single ROS executor
must continue its periodic callbacks, reject another goal and fence a revoked
authorization before the validation result is accepted. No hardware interface.
"""
import threading
import time
import uuid
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from robot_motion.host_action import HostMotionActionAdapter


def wait(condition):
    until=time.monotonic()+2
    while time.monotonic()<until:
        if condition(): return
        time.sleep(.002)
    assert condition()


@pytest.mark.parametrize('revoke',[False,True])
def test_validation_yields_executor_and_rechecks_authority(tmp_path,monkeypatch,revoke):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    driver=tmp_path/'driver.yaml';urdf=tmp_path/'fixture.urdf'
    driver.write_text('motor_hardware:\n  motors:\n    axis: {id: 1, joint: j1}\n')
    urdf.write_text('<robot name="fixture"><link name="base"/><link name="tip"/>'
        '<joint name="j1" type="revolute"><parent link="base"/><child link="tip"/>'
        '<limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>')
    context=Context();rclpy.init(args=[],context=context,domain_id=195)
    node=Node('host_validation_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    permission={'valid':True};stops=[];ticks=[]
    adapter=HostMotionActionAdapter(node,str(driver),str(urdf),
        lambda _:permission['valid'],lambda *args:stops.append(args))
    adapter._controllers_ready=True
    monkeypatch.setattr(adapter._client,'server_is_ready',lambda:True)
    began=threading.Event();release=threading.Event()
    def validate(goal):
        began.set()
        assert release.wait(timeout=2)
        return goal
    monkeypatch.setattr(adapter,'_compile',validate)
    node.create_timer(.01,lambda:ticks.append(time.monotonic()))
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        adapter.authorize_goal(uuid.uuid4().bytes,{},expires_in_s=1.)
        future=executor.create_task(adapter._goal,FollowJointTrajectory.Goal())
        wait(began.is_set)
        before=len(ticks)
        wait(lambda:len(ticks)>=before+3)
        assert not future.done(), 'periodic callbacks must run before validation can finish'
        duplicate=executor.create_task(adapter._goal,FollowJointTrajectory.Goal())
        wait(duplicate.done);assert duplicate.result()==GoalResponse.REJECT
        if revoke:permission['valid']=False
        release.set();wait(future.done)
        assert future.result()==(GoalResponse.REJECT if revoke else GoalResponse.ACCEPT)
        assert not stops and adapter._active is None and adapter.context is None
    finally:
        release.set();adapter.shutdown();executor.shutdown();thread.join(timeout=2)
        node.destroy_node();context.shutdown()


@pytest.mark.parametrize('same_owner',[True,False])
def test_terminal_hold_continuation_restarts_only_for_a_new_owner(same_owner):
    adapter=object.__new__(HostMotionActionAdapter)
    adapter._valid=lambda _:True
    dispatched=[];switches=[];failed=[]
    adapter._dispatch=dispatched.append
    adapter._finish=lambda *args:failed.append(args)
    adapter._switch=SimpleNamespace(call_async=lambda request:(switches.append(request)
        or SimpleNamespace(add_done_callback=lambda _:None)))
    active={'context':{},'cancel':False,'continuing':same_owner}
    adapter._active=active
    reply=SimpleNamespace(result=lambda:SimpleNamespace(controller=[
        SimpleNamespace(name='trajectory_controller',state='active'),
        SimpleNamespace(name='gravity_controller',state='inactive')]))
    adapter._prepare(active,reply)
    assert not failed
    if same_owner:
        assert dispatched==[active] and not switches
    else:
        assert not dispatched and len(switches)==1
        assert list(switches[0].deactivate_controllers)==['trajectory_controller']


def test_cancel_during_controller_prepare_does_not_dispatch_or_disable():
    adapter=object.__new__(HostMotionActionAdapter)
    adapter._valid=lambda _:True
    finished=[]
    adapter._finish=lambda *args:finished.append(args)
    active={'context':{},'cancel':True};adapter._active=active
    adapter._prepare(active,SimpleNamespace(result=lambda:None))
    assert len(finished)==1 and finished[0][1:3]==('cancel',0)
