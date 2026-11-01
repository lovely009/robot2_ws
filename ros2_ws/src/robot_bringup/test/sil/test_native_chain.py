"""Actual host JTC + supervisor + native firmware/CAN + 37 bounded plants.

The shared fixture preserves process boundaries and wall clocks. No hardware or
ideal target-to-feedback shortcut is used. Set ROBOT2_SIL_LIBRARY explicitly.
"""
from action_msgs.msg import GoalStatus
from managed_native_fixture import ManagedNativeFixture, wait
from native_device import N


def test_supervised_37_axis_native_firmware_chain(tmp_path):
    with ManagedNativeFixture(tmp_path,domain=191) as h:
        h.acquire();h.arm();h.enable()
        feedback=[]
        goal=h.goal(.05,2,feedback=feedback.append)
        assert goal.accepted
        result=goal.get_result_async();wait(result.done,10)
        assert result.result().status==GoalStatus.STATUS_SUCCEEDED,result.result()
        assert result.result().result.error_code==0 and feedback
        assert max(abs(p-.05) for p in h.device.positions())<.01
        print('SIL: actual supervised JTC/native/CAN/37-axis feedback convergence passed',flush=True)
        # A new goal under the SAME authority must not retire JTC and interrupt
        # its periodic target stream. Cancellation preserves enabled policy.
        goal=h.goal(.15,4);assert goal.accepted
        wait(lambda:min(h.device.positions())>.055)
        canceled=goal.cancel_goal_async();wait(canceled.done)
        assert canceled.result().goals_canceling
        result=goal.get_result_async();wait(result.done)
        assert result.result().status==GoalStatus.STATUS_CANCELED,result.result()
        assert all(h.device.states[i*10+1] for i in range(N))
        assert h.latest.armed
        print('SIL: second goal, cancel and independent enabled policy passed',flush=True)
        h.device.drop_commands=True
        wait(lambda:not any(h.device.states[i*10+1] for i in range(N)),2)
        assert not h.device.head[3]
        wait(lambda:h.device.enabled_mask()==0,1)
        print('SIL: target-source loss revokes despite continued ownership renewal passed',flush=True)
