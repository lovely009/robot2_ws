"""Bounded local measurement; production safety deadlines remain unchanged."""
import json
from pathlib import Path
import sys
import time
import traceback
import ctypes as C
import gc
import argparse

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'ros2_ws/src/robot_bringup/test/sil'))
from managed_native_fixture import ManagedNativeFixture, wait
from action_msgs.msg import GoalStatus


def run(index,options):
    folder = ROOT/'evidence'/f'{options.prefix}-{index}'
    h = ManagedNativeFixture(folder, domain=options.domain+index)
    original_gc=gc.isenabled()
    class OutputEvent(C.Structure):
        _fields_=[('mono_ns',C.c_uint64),('tick',C.c_uint32),('motor_id',C.c_uint32)]
    h.device.lib.sil_output_events.argtypes=[C.POINTER(OutputEvent),C.c_uint]
    h.device.lib.sil_output_events.restype=C.c_uint
    def collection_phase(phase,info):
        if info['generation']==2:h.device.record('python_gc_generation2',phase=phase)
    gc.callbacks.append(collection_phase)
    outcome=dict(index=index, requested_goal_seconds=options.duration, target_rad=.05,
        fixture_gc_deferred=options.defer_fixture_gc)
    try:
        h.start()
        if options.defer_fixture_gc:
            gc.collect();gc.disable()
        h.acquire(); h.arm(); h.enable()
        h.device.record('measurement_start')
        outcome['start_ns']=time.monotonic_ns()
        goal=h.goal(.05,options.duration)
        outcome['accepted']=goal.accepted
        if goal.accepted:
            result=goal.get_result_async()
            wait(result.done,options.duration+4)
            outcome['result_status']=result.result().status
            outcome['error_code']=result.result().result.error_code
            outcome['error_string']=result.result().result.error_string
            outcome['converged']=max(abs(p-.05) for p in h.device.positions())<.01
            outcome['goal_passed']=outcome['result_status']==GoalStatus.STATUS_SUCCEEDED and outcome['converged']
        outcome['goal_end_ns']=time.monotonic_ns()
        outcome['positions']=h.device.positions()
        # Continued ownership renewal is intentional: target-source loss must
        # still remove policy/authority and disable all virtual drivers.
        h.device.record('measurement_drop_commands')
        outcome['drop_ns']=time.monotonic_ns()
        h.device.drop_commands=True
        wait(lambda:not any(h.device.states[i*10+1] for i in range(37)),2)
        wait(lambda:h.device.enabled_mask()==0,1)
        outcome['safe_disabled']=True
        outcome['disabled_ns']=time.monotonic_ns()
        time.sleep(.2)
    except Exception as error:
        outcome['exception']=f'{type(error).__name__}: {error}'
        traceback.print_exc()
    finally:
        outcome['finish_ns']=time.monotonic_ns()
        outcome['final_authorized']=bool(h.device.head[3])
        outcome['final_enabled_mask']=h.device.enabled_mask()
        events=(OutputEvent*(37*200*60))()
        with h.device._native_lock:
            count=h.device.lib.sil_output_events(events,len(events))
            outcome['output_overflow']=h.device.lib.sil_output_overflow()
        (folder/'outputs.jsonl').write_text(''.join(json.dumps(dict(mono_ns=e.mono_ns,
            tick=e.tick,motor_id=e.motor_id))+'\n' for e in events[:count]))
        h.close()
        gc.callbacks.remove(collection_phase)
        if original_gc:gc.enable()
        (folder/'outcome.json').write_text(json.dumps(outcome,indent=2))
        print(json.dumps(outcome),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=int,default=3)
    parser.add_argument('--duration',type=float,default=10)
    parser.add_argument('--prefix',default='measure')
    parser.add_argument('--domain',type=int,default=181)
    parser.add_argument('--defer-fixture-gc',action='store_true')
    options=parser.parse_args()
    for index in range(1,options.runs+1):run(index,options)
