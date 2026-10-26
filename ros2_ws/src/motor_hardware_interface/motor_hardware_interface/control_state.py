"""MCU 控制快照缓存；新协议出现后，失效不能降级到旧运行态授权。"""
import math

_LIFECYCLES = {0, 1, 2, 3, 4, 10, 13, 14, 15}


class ControlStateTracker:
    def __init__(self) -> None:
        self.seen = False
        self.valid = False
        self.at = 0.
        self.session = 0
        self.sequence = 0
        self.states = {}

    def update(self, schema: int, session: int, sequence: int,
               states: list, now: float) -> bool:
        self.seen = True
        if (type(schema) is not int or schema != 1 or type(session) is not int or session <= 0
                or type(sequence) is not int or not 0 <= sequence <= 0xFFFFFFFF
                or not math.isfinite(now) or not isinstance(states, (list, tuple))
                or len(states) > 64):
            self.valid = False
            return False
        if self.session and session < self.session:
            return False
        if self.session == session:
            advance = (sequence - self.sequence) & 0xFFFFFFFF
            if advance == 0 or advance >= 0x80000000:
                return False
        fresh = {}
        for state in states:
            if (type(state.motor_id) is not int or not 0 <= state.motor_id <= 0xFFFF
                    or state.motor_id in fresh
                    or type(state.control_lifecycle) is not int
                    or state.control_lifecycle not in _LIFECYCLES
                    or type(state.execution_state) is not int
                    or not 0 <= state.execution_state <= 7
                    or type(state.online) is not bool
                    or type(state.policy_enabled) is not bool
                    or type(state.motion_enabled) is not bool
                    or type(state.unified_fault_code) is not int
                    or not 0 <= state.unified_fault_code <= 0xFFFF
                    or type(state.state_age_ms) is not int
                    or not 0 <= state.state_age_ms <= 0xFFFFFFFF
                    or type(state.policy_generation) is not int
                    or not 0 <= state.policy_generation <= 0xFFFFFFFF
                    or type(state.vendor_fault_code) is not int
                    or not 0 <= state.vendor_fault_code <= 0xFFFFFFFF
                    or type(state.last_failure_reason) is not int
                    or not 0 <= state.last_failure_reason <= 0xFFFF):
                self.valid = False
                return False
            fresh[state.motor_id] = state
        self.session, self.sequence = session, sequence
        self.at, self.states, self.valid = now, fresh, True
        return True

    def ready(self, motor_id: int, now: float, timeout: float) -> bool:
        state = self.states.get(motor_id)
        age = now-self.at
        return bool(self.valid and state and 0 <= age <= timeout
                    and state.online and state.control_lifecycle == 3
                    and state.policy_enabled and state.unified_fault_code == 0
                    and state.state_age_ms/1000 + age <= timeout)
