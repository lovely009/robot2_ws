"""控制快照不能被旧运行态、本地使能请求或过期数据覆盖。"""
from types import SimpleNamespace
import pytest
from motor_hardware_interface.control_state import ControlStateTracker


def state(**changes: object) -> SimpleNamespace:
    values = dict(motor_id=1, online=True, control_lifecycle=3, execution_state=1,
                  policy_enabled=True, motion_enabled=False, state_age_ms=10,
                  unified_fault_code=0, vendor_fault_code=0, last_failure_reason=0,
                  policy_generation=1)
    return SimpleNamespace(**(values | changes))


def test_active_permission_accepts_first_target_without_motion() -> None:
    tracker = ControlStateTracker()
    assert tracker.update(1, 100, 1, [state()], 1.)
    assert tracker.ready(1, 1.1, .5)
    assert not tracker.ready(1, 2., .5)
    assert not tracker.ready(2, 1.1, .5)


def test_complete_snapshot_missing_member_revokes_old_permission() -> None:
    tracker = ControlStateTracker()
    tracker.update(1, 100, 1, [state()], 1.)
    assert tracker.update(1, 100, 2, [], 1.1)
    assert not tracker.ready(1, 1.2, .5)


def test_replay_and_old_session_cannot_refresh_authorization() -> None:
    tracker = ControlStateTracker()
    tracker.update(1, 100, 3, [state()], 1.)
    assert not tracker.update(1, 100, 2, [state()], 1.1)
    assert tracker.update(1, 101, 1, [state(policy_enabled=False)], 1.2)
    assert not tracker.update(1, 100, 4, [state()], 1.3)
    assert not tracker.ready(1, 1.4, .5)


@pytest.mark.parametrize('changes', [dict(online=False), dict(control_lifecycle=2),
    dict(policy_enabled=False), dict(unified_fault_code=2), dict(state_age_ms=600)])
def test_unavailable_motor_cannot_be_authorized_by_snapshot(changes: dict) -> None:
    tracker = ControlStateTracker()
    tracker.update(1, 100, 1, [state(**changes)], 1.)
    assert not tracker.ready(1, 1.1, .5)


def test_invalid_schema_fails_closed_without_legacy_fallback() -> None:
    tracker = ControlStateTracker()
    tracker.update(1, 100, 1, [state()], 1.)
    assert not tracker.update(2, 100, 2, [state()], 1.1)
    assert tracker.seen
    assert not tracker.ready(1, 1.2, .5)


def test_snapshot_capacity_does_not_limit_motor_identity_to_array_index() -> None:
    tracker = ControlStateTracker()
    assert tracker.update(1, 100, 1, [state(motor_id=120)], 1.)
    assert tracker.ready(120, 1.1, .5)
