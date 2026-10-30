from types import SimpleNamespace

import pytest

from robot_supervisor.mcu_lease import McuLease


SESSION = 1_800_000_000_000


def fixture(tick=1000):
    now = [10.]
    lease = McuLease(clock=lambda: now[0])
    report = SimpleNamespace(session_id=SESSION, lease_id=0, lease_sequence=0,
                             tick_ms=tick, lease_valid_until_tick_ms=0, lease_active=False)
    lease.observe(report)
    return lease, now, report


def test_new_authorization_waits_for_ack_and_never_extends_client_deadline():
    lease, now, report = fixture()
    lease.start(SESSION)
    packet = lease.packet(80)
    assert packet['lease_id'] == SESSION * 1_000_000 + 1
    assert packet['valid_until_tick_ms'] == 1079
    assert not lease.confirmed
    report.lease_id = packet['lease_id']
    report.lease_sequence = packet['sequence']
    report.lease_valid_until_tick_ms = packet['valid_until_tick_ms']
    report.lease_active = True
    lease.observe(report)
    assert lease.confirmed
    now[0] += .080
    assert lease.failed and not lease.confirmed


def test_rearm_never_reuses_an_expired_generation():
    lease, _, report = fixture()
    lease.start(SESSION)
    first = lease.lease_id
    packet = lease.packet(0, revoke=True)
    assert not packet['active']
    lease.start(SESSION)
    assert lease.lease_id == first + 1
    report.lease_id = lease.lease_id + 7  # another supervisor's high water
    lease.observe(report)
    lease.start(SESSION)
    assert lease.lease_id == first + 9


def test_stale_control_reports_cannot_keep_issuing_fresh_deadlines():
    lease, now, _ = fixture()
    lease.start(SESSION)
    now[0] += .26
    assert lease.packet(1000) is None
    now[0] += .15
    assert lease.failed


def test_mcu_tick_wrap_is_handled_without_ros_clock():
    lease, now, _ = fixture(0xFFFFFFF0)
    lease.start(SESSION)
    now[0] += .020
    packet = lease.packet(1000)
    assert 202 <= packet['valid_until_tick_ms'] <= 204


def test_partition_exhaustion_and_cross_session_ack_fail_closed():
    lease, _, report = fixture()
    report.lease_id = SESSION * 1_000_000 + 999999
    lease.observe(report)
    with pytest.raises(ValueError, match='exhausted'):
        lease.start(SESSION)
    report.lease_id += 1
    with pytest.raises(ValueError, match='another session'):
        lease.observe(report)
