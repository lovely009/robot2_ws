"""Bounded MCU authorization; all deadlines use the MCU's monotonic clock.

The wire identifier is a session-local generation, not the client's secret
token. Its partition also fences LowCommand without enlarging that MTU-bound
message. Acknowledgements never extend the host client's own lease.
"""
from __future__ import annotations

import math
import time


class McuLease:
    PARTITION = 1_000_000
    MAX_U64 = (1 << 64) - 1
    MAX_U32 = (1 << 32) - 1

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.session = 0
        self.lease_id = 0
        self.sequence = 0
        self.active = False
        self.confirmed_once = False
        self.started_at = 0.
        self.report = None
        self.report_at = -math.inf

    def observe(self, report):
        for field, maximum in (('session_id', self.MAX_U64),
                               ('lease_id', self.MAX_U64),
                               ('lease_sequence', self.MAX_U32),
                               ('lease_valid_until_tick_ms', self.MAX_U32),
                               ('tick_ms', self.MAX_U32)):
            value = getattr(report, field, None)
            if type(value) is not int or not 0 <= value <= maximum:
                raise ValueError('invalid MCU lease acknowledgement: ' + field)
        if type(report.lease_active) is not bool or not report.session_id:
            raise ValueError('invalid MCU lease acknowledgement')
        if report.lease_id and report.lease_id // self.PARTITION != report.session_id:
            raise ValueError('MCU lease acknowledgement belongs to another session')
        if report.session_id != self.session:
            self.session = report.session_id
            self.lease_id = self.sequence = 0
            self.active = self.confirmed_once = False
        self.report, self.report_at = report, self.clock()
        if self.confirmed:
            self.confirmed_once = True

    def start(self, session):
        if session != self.session or self.clock() - self.report_at > .25:
            raise ValueError('fresh MCU authorization state required before arming')
        base = session * self.PARTITION
        candidate = max(base + 1, self.report.lease_id + 1, self.lease_id + 1)
        if candidate > self.MAX_U64 or candidate >= base + self.PARTITION:
            raise ValueError('MCU lease generations exhausted; new MCU session required')
        self.lease_id, self.sequence = candidate, 0
        self.active, self.confirmed_once = True, False
        self.started_at = self.clock()

    def _tick(self):
        return (self.report.tick_ms + int((self.clock() - self.report_at) * 1000)) & self.MAX_U32

    @property
    def confirmed(self):
        if (not self.active or self.report is None
                or not 0 <= self.clock() - self.report_at <= .25
                or not self.report.lease_active or self.report.lease_id != self.lease_id
                or not 0 < self.report.lease_sequence <= self.sequence):
            return False
        remaining = (self.report.lease_valid_until_tick_ms - self._tick()) & self.MAX_U32
        return 0 < remaining <= 300

    @property
    def failed(self):
        return self.active and not self.confirmed and (
            self.confirmed_once or self.clock() - self.started_at > .4)

    def packet(self, remaining_ms, *, revoke=False):
        if not self.lease_id or not self.session:
            return None
        if self.sequence >= self.MAX_U32:
            raise ValueError('MCU lease sequence exhausted')
        if revoke:
            self.active = False
        elif (not self.active or self.report is None
              or not 0 <= self.clock() - self.report_at <= .25 or remaining_ms <= 0):
            return None
        self.sequence += 1
        # Floor the remaining client duration, so the MCU deadline is never
        # extended by ceil rounding of the public RobotState field.
        duration = max(0, min(200, int(remaining_ms) - 1))
        return dict(session_id=self.session, lease_id=self.lease_id,
                    sequence=self.sequence,
                    valid_until_tick_ms=0 if revoke else (self._tick() + duration) & self.MAX_U32,
                    active=not revoke and duration > 0)
