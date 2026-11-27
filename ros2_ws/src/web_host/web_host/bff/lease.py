"""Thread-safe single-controller lease manager (0 or 1 active controller)."""
from __future__ import annotations

import hmac
import secrets
import threading
import time
from typing import Callable

from .schemas import LeaseState


class LeaseConflictError(Exception):
    """Raised when another client holds the lease and force=False."""

    def __init__(self, current: LeaseState) -> None:
        super().__init__(f'控制权当前由 {current.owner_label or current.owner_id} 持有')
        self.current = current


class LeaseAuthError(Exception):
    """Raised when the configured control PIN does not match."""


class ControlLeaseManager:
    """Enforces 1-controller / N-viewer exclusivity with TTL and forced preemption."""

    def __init__(
        self,
        *,
        ttl_s: float = 3.0,
        pin: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        wall_ms: Callable[[], int] = lambda: int(time.time() * 1000),
        on_active_count_changed: Callable[[int], None] | None = None,
        on_stop_requested: Callable[[str], None] | None = None,
        on_state_changed: Callable[[LeaseState], None] | None = None,
    ) -> None:
        self._ttl_s = max(0.2, float(ttl_s))
        self._pin = pin if pin else None
        self._clock = clock
        self._wall_ms = wall_ms
        self._on_active_count_changed = on_active_count_changed
        self._on_stop_requested = on_stop_requested
        self._on_state_changed = on_state_changed

        self._lock = threading.RLock()
        self._owner_id: str | None = None
        self._owner_label: str | None = None
        self._token: str | None = None
        self._acquired_at_ms: int | None = None
        self._expires_mono: float | None = None
        self._expires_at_ms: int | None = None
        self._connected_clients = 0
        self._epoch = 0

    def _snapshot_locked(self) -> LeaseState:
        return LeaseState(
            owner_id=self._owner_id,
            owner_label=self._owner_label,
            acquired_at_ms=self._acquired_at_ms,
            expires_at_ms=self._expires_at_ms,
            active_controllers=1 if self._owner_id is not None else 0,
            connected_clients=self._connected_clients,
            pin_required=self._pin is not None,
            epoch=self._epoch,
        )

    def _emit(
        self,
        *,
        stop_reason: str | None = None,
        active_count: int | None = None,
        state: LeaseState | None = None,
    ) -> None:
        if stop_reason is not None and self._on_stop_requested is not None:
            self._on_stop_requested(stop_reason)
        if active_count is not None and self._on_active_count_changed is not None:
            self._on_active_count_changed(active_count)
        if state is not None and self._on_state_changed is not None:
            self._on_state_changed(state)

    def snapshot(self) -> LeaseState:
        with self._lock:
            return self._snapshot_locked()

    def is_owner(self, client_id: str | None, token: str | None) -> bool:
        expired_state: LeaseState | None = None
        with self._lock:
            if (
                self._owner_id is not None
                and self._expires_mono is not None
                and self._clock() >= self._expires_mono
            ):
                self._clear_locked()
                expired_state = self._snapshot_locked()
            else:
                return (
                    bool(client_id)
                    and bool(token)
                    and self._owner_id == client_id
                    and self._token is not None
                    and hmac.compare_digest(token, self._token)
                )
        if expired_state is not None:
            self._emit(stop_reason='lease_timeout', active_count=0, state=expired_state)
        return False

    def set_connected_clients(self, count: int) -> LeaseState:
        with self._lock:
            self._connected_clients = max(0, int(count))
            state = self._snapshot_locked()
        self._emit(state=state)
        return state

    def acquire(
        self,
        client_id: str,
        label: str = '',
        *,
        force: bool = False,
        pin: str | None = None,
    ) -> tuple[LeaseState, str, bool]:
        if self._pin is not None and not hmac.compare_digest(pin or '', self._pin):
            raise LeaseAuthError('控制口令不正确')

        stop_reason: str | None = None
        active_count: int | None = None
        preempted_previous = False

        with self._lock:
            now_mono = self._clock()
            now_wall = self._wall_ms()
            if self._owner_id is not None and self._expires_mono is not None and now_mono >= self._expires_mono:
                self._clear_locked()
                stop_reason = 'lease_timeout'
                active_count = 0

            if self._owner_id is not None and self._owner_id != client_id:
                if not force:
                    raise LeaseConflictError(self._snapshot_locked())
                preempted_previous = True
                stop_reason = 'lease_preempted'

            was_idle = self._owner_id is None
            self._owner_id = client_id
            self._owner_label = label.strip() or client_id
            self._token = secrets.token_urlsafe(24)
            self._acquired_at_ms = now_wall
            self._expires_mono = now_mono + self._ttl_s
            self._expires_at_ms = now_wall + int(self._ttl_s * 1000)
            self._epoch += 1
            token = self._token
            state = self._snapshot_locked()
            if was_idle or active_count == 0:
                active_count = 1

        self._emit(stop_reason=stop_reason, active_count=active_count, state=state)
        return state, token, preempted_previous

    def heartbeat(self, client_id: str, token: str | None) -> bool:
        with self._lock:
            if (
                self._owner_id != client_id
                or not self._token
                or not token
                or not hmac.compare_digest(token, self._token)
            ):
                return False
            now_mono = self._clock()
            now_wall = self._wall_ms()
            self._expires_mono = now_mono + self._ttl_s
            self._expires_at_ms = now_wall + int(self._ttl_s * 1000)
            return True

    def release(self, client_id: str, token: str | None = None) -> LeaseState:
        active_count: int | None = None
        changed = False
        with self._lock:
            if self._owner_id == client_id and (
                token is None or (self._token is not None and hmac.compare_digest(token, self._token))
            ):
                self._clear_locked()
                active_count = 0
                changed = True
            state = self._snapshot_locked()
        if changed:
            self._emit(active_count=active_count, state=state)
        return state

    def on_client_disconnected(self, client_id: str) -> bool:
        with self._lock:
            if self._owner_id != client_id:
                return False
            self._clear_locked()
            state = self._snapshot_locked()
        self._emit(active_count=0, state=state)
        return True

    def expire_if_needed(self) -> bool:
        with self._lock:
            if self._owner_id is None or self._expires_mono is None:
                return False
            if self._clock() < self._expires_mono:
                return False
            self._clear_locked()
            state = self._snapshot_locked()
        self._emit(stop_reason='lease_timeout', active_count=0, state=state)
        return True

    def request_stop(self, reason: str) -> None:
        if self._on_stop_requested is not None:
            self._on_stop_requested(reason)

    def _clear_locked(self) -> None:
        self._owner_id = None
        self._owner_label = None
        self._token = None
        self._acquired_at_ms = None
        self._expires_mono = None
        self._expires_at_ms = None
        self._epoch += 1
