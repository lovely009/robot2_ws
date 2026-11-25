"""Shared transmission status payload helpers for workbench_control and firmware_topic_bridge."""

from __future__ import annotations

import json
import math


def format_joint_limits(limits: dict[str, tuple[float, float]]) -> dict[str, dict[str, float | None]]:
    """Format finite/infinite joint bounds for /robot_transmission/status JSON."""
    return {
        name: {
            'lower': lower if math.isfinite(lower) else None,
            'upper': upper if math.isfinite(upper) else None,
        }
        for name, (lower, upper) in limits.items()
    }


def build_transmission_status(
    *,
    model: str,
    driver_file: str,
    trajectory_enabled: bool,
    control_state_source: str,
    control_service: str,
    joint_names: list[str],
    joint_limits: dict[str, tuple[float, float]],
    last_error: str = '',
    extra: dict | None = None,
) -> str:
    """Serialize /robot_transmission/status payload consistently across feedback nodes."""
    payload = {
        'model': model,
        'driver_file': driver_file,
        'trajectory_enabled': trajectory_enabled,
        'control_state_source': control_state_source,
        'control_service': control_service,
        'last_error': last_error,
        'joint_names': list(joint_names),
        'joint_limits': format_joint_limits(joint_limits),
    }
    if extra:
        payload.update(extra)
    return json.dumps(payload, ensure_ascii=False)
