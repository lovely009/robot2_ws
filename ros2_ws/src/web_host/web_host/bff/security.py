"""Unified same-origin and request payload validation for HTTP and WebSocket."""
from __future__ import annotations

from urllib.parse import urlsplit

from fastapi import HTTPException, Request


def same_origin(origin: str, host: str) -> bool:
    """Compare Origin header strictly against Host; default port 80 is equivalent."""
    if not origin or not host:
        return False
    try:
        source = urlsplit(origin)
        target = urlsplit('http://' + host)
        return (
            source.scheme == target.scheme == 'http'
            and source.hostname is not None
            and source.hostname == target.hostname
            and (source.port if source.port is not None else 80)
            == (target.port if target.port is not None else 80)
            and source.username is None
            and source.password is None
            and target.username is None
            and target.password is None
            and not source.path
            and not source.query
            and not source.fragment
            and not target.path
            and not target.query
            and not target.fragment
        )
    except (TypeError, ValueError):
        return False


def enforce_same_origin(
    request: Request,
    *,
    require_json: bool = True,
    max_bytes: int | None = None,
) -> None:
    """Reject cross-origin simple POSTs, non-JSON payloads, and oversized bodies."""
    origin = request.headers.get('origin')
    if origin is not None and not same_origin(origin, request.headers.get('host', '')):
        raise HTTPException(status_code=403, detail={'error': '只接受同源请求'})
    if require_json:
        media_type = request.headers.get('content-type', '').split(';', 1)[0].strip().lower()
        if media_type != 'application/json':
            raise HTTPException(status_code=415, detail={'error': '请求必须为 application/json'})
    if max_bytes is not None and 'content-length' in request.headers:
        try:
            length = int(request.headers['content-length'])
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid Content-Length'}) from exc
        if length < 0 or length > max_bytes:
            raise HTTPException(status_code=413, detail={'error': 'request body too large'})
