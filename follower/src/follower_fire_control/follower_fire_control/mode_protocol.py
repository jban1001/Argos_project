"""Strict, dependency-free protocol for selecting the follower mission mode."""

from __future__ import annotations

from dataclasses import dataclass
import json
import re


MODES = ("auto", "follow", "coordinate_fire", "standby")
_REQUEST_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")
_FIELDS = {"schema", "request_id", "mode"}


@dataclass(frozen=True)
class ModeRequest:
    request_id: str
    mode: str


def parse_mode_request(payload: str) -> ModeRequest:
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("message must be a JSON object")
    unknown = set(data) - _FIELDS
    missing = _FIELDS - set(data)
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing fields: {sorted(missing)}")
    if data["schema"] != 1:
        raise ValueError("schema must be 1")
    request_id = data["request_id"]
    if not isinstance(request_id, str) or not _REQUEST_ID.fullmatch(request_id):
        raise ValueError(
            "request_id must match [A-Za-z0-9][A-Za-z0-9_.:-]{0,63}")
    mode = data["mode"]
    if mode not in MODES:
        raise ValueError(f"mode must be one of {', '.join(MODES)}")
    return ModeRequest(request_id=request_id, mode=mode)


def allows_follow(mode: str) -> bool:
    return mode in ("auto", "follow")


def allows_dispatch(mode: str) -> bool:
    return mode in ("auto", "coordinate_fire")
