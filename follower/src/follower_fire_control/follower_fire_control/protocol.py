"""Strict, dependency-free wire protocol for main-to-follower fire dispatch."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re


_MISSION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")
_DISPATCH_FIELDS = {
    "schema", "mission_id", "frame_id", "x", "y", "yaw", "main_cleared"
}
_CANCEL_FIELDS = {"schema", "mission_id"}


@dataclass(frozen=True)
class FireDispatch:
    mission_id: str
    x: float
    y: float
    yaw: float
    main_cleared: bool
    frame_id: str = "map"


def _object(payload: str) -> dict:
    try:
        value = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("message must be a JSON object")
    return value


def _mission_id(value: object) -> str:
    if not isinstance(value, str) or not _MISSION_ID.fullmatch(value):
        raise ValueError("mission_id must match [A-Za-z0-9][A-Za-z0-9_.:-]{0,63}")
    return value


def parse_dispatch(payload: str) -> FireDispatch:
    data = _object(payload)
    unknown = set(data) - _DISPATCH_FIELDS
    missing = _DISPATCH_FIELDS - set(data)
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing fields: {sorted(missing)}")
    if data["schema"] != 1:
        raise ValueError("schema must be 1")
    if data["frame_id"] != "map":
        raise ValueError("frame_id must be map")
    if not isinstance(data["main_cleared"], bool):
        raise ValueError("main_cleared must be a JSON boolean")
    try:
        x, y, yaw = (float(data[name]) for name in ("x", "y", "yaw"))
    except (TypeError, ValueError) as exc:
        raise ValueError("x, y and yaw must be numbers") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("x, y and yaw must be finite")
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return FireDispatch(
        mission_id=_mission_id(data["mission_id"]),
        x=x,
        y=y,
        yaw=yaw,
        main_cleared=data["main_cleared"],
    )


def parse_cancel(payload: str) -> str:
    data = _object(payload)
    unknown = set(data) - _CANCEL_FIELDS
    missing = _CANCEL_FIELDS - set(data)
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing fields: {sorted(missing)}")
    if data["schema"] != 1:
        raise ValueError("schema must be 1")
    return _mission_id(data["mission_id"])
