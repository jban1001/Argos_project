import json
import math

import pytest

from follower_fire_control.protocol import parse_cancel, parse_dispatch


def dispatch(**updates):
    value = {
        "schema": 1,
        "mission_id": "fire-20260830-001",
        "frame_id": "map",
        "x": 1.25,
        "y": -0.4,
        "yaw": 3.5,
        "main_cleared": False,
    }
    value.update(updates)
    return json.dumps(value)


def test_valid_dispatch_is_typed_and_yaw_is_normalized():
    result = parse_dispatch(dispatch(main_cleared=True))
    assert result.mission_id == "fire-20260830-001"
    assert result.main_cleared is True
    assert -math.pi <= result.yaw <= math.pi


@pytest.mark.parametrize("payload", [
    dispatch(frame_id="odom"),
    dispatch(schema=2),
    dispatch(mission_id="bad id"),
    dispatch(main_cleared=1),
    dispatch(x=float("nan")),
])
def test_unsafe_dispatch_is_rejected(payload):
    with pytest.raises(ValueError):
        parse_dispatch(payload)


def test_unknown_or_missing_fields_are_rejected():
    data = json.loads(dispatch())
    data["typo"] = 1
    with pytest.raises(ValueError, match="unknown"):
        parse_dispatch(json.dumps(data))
    del data["typo"]
    del data["x"]
    with pytest.raises(ValueError, match="missing"):
        parse_dispatch(json.dumps(data))


def test_cancel_is_strict_and_correlated():
    assert parse_cancel('{"schema":1,"mission_id":"fire:1"}') == "fire:1"
    with pytest.raises(ValueError):
        parse_cancel('{"schema":1,"mission_id":"fire:1","all":true}')
