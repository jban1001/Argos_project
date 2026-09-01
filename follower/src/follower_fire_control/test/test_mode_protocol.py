import json

import pytest

from follower_fire_control.mode_protocol import (
    MODES, allows_dispatch, allows_follow, parse_mode_request,
)


def request(**updates):
    value = {"schema": 1, "request_id": "mode-1", "mode": "follow"}
    value.update(updates)
    return json.dumps(value)


def test_all_modes_parse_and_have_one_clear_policy():
    for mode in MODES:
        assert parse_mode_request(request(mode=mode)).mode == mode
    assert allows_follow("auto") and allows_dispatch("auto")
    assert allows_follow("follow") and not allows_dispatch("follow")
    assert allows_dispatch("coordinate_fire") and not allows_follow("coordinate_fire")
    assert not allows_follow("standby") and not allows_dispatch("standby")


@pytest.mark.parametrize("payload", [
    request(schema=2),
    request(request_id="bad id"),
    request(mode="manual"),
    request(extra=True),
    '{"schema":1}',
    '[]',
])
def test_invalid_or_ambiguous_requests_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_mode_request(payload)
