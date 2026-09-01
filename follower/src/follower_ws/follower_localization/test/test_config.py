"""측정되지 않은 값으로는 기동하지 않는다."""

import pytest

from follower_localization.config import (
    ArucoThresholds,
    UnmeasuredValue,
    parse_aruco_thresholds,
    require_measured,
)

GOOD = {
    "max_reprojection_px": 1.5,
    "max_distance_m": 2.0,
    "min_distance_m": 0.3,
    "position_variance_m2": 0.0004,
}


def test_measured_values_parse():
    thresholds = parse_aruco_thresholds(GOOD)
    assert isinstance(thresholds, ArucoThresholds)
    assert thresholds.max_distance_m == 2.0


def test_placeholder_is_refused():
    values = dict(GOOD, max_reprojection_px="<CONFIGURE>")
    with pytest.raises(UnmeasuredValue) as caught:
        parse_aruco_thresholds(values)
    # 실패 메시지는 무엇을 어떻게 재야 하는지 알려줘야 한다
    assert "26_characterize_aruco" in str(caught.value)


def test_every_threshold_is_checked():
    """한 항목만 검사하고 나머지를 흘려보내면 안 된다."""
    for name in GOOD:
        with pytest.raises(UnmeasuredValue):
            parse_aruco_thresholds(dict(GOOD, **{name: "<CONFIGURE>"}))


def test_numeric_strings_are_accepted():
    """YAML 이 문자열로 넘겨도 숫자면 받는다."""
    thresholds = parse_aruco_thresholds(dict(GOOD, max_distance_m="2.5"))
    assert thresholds.max_distance_m == 2.5


def test_nonsense_string_is_refused():
    with pytest.raises(UnmeasuredValue):
        parse_aruco_thresholds(dict(GOOD, max_distance_m="곧 잴 것"))


def test_missing_field_is_refused():
    values = dict(GOOD)
    del values["min_distance_m"]
    with pytest.raises(UnmeasuredValue):
        parse_aruco_thresholds(values)


def test_inverted_distance_range_is_refused():
    with pytest.raises(UnmeasuredValue):
        parse_aruco_thresholds(dict(GOOD, min_distance_m=3.0, max_distance_m=2.0))


def test_nonpositive_values_are_refused():
    with pytest.raises(UnmeasuredValue):
        parse_aruco_thresholds(dict(GOOD, position_variance_m2=0.0))


def test_empty_string_is_treated_as_unmeasured():
    with pytest.raises(UnmeasuredValue):
        require_measured("x", "   ")
