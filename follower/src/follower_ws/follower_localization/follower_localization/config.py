"""설정값을 읽되, 아직 측정되지 않은 값으로는 기동하지 않는다.

<CONFIGURE> 는 "이 값은 아직 재지 않았다" 는 표시다. 그런 값을 기본값으로
슬쩍 채우고 돌리면 시스템은 돌아가는 것처럼 보이지만 결과가 조용히 틀리고,
나중에 그 원인을 역추적하는 비용이 훨씬 크다. 그래서 여기서 멈춘다.
"""

from __future__ import annotations

from dataclasses import dataclass

PLACEHOLDER = "<CONFIGURE>"


class UnmeasuredValue(ValueError):
    """아직 측정되지 않은 설정값으로 기동을 시도했다."""


@dataclass(frozen=True)
class ArucoThresholds:
    max_reprojection_px: float
    max_distance_m: float
    min_distance_m: float
    position_variance_m2: float


DEFAULT_HINT = "해당 측정 절차를 먼저 수행할 것."


def require_measured(name: str, value, hint: str = DEFAULT_HINT) -> float:
    """<CONFIGURE> 면 무엇을 어떻게 재야 하는지 알려주며 실패한다.

    hint 는 호출자가 준다. 여기서 하드코딩하면 모든 호출자가 같은 안내를
    받는데, 실제로 그것 때문에 마커 장착값이 없다는 오류가 ArUco 임계값
    스크립트를 가리켰다. 엉뚱한 도구로 보내는 안내는 없는 것보다 나쁘다.
    """
    if isinstance(value, str):
        text = value.strip()
        if text == PLACEHOLDER or not text:
            raise UnmeasuredValue(f"'{name}' 이(가) 아직 {PLACEHOLDER} 다. {hint}")
        try:
            return float(text)
        except ValueError as exc:
            raise UnmeasuredValue(f"'{name}' 을(를) 숫자로 읽을 수 없다: {value!r}") from exc
    return float(value)


def parse_aruco_thresholds(values: dict) -> ArucoThresholds:
    fields = ("max_reprojection_px", "max_distance_m",
              "min_distance_m", "position_variance_m2")
    missing = [name for name in fields if name not in values]
    if missing:
        raise UnmeasuredValue(f"설정에 없는 항목: {', '.join(missing)}")

    hint = ("scripts/26_characterize_aruco.py 로 실측한 뒤 "
            "src/follower_bringup/config/aruco.yaml 에 채울 것. 짐작한 임계값은 "
            "관측을 과하게 버리거나 틀린 pose 를 통과시키는데, 둘 다 나중에 "
            "원인을 가리기 어렵다.")
    parsed = {name: require_measured(name, values[name], hint) for name in fields}
    thresholds = ArucoThresholds(**parsed)

    if thresholds.min_distance_m >= thresholds.max_distance_m:
        raise UnmeasuredValue(
            f"min_distance_m ({thresholds.min_distance_m}) 이 "
            f"max_distance_m ({thresholds.max_distance_m}) 이상이다")
    for name, value in parsed.items():
        if value <= 0.0:
            raise UnmeasuredValue(f"'{name}' 은(는) 양수여야 한다: {value}")
    return thresholds
