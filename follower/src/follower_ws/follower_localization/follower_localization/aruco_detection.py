"""마커 한 개를 검출하고 카메라 기준 자세 T_C_A 를 낸다.

여기가 정본이다. scripts/26_characterize_aruco.py 도 이 모듈을 쓴다 -- 같은
계산이 두 벌 있으면 한쪽만 고쳐지고 갈라진다.

평면 자세 모호성
----------------
정사각 평면 마커를 정면에 가깝게 보면, 카메라 쪽으로 기운 해와 반대로 기운
해가 거의 같은 이미지를 만든다. PnP 는 둘 다 찾아내고 재투영 오차도 비슷해서
노이즈에 따라 프레임마다 다른 쪽을 고르고, 그때 마커 자세가 홱 뒤집힌다.

spec section 16 의 연쇄가 inv(T_C_A) 를 쓰므로 이 뒤집힘은 그대로 팔로워
위치 오차가 된다. 마커 자세가 theta 틀리면 위치는 대략 (거리 x theta) 만큼
틀린다. 합성 마커로 노이즈 없이 재 본 값:

    기울기 0 도 -> 모호성비 0.998, 두 해 차이 8.7 도
    기울기 20 도 -> 모호성비 0.050, 두 해 차이 42 도

1.5 m 에서 8.7 도면 23 cm 다. 그리고 이 로봇은 마커가 메인 로봇 뒤를 향하고
팔로워가 뒤에서 따라가므로 평소 시야가 바로 그 0 도 근처다. 그래서 모호성비를
같이 내보내고, 게이팅에서 쓴다.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    """한 프레임에서 얻은 마커 관측."""

    marker_id: int
    corners_px: np.ndarray = field(repr=False)      # (4, 2)
    rotation: np.ndarray = field(repr=False)        # (3, 3)  R_C_A
    translation: np.ndarray = field(repr=False)     # (3,)    t_C_A [m]
    reprojection_px: float
    apparent_px: float
    # 두 번째 해의 재투영 오차와의 비. 1 에 가까우면 두 해를 구분할 수 없다.
    # 두 번째 해가 없으면 0.0 (모호하지 않음).
    ambiguity: float = 0.0
    # 두 해 사이의 회전 차이 [deg]. 뒤집혔을 때 실제로 얼마나 틀리는지.
    solution_gap_deg: float = 0.0

    # 두 번째 해. 모호한 프레임을 버리는 게이팅은 이 로봇에서 쓸 수 없다 --
    # 팔로워가 마커 정면에서 따라가므로 모호성비가 늘 1 에 가깝고, 그렇게
    # 게이팅하면 관측을 전부 버리게 된다. 대신 협조 위치추정이 답을 준다:
    # 메인 로봇의 AMCL 자세를 이미 알고 있으므로, 두 해 중 그쪽과 맞는 것을
    # 고르면 된다. 그 선택을 하려면 두 해가 모두 밖으로 나가야 한다.
    rotation_alt: np.ndarray | None = field(default=None, repr=False)
    translation_alt: np.ndarray | None = field(default=None, repr=False)

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.translation))


def marker_object_points(size_m: float) -> np.ndarray:
    """detectMarkers 가 내놓는 모서리 순서(좌상,우상,우하,좌하)에 맞춘 마커 좌표계.

    OpenCV 의 estimatePoseSingleMarkers 와 같은 규약이다. 마커 좌표계는 +y 가
    위, +z 가 마커 면 바깥을 향한다. 이미지 좌표는 +y 가 아래이므로, 정면으로
    마주 본 마커의 회전은 항등행렬이 아니라 X 축 180 도 회전이 된다.
    """
    half = size_m / 2.0
    return np.array([[-half, half, 0.0],
                     [half, half, 0.0],
                     [half, -half, 0.0],
                     [-half, -half, 0.0]], dtype=np.float64)


def default_detector_parameters():
    """OpenCV 4.6 legacy API. 4.7 이상의 ArucoDetector 클래스는 없다."""
    params = cv2.aruco.DetectorParameters_create()
    # 모서리 정밀화는 pose 정확도에 직접 영향을 준다. 기본값은 꺼져 있다.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def get_dictionary(name: str):
    """'DICT_4X4_50' 같은 이름을 OpenCV 사전 객체로."""
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"알 수 없는 ArUco 사전: {name}")
    return cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))


def _rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    delta = R_a.T @ R_b
    cos = (np.trace(delta) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def detect(image: np.ndarray,
           camera_matrix: np.ndarray,
           dist_coeffs: np.ndarray,
           marker_id: int,
           marker_size_m: float,
           dictionary,
           parameters=None) -> Detection | None:
    """마커가 보이면 Detection, 아니면 None.

    marker_id 가 -1 이면 검출된 것 중 첫 번째를 쓴다. 실전에서는 반드시
    지정할 것 -- 다른 마커를 메인 로봇으로 착각하면 위치가 통째로 틀린다.
    """
    if parameters is None:
        parameters = default_detector_parameters()

    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    if ids is None or len(ids) == 0:
        return None

    flat = ids.flatten()
    if marker_id >= 0:
        matches = np.where(flat == marker_id)[0]
        if len(matches) == 0:
            return None
        index = int(matches[0])
    else:
        index = 0

    points = corners[index].reshape(4, 2).astype(np.float64)
    object_points = marker_object_points(marker_size_m)

    count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points, points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if count < 1:
        return None

    errors = np.asarray(errors, dtype=np.float64).flatten()
    order = np.argsort(errors)
    best = int(order[0])

    rotation, _ = cv2.Rodrigues(np.asarray(rvecs[best], dtype=np.float64))
    translation = np.asarray(tvecs[best], dtype=np.float64).flatten()

    ambiguity = 0.0
    gap_deg = 0.0
    rotation_alt = None
    translation_alt = None
    if count >= 2:
        second = int(order[1])
        if errors[second] > 0.0:
            ambiguity = float(errors[best] / errors[second])
        else:
            ambiguity = 1.0
        rotation_alt, _ = cv2.Rodrigues(np.asarray(rvecs[second], dtype=np.float64))
        translation_alt = np.asarray(tvecs[second], dtype=np.float64).flatten()
        gap_deg = _rotation_angle_deg(rotation, rotation_alt)

    sides = [float(np.linalg.norm(points[i] - points[(i + 1) % 4])) for i in range(4)]

    return Detection(
        marker_id=int(flat[index]),
        corners_px=points,
        rotation=rotation,
        translation=translation,
        reprojection_px=float(errors[best]),
        apparent_px=float(np.mean(sides)),
        ambiguity=ambiguity,
        solution_gap_deg=gap_deg,
        rotation_alt=rotation_alt,
        translation_alt=translation_alt,
    )


def pose_matrix(detection: Detection) -> np.ndarray:
    """T_C_A -- 카메라 좌표계에서 본 마커의 4x4 동차변환."""
    transform = np.eye(4)
    transform[:3, :3] = detection.rotation
    transform[:3, 3] = detection.translation
    return transform


def resolve_ambiguity(detection: Detection,
                      expected_rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool]:
    """두 해 중 기대 자세에 가까운 쪽을 고른다.

    expected_rotation 은 카메라 좌표계에서 마커가 어떤 자세일 것이라 믿는
    값이다 (메인 로봇 AMCL 자세 + 마커 장착값 + 팔로워 추정 자세에서 나온다).

    반환: (rotation, translation, used_alternative)

    재투영 오차만 보고 고르면 노이즈에 따라 프레임마다 뒤집히지만, 기대값과
    비교하면 그 선택이 안정된다. 기대값 자체가 틀리면 이 함수도 틀리므로,
    호출하는 쪽이 기대값의 신뢰도를 함께 판단해야 한다.
    """
    if detection.rotation_alt is None:
        return detection.rotation, detection.translation, False

    best = _rotation_angle_deg(expected_rotation, detection.rotation)
    alt = _rotation_angle_deg(expected_rotation, detection.rotation_alt)
    if alt < best:
        return detection.rotation_alt, detection.translation_alt, True
    return detection.rotation, detection.translation, False
