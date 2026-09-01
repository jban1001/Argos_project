"""aruco_pose 노드가 실제로 pose 를 내는지 확인한다.

기동 거부만 시험하고 정상 경로를 두면, 거부는 잘 되는데 아무것도 발행하지
않는 노드가 통과한다.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tests"))
import aruco_synthetic as synth                                    # noqa: E402

from follower_localization.aruco_pose_node import ArucoPoseNode    # noqa: E402
from follower_localization.config import UnmeasuredValue           # noqa: E402

MEASURED = [
    Parameter("max_reprojection_px", value="2.0"),
    Parameter("max_distance_m", value="2.5"),
    Parameter("min_distance_m", value="0.25"),
    Parameter("position_variance_m2", value="0.0004"),
]


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(overrides=None):
    return ArucoPoseNode(parameter_overrides=list(overrides or MEASURED))


def camera_info() -> CameraInfo:
    message = CameraInfo()
    message.k = [float(v) for v in synth.K.flatten()]
    message.d = [0.0] * 5
    return message


def image_of(distance_m: float, rvec=None) -> Image:
    from cv_bridge import CvBridge
    picture = synth.at(distance_m, rvec=rvec)
    message = CvBridge().cv2_to_imgmsg(picture, encoding="mono8")
    message.header.frame_id = "follower_camera"
    return message


def collect(node, topic_attr):
    captured = []
    getattr(node, topic_attr).publish = lambda m: captured.append(m)
    return captured


def test_refuses_unmeasured_thresholds(ros):
    with pytest.raises(UnmeasuredValue):
        make_node([Parameter("max_reprojection_px", value="<CONFIGURE>")])


def test_publishes_pose_for_a_visible_marker(ros):
    node = make_node()
    poses = collect(node, "_pose_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(1.0))
    node.destroy_node()

    assert len(poses) == 1, "마커가 보이는데 pose 가 나오지 않았다"
    position = poses[0].pose.pose.position
    assert position.z == pytest.approx(1.0, abs=0.02)
    assert poses[0].pose.covariance[0] == pytest.approx(0.0004)


def test_publishes_both_solutions(ros):
    node = make_node()
    best = collect(node, "_pose_pub")
    alt = collect(node, "_alt_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(1.5))
    node.destroy_node()

    assert len(best) == 1 and len(alt) == 1, "두 해가 모두 나가야 한다"
    a = best[0].pose.pose.orientation
    b = alt[0].pose.pose.orientation
    assert (a.x, a.y, a.z, a.w) != (b.x, b.y, b.z, b.w)


def test_nothing_published_before_camera_info(ros):
    """내부 파라미터 없이 pose 를 내면 그 값은 무의미하다."""
    node = make_node()
    poses = collect(node, "_pose_pub")
    node._on_image(image_of(1.0))
    node.destroy_node()
    assert poses == []


def test_too_far_is_rejected(ros):
    node = make_node()
    poses = collect(node, "_pose_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(3.0))          # max_distance_m = 2.5
    node.destroy_node()
    assert poses == []


def test_too_close_is_rejected(ros):
    node = make_node()
    poses = collect(node, "_pose_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(0.20))         # min_distance_m = 0.25
    node.destroy_node()
    assert poses == []


def test_status_reports_the_rejection_reason(ros):
    import json
    node = make_node()
    status = collect(node, "_status_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(3.0))
    node.destroy_node()

    assert status, "거부해도 상태는 나가야 한다 -- 안 그러면 조용히 멈춘 것과 구분되지 않는다"
    payload = json.loads(status[-1].data)
    assert payload["seen"] is True
    assert payload["accepted"] is False
    assert "too far" in payload["reason"]


def test_status_carries_the_ambiguity(ros):
    import json
    node = make_node()
    status = collect(node, "_status_pub")
    node._on_camera_info(camera_info())
    node._on_image(image_of(1.5))
    node.destroy_node()

    payload = json.loads(status[-1].data)
    assert payload["accepted"] is True
    assert payload["ambiguity"] > 0.5, "정면인데 모호성이 낮게 보고됐다"
    assert payload["solution_gap_deg"] > 1.0


def test_blank_frame_reports_not_seen(ros):
    import json
    from cv_bridge import CvBridge
    node = make_node()
    status = collect(node, "_status_pub")
    poses = collect(node, "_pose_pub")
    node._on_camera_info(camera_info())
    blank = CvBridge().cv2_to_imgmsg(np.full((480, 640), 255, np.uint8), encoding="mono8")
    node._on_image(blank)
    node.destroy_node()

    assert poses == []
    assert json.loads(status[-1].data)["seen"] is False
