"""마커를 보고 T_C_A (카메라 -> 마커) 를 발행한다.

발행:
    /follower/aruco/pose      PoseWithCovarianceStamped   가장 잘 맞는 해
    /follower/aruco/pose_alt  PoseWithCovarianceStamped   두 번째 해 (아래 설명)
    /follower/aruco/status    std_msgs/String (JSON)      품질 지표
    TF  follower_camera -> main_marker

왜 두 해를 다 내보내는가
------------------------
정사각 평면 마커를 정면에 가깝게 보면 두 개의 해가 거의 같은 이미지를 만들고,
PnP 는 노이즈에 따라 프레임마다 다른 쪽을 고른다. spec section 16 의 연쇄가
inv(T_C_A) 를 쓰므로 그 뒤집힘은 그대로 팔로워 위치 오차가 된다 -- 1.5 m 에서
8.7 도면 23 cm 다.

모호한 프레임을 버리는 방식은 여기서 쓸 수 없다. 팔로워가 마커 정면에서
따라가므로 그런 프레임이 대부분이고, 버리면 관측이 남지 않는다. 대신 협조
위치추정 쪽이 메인 로봇 AMCL 자세로 맞는 해를 고른다. 그러려면 두 해가 모두
나가야 한다.

임계값이 아직 <CONFIGURE> 면 이 노드는 기동하지 않는다. 짐작한 임계값으로
돌리면 시스템은 도는 것처럼 보이지만 결과가 조용히 틀린다.
"""

from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from follower_localization import aruco_detection as ad
from follower_localization.config import UnmeasuredValue, parse_aruco_thresholds
from follower_localization.transforms import matrix_to_quaternion


class ArucoPoseNode(Node):

    def __init__(self, **kwargs) -> None:
        # kwargs 는 parameter_overrides 를 넣기 위한 것이다. 시험에서 임계값을
        # 채운 노드를 만들 수 있어야, 기동 거부만 검증하고 정작 동작은
        # 확인하지 않는 상황을 피할 수 있다.
        super().__init__("aruco_pose", **kwargs)

        self.declare_parameter("marker_id", 5)
        self.declare_parameter("marker_size_m", 0.115)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_alternative", True)
        self.declare_parameter("marker_frame_id", "main_marker")
        # 거리 보정 [m]. solvePnP 가 내는 거리가 실제보다 짧게 나온다
        # (2026-08-29 실측: 줄자 0.50 m 에서 0.304 m, 편차 -196 mm).
        # 보정하지 않으면 팔로워가 실제보다 가깝다고 믿어 덜 가고, 뒤처지다
        # 마커를 놓친다. 거리 게이트도 같은 만큼 어긋난다.
        self.declare_parameter("range_bias_m", 0.0)

        # 임계값은 동적 타입으로 선언한다. 아직 안 잰 값은 <CONFIGURE> 라는
        # 문자열로, 다 잰 값은 숫자로 들어오는데 둘 다 통과해야 한다.
        # 문자열로 고정 선언하면 실측값을 채운 순간 타입 오류로 기동을 못 하고,
        # float 로 고정하면 <CONFIGURE> 를 잡아낼 기회가 없다.
        dynamic = ParameterDescriptor(dynamic_typing=True)
        for name in ("max_reprojection_px", "max_distance_m",
                     "min_distance_m", "position_variance_m2"):
            self.declare_parameter(name, "<CONFIGURE>", dynamic)

        raw = {name: self.get_parameter(name).value
               for name in ("max_reprojection_px", "max_distance_m",
                            "min_distance_m", "position_variance_m2")}
        self._limits = parse_aruco_thresholds(raw)

        self._marker_id = int(self.get_parameter("marker_id").value)
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        self._marker_frame = str(self.get_parameter("marker_frame_id").value)
        self._range_bias = float(self.get_parameter("range_bias_m").value)
        self._publish_alt = bool(self.get_parameter("publish_alternative").value)
        self._dictionary = ad.get_dictionary(str(self.get_parameter("dictionary").value))
        self._parameters = ad.default_detector_parameters()

        self._bridge = CvBridge()
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "aruco/pose", 10)
        self._alt_pub = self.create_publisher(
            PoseWithCovarianceStamped, "aruco/pose_alt", 10)
        self._status_pub = self.create_publisher(String, "aruco/status", 10)
        self._tf = TransformBroadcaster(self)

        self.create_subscription(CameraInfo, "camera/camera_info",
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, "camera/image_raw",
                                 self._on_image, qos_profile_sensor_data)

        self._seen = 0
        self._total = 0
        self.get_logger().info(
            f"aruco_pose: marker {self._marker_id}, {self._marker_size * 1000:.0f} mm, "
            f"{self._limits.min_distance_m:.2f}-{self._limits.max_distance_m:.2f} m, "
            f"reproj <= {self._limits.max_reprojection_px:.2f} px")

    def _on_camera_info(self, message: CameraInfo) -> None:
        # camera_info 를 쓰는 이유: 카메라 노드가 camera_info_url 로 읽은 것과
        # 같은 값이 보장된다. yaml 을 여기서 또 읽으면 두 벌이 되어 갈라진다.
        self._camera_matrix = np.array(message.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(message.d, dtype=np.float64).reshape(1, -1)

    def _on_image(self, message: Image) -> None:
        if self._camera_matrix is None:
            # 조용히 반환하면 노드가 살아 있는데 아무 일도 안 하는 것처럼
            # 보인다. 실제로 camera_name 불일치로 캘리브레이션이 안 실렸을 때
            # status 가 한 번도 안 나와 원인을 찾는 데 오래 걸렸다.
            self.get_logger().warn(
                "camera_info 를 아직 못 받았다 -- 카메라의 camera_name 이 "
                "camera_info.yaml 의 camera_name 과 같은지 확인할 것",
                throttle_duration_sec=5.0)
            return
        self._total += 1
        image = self._bridge.imgmsg_to_cv2(message, "mono8")
        found = ad.detect(image, self._camera_matrix, self._dist_coeffs,
                          self._marker_id, self._marker_size,
                          self._dictionary, self._parameters)

        status = {"seen": found is not None}
        if found is None:
            self._publish_status(status)
            return

        # 보정은 게이팅보다 먼저 건다. 게이트가 보는 거리와 발행하는 거리가
        # 다르면, 어느 쪽이 임계값에 걸렸는지 나중에 구분할 수 없다.
        found = self._apply_range_bias(found)

        distance = found.distance_m
        status.update({
            "distance_m": round(distance, 4),
            "reprojection_px": round(found.reprojection_px, 4),
            "apparent_px": round(found.apparent_px, 1),
            "ambiguity": round(found.ambiguity, 4),
            "solution_gap_deg": round(found.solution_gap_deg, 2),
        })

        rejected = self._rejection_reason(found)
        status["accepted"] = rejected is None
        if rejected is not None:
            status["reason"] = rejected
            self._publish_status(status)
            return

        self._seen += 1
        stamp = message.header.stamp
        frame = message.header.frame_id or "follower_camera"

        self._pose_pub.publish(self._as_pose(stamp, frame,
                                             found.rotation, found.translation))
        if self._publish_alt and found.rotation_alt is not None:
            self._alt_pub.publish(self._as_pose(stamp, frame,
                                                found.rotation_alt,
                                                found.translation_alt))
        self._broadcast_tf(stamp, frame, found.rotation, found.translation)
        self._publish_status(status)

    def _rejection_reason(self, found: ad.Detection) -> str | None:
        distance = found.distance_m
        if distance < self._limits.min_distance_m:
            return f"too close: {distance:.2f} m"
        if distance > self._limits.max_distance_m:
            return f"too far: {distance:.2f} m"
        if found.reprojection_px > self._limits.max_reprojection_px:
            return f"reprojection {found.reprojection_px:.2f} px"
        return None

    def _apply_range_bias(self, found):
        """거리 편차를 더한다. 방위는 건드리지 않는다.

        평행이동 벡터의 길이만 늘리므로 마커가 화면 어디에 있는지는 그대로다.
        더하는 보정이지 곱하는 보정이 아니다 -- 곱셈이면 마커 실측 크기가
        틀렸다는 뜻인데, 검은 테두리 한 변을 115 mm 로 직접 쟀으므로 그쪽이
        아니라 거리 기준점(렌즈 앞면 대 광학 중심)의 문제로 본다. 두 번째
        거리에서 한 번 더 재면 어느 쪽인지 갈린다.
        """
        if self._range_bias == 0.0:
            return found

        def _stretch(translation):
            if translation is None:
                return None
            norm = float(np.linalg.norm(translation))
            if norm < 1e-9:
                return translation
            return translation * ((norm + self._range_bias) / norm)

        # distance_m 은 필드가 아니라 translation 에서 계산되는 값이므로
        # 따로 넘기지 않는다. 평행이동만 늘리면 거리도 따라 늘어난다.
        return replace(
            found,
            translation=_stretch(found.translation),
            translation_alt=_stretch(found.translation_alt))

    def _as_pose(self, stamp, frame_id: str, rotation: np.ndarray,
                 translation: np.ndarray) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.pose.position.x = float(translation[0])
        message.pose.pose.position.y = float(translation[1])
        message.pose.pose.position.z = float(translation[2])
        x, y, z, w = matrix_to_quaternion(rotation)
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        # 위치 3 축만 채운다. 자세 공분산은 모호성 때문에 등방적이지 않으므로
        # 하나의 숫자로 뭉개면 거짓 정보가 된다 -- 협조 위치추정이 두 해 중
        # 고르는 방식으로 다루고, 여기서는 비워 둔다.
        variance = self._limits.position_variance_m2
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = variance
        return message

    def _broadcast_tf(self, stamp, frame_id: str, rotation: np.ndarray,
                      translation: np.ndarray) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = frame_id
        transform.child_frame_id = self._marker_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        x, y, z, w = matrix_to_quaternion(rotation)
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self._tf.sendTransform(transform)

    def _publish_status(self, status: dict) -> None:
        status["frames"] = self._total
        status["accepted_frames"] = self._seen
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(message)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    try:
        node = ArucoPoseNode()
    except UnmeasuredValue as exc:
        print(f"aruco_pose 기동 거부: {exc}")
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
