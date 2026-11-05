#!/usr/bin/env python3
"""
카메라 영상에서 QR 을 찾아, 카메라에서 본 QR 의 위치와 방향을 dock/qr (bt_mission_msgs/QrCode) 로 발행한다.
docking BT 노드(DetectDock, ApproachDock)가 이것을 받아 dock 의 자리를 잡는다.

  ros2 run bt_mission qr_detector.py --ros-args -p qr_size:=0.1448 \\
      -p image_topic:=/camera/camera/color/image_raw -p camera_info_topic:=/camera/camera/color/camera_info \\
      -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw

1. OpenCV QRCodeDetector 로 QR 의 바깥 모서리 4개와 내용을 찾는다 (영상 하나에 여러 개도 된다).
2. 모서리를 서브픽셀로 다듬고, QR 의 실제 한 변(qr_size)과 카메라 내부값(camera_info)으로 PnP 를 푼다.
   네 점이 한 평면의 정사각형이라 OpenCV 의 SOLVEPNP_IPPE_SQUARE 를 쓴다.
3. depth_topic 을 주면 QR 안쪽의 깊이 점들에 평면을 맞춰 법선(방향)을 다시 구한다 (아래).
4. 결과(QR 중심의 위치와 방향, 카메라 optical frame)를 영상 시각으로 발행한다. 영상의 frame_id 를 쓰고,
   그것이 optical frame 이 아니면 optical_frame 파라미터로 바꿔 적는다.

QR 좌표계 (QrCode.msg, OpenCV ArUco 마커와 같다): x = 인쇄된 QR 을 바로 봤을 때 오른쪽, y = 위,
z = QR 면에서 앞으로(카메라 쪽으로) 나오는 법선.

왜 깊이를 쓰나: 네 모서리만으로는 위치는 mm 까지 맞지만 방향은 약하다. QR 이 작게 보이면(멀리서) 기울기가
모서리 몇 px 의 차이로만 드러나고, 평면 정사각형의 PnP 는 답이 둘(시선을 축으로 뒤집은 것)이라 가끔 틀린 쪽을
고른다. 시뮬레이션에서 0.75 m 거리, 120 px 일 때 법선이 -4 도 근처에 있다가 가끔 +16 도로 뛰었다.
dock 의 자리(로봇이 설 곳)는 법선 방향으로 정해지므로 이것이 그대로 옆 오차가 된다.
RealSense 의 깊이는 QR 판 위 수천 점이라 평면의 기울기를 1 도 안으로 준다. 깊이는 컬러에 맞춘(aligned)
영상이어야 한다 (realsense-ros: align_depth.enable:=true). 크기가 컬러와 다르면 같은 화각으로 보고 비율로 맞춘다.

qr_size 는 바깥 finder pattern 모서리 사이의 길이다 (흰 여백 제외). 인쇄한 것을 자로 재서 넣는다.
1 mm 틀리면 거리가 0.1 m 마다 약 1 mm 틀린다 (거리는 qr_size 에 비례한다).
"""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from bt_mission_msgs.msg import QrCode


def quaternion_from_rvec(rvec):
    """Rodrigues 벡터 -> (x, y, z, w)"""
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.reshape(3) / angle
    s = math.sin(angle / 2.0)
    return axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)


def rotation_to_quaternion(r):
    """3x3 회전 행렬 -> (x, y, z, w)"""
    rvec, _ = cv2.Rodrigues(np.asarray(r, dtype=np.float64))
    return quaternion_from_rvec(rvec)


def fit_plane(points):
    """N x 3 점 (카메라 optical frame) -> (단위 법선, 평면까지 거리 잔차 RMS).
    깊이 잡음은 시선을 따라(주로 z) 생기므로, 점들에 그냥 평면을 맞추면(SVD) 기운 평면에서 기울기가 치우친다
    (10 mm 잡음에 약 1 도). 평면 n.p = d 위의 점은 1/z = (n_x x' + n_y y' + n_z) / d (x' = x/z, y' = y/z:
    픽셀에서 정해져 잡음이 없다) 이므로, 1/z 를 x', y' 의 일차식으로 맞춘다 (최소제곱). 크게 벗어난 점(가장자리의
    배경)은 빼고 한 번 더 맞춘다"""
    for _ in range(2):
        a = np.column_stack([points[:, 0] / points[:, 2], points[:, 1] / points[:, 2], np.ones(len(points))])
        coef, *_ = np.linalg.lstsq(a, 1.0 / points[:, 2], rcond=None)
        scale = np.linalg.norm(coef)
        normal, d = coef / scale, 1.0 / scale
        residual = np.abs(points @ normal - d)            # 평면까지의 거리 [m]
        keep = residual < max(3.0 * float(np.median(residual)), 0.002)
        if keep.all() or keep.sum() < 30:
            break
        points = points[keep]
    rms = float(np.sqrt(np.mean((points @ normal - d) ** 2)))
    return normal, rms


def depth_points(depth, corners, scale, camera_matrix, shrink=0.8, max_points=3000):
    """컬러 영상의 QR 모서리 안쪽(중심 쪽으로 shrink 배)의 깊이 점들 -> N x 3 (카메라 optical frame, m)"""
    center = corners.mean(axis=0)
    inner = (center + (corners - center) * shrink) * scale
    mask = np.zeros(depth.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(inner).astype(np.int32), 1)
    v, u = np.nonzero(mask)
    if len(v) > max_points:
        pick = np.linspace(0, len(v) - 1, max_points).astype(int)
        v, u = v[pick], u[pick]
    z = depth[v, u].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    u, v, z = u[ok], v[ok], z[ok]
    fx, fy = camera_matrix[0, 0] * scale, camera_matrix[1, 1] * scale
    cx, cy = camera_matrix[0, 2] * scale, camera_matrix[1, 2] * scale
    return np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1)


def normal_from_depth(rvec, points):
    """PnP 의 회전에서 z(법선)만 깊이 평면의 법선으로 바꾼다. x 는 PnP 의 x 를 평면에 투영한 것 -> (3x3, RMS)"""
    normal, rms = fit_plane(points)
    if normal @ points.mean(axis=0) > 0:          # QR 의 z 는 카메라 쪽을 본다
        normal = -normal
    r_pnp, _ = cv2.Rodrigues(rvec)
    x = r_pnp[:, 0] - (r_pnp[:, 0] @ normal) * normal
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(normal, x), normal]), rms


def solve_qr_pose(corners, size, camera_matrix, dist_coeffs):
    """QR 바깥 모서리 4개(영상, 왼위 -> 오른위 -> 오른아래 -> 왼아래) -> (rvec, tvec, 재투영 오차 px).
    QR 좌표계에서 모서리는 (-s/2,s/2) (s/2,s/2) (s/2,-s/2) (-s/2,-s/2), z = 0.
    SOLVEPNP_IPPE_SQUARE 는 이 순서(y 가 위)만 받는다"""
    h = size / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    error = float(np.mean(np.linalg.norm(projected.reshape(4, 2) - img.reshape(4, 2), axis=1)))
    return rvec, tvec, error


class QrDetector(Node):

    def __init__(self):
        super().__init__('qr_detector')
        self.qr_size = self.declare_parameter('qr_size', 0.116).value
        image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
        info_topic = self.declare_parameter('camera_info_topic', '/camera/color/camera_info').value
        self.optical_frame = self.declare_parameter('optical_frame', '').value
        self.min_pixel_size = self.declare_parameter('min_pixel_size', 20.0).value
        self.publish_debug = self.declare_parameter('debug_image', True).value
        depth_topic = self.declare_parameter('depth_topic', '').value
        self.depth_scale = self.declare_parameter('depth_scale', 0.001).value   # 16UC1 깊이의 단위 [m] (RealSense: mm)
        self.min_depth_points = self.declare_parameter('min_depth_points', 200).value
        self.depth = None

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.pub = self.create_publisher(QrCode, 'dock/qr', 10)
        self.debug_pub = self.create_publisher(Image, 'dock/qr_debug', 1) if self.publish_debug else None
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        if depth_topic:
            self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'QR detector: {image_topic}, qr_size {self.qr_size:.4f} m, '
            f'normal from {"depth " + depth_topic if depth_topic else "the corners only (no depth_topic)"}')

    def on_depth(self, msg):
        self.depth = msg

    def depth_for(self, stamp, color_shape):
        """영상과 0.1 s 안에 찍은 깊이 -> (float m 배열, 컬러 대비 크기 비율). 없으면 None"""
        msg = self.depth
        if msg is None:
            return None
        dt = abs((msg.header.stamp.sec - stamp.sec) + (msg.header.stamp.nanosec - stamp.nanosec) * 1e-9)
        if dt > 0.1:
            return None
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * self.depth_scale
        scale = depth.shape[1] / color_shape[1]
        if abs(depth.shape[0] / color_shape[0] - scale) > 0.01:
            self.get_logger().warn('depth and color have different aspect ratios: not aligned?', throttle_duration_sec=10.0)
            return None
        return depth, scale

    def on_info(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def on_image(self, msg):
        try:
            self.detect(msg)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as e:   # 영상 하나가 이상해도 노드는 살아 있게
            self.get_logger().warn(f'skipped a frame: {e}', throttle_duration_sec=5.0)

    def detect(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('no camera_info yet', throttle_duration_sec=5.0)
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(gray)
        if not ok or points is None:
            self.publish_debug_image(image, msg.header)
            return
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        depth = self.depth_for(msg.header.stamp, gray.shape)
        height, width = gray.shape
        for text, quad in zip(texts, points):
            if not text:                     # 모서리는 찾았지만 읽지 못했다
                continue
            quad = quad.astype(np.float32).reshape(4, 2)
            margin = 6                       # cornerSubPix 의 창 (5) + 1. 영상 끝에 걸린 QR 은 모서리를 믿을 수 없다
            if (quad.min() < margin or (quad[:, 0] > width - 1 - margin).any() or
                    (quad[:, 1] > height - 1 - margin).any()):
                continue
            corners = cv2.cornerSubPix(gray, quad.reshape(4, 1, 2), (5, 5), (-1, -1), criteria).reshape(4, 2)
            pixel_size = float(np.mean(np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)))
            if pixel_size < self.min_pixel_size:
                continue
            solved = solve_qr_pose(corners, self.qr_size, self.camera_matrix, self.dist_coeffs)
            if solved is None:
                continue
            rvec, tvec, error = solved
            rotation, plane_rms = None, 0.0
            if depth is not None:
                plane_points = depth_points(depth[0], corners, depth[1], self.camera_matrix)
                if len(plane_points) >= self.min_depth_points:
                    rotation, plane_rms = normal_from_depth(rvec, plane_points)
            out = QrCode()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.optical_frame or msg.header.frame_id
            out.text = text
            out.pose.position.x, out.pose.position.y, out.pose.position.z = (float(v) for v in tvec.reshape(3))
            (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
             out.pose.orientation.w) = (quaternion_from_rvec(rvec) if rotation is None
                                        else rotation_to_quaternion(rotation))
            out.normal_source = QrCode.NORMAL_PNP if rotation is None else QrCode.NORMAL_DEPTH
            out.plane_rms = plane_rms
            out.size = float(self.qr_size)
            out.pixel_size = pixel_size
            out.reprojection_error = error
            self.pub.publish(out)
            if self.debug_pub is not None:
                cv2.polylines(image, [corners.astype(np.int32)], True, (0, 200, 0), 2)
                axes = rvec if rotation is None else cv2.Rodrigues(rotation)[0]
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, axes, tvec, self.qr_size / 2, 2)
                label = f"{text} {float(tvec.reshape(3)[2]):.2f} m ({'pnp' if rotation is None else 'depth'})"
                cv2.putText(image, label, tuple(int(v) for v in corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 200, 0), 1)
        self.publish_debug_image(image, msg.header)

    def publish_debug_image(self, image, header):
        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            out.header = header
            self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
