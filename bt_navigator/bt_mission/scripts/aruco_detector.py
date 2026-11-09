#!/usr/bin/env python3
"""
카메라 영상에서 ArUco dock board 를 찾아, 카메라에서 본 dock 좌표계 D 의 자세와 그 불확실도(공분산)를
dock/aruco (bt_mission_msgs/ArucoDetection) 로 발행한다. ArUco 도킹 BT 노드(DetectArucoDock, ArucoServo,
ArucoRefine)가 이것을 받는다 (ARUCO_DOCKING.md).

  ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True              # 시뮬레이션
  ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera          # RealSense (realsense-ros 4.x)

1. OpenCV ArucoDetector 로 marker 를 찾는다. 모서리는 서브픽셀로 다듬는다 (CORNER_REFINE_SUBPIX).
   영상 끝에 걸린 marker 는 모서리를 믿을 수 없으므로 버린다.
2. 한 board 의 marker 가 몇 개든 그 모서리를 모두 모아 PnP 를 한 번에 푼다 (marker 4개면 점 16개).
   - 점들이 한 평면에 있으므로 SOLVEPNP_IPPE 로 두 해를 구해 재투영 오차가 작은 쪽을 고르고, LM 으로 다듬는다.
   - 두 해의 오차 비(ambiguity)도 낸다. 점이 넓게 퍼질수록 틀린 해의 오차가 커져서 뒤집힘이 사라진다
     (QR 하나, marker 하나일 때의 법선 뒤집힘이 여기서 줄어든다).
   - marker 가 하나만 보이면 그 정사각형으로 SOLVEPNP_IPPE_SQUARE.
3. 공분산: 모서리 위치의 잡음(pixel_sigma, 재투영 오차가 더 크면 그것)을 PnP 의 Jacobian 으로 자세로 옮긴다
   (Σ = σ² (JᵀJ)⁻¹). 멀리서 작게 보이면 크고, 가까이서 크게 보이면 작다. BT 의 필터가 이것으로 측정마다
   무게를 다르게 준다.
4. depth_topic 을 주면 판 위 깊이 점들에 평면을 맞춰 판의 기울기(법선)를 바꾼다 (qr_detector.py 와 같은 방법).
   판이 넓어 PnP 만으로도 법선이 좋으므로 기본은 쓰지 않는다.

board 는 config/aruco_boards.yaml 에 dock 좌표계로 적는다. board_scale 은 그 크기와 자리 전체에 곱한다
(인쇄가 1 % 크게 나왔으면 1.01. 시뮬레이션에서 크기 오차를 일부러 넣을 때도 쓴다).
"""

import math
import os

import cv2
import numpy as np
import yaml

SIDE_MARGIN = 6          # 영상 끝에서 이 px 안에 모서리가 있는 marker 는 버린다


# ---------------------------------------------------------------------------------------------
# board

class Board:
    """dock 좌표계 D 와 board 좌표계 B.
    B: 원점 = D 의 (-board_offset, 0, 0), x_B = +y_D (영상에서 오른쪽), y_B = +z_D (위), z_B = +x_D (카메라 쪽).
    OpenCV ArUco 의 marker 좌표계(x 오른쪽, y 위, z 앞)와 같은 방향이라 marker 모서리를 (y, z, 0) 으로 적는다."""

    R_D_B = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # 열: x_B, y_B, z_B 를 D 에서 본 것

    def __init__(self, name, cfg, scale=1.0):
        self.name = name
        self.offset = cfg['board_offset'] * scale
        self.markers = {m['id']: (m['y'] * scale, m['z'] * scale, m['size'] * scale) for m in cfg['markers']}

    def corners(self, marker_id):
        """marker 모서리 4개 (B 좌표, OpenCV 순서: 왼위, 오른위, 오른아래, 왼아래)"""
        y, z, s = self.markers[marker_id]
        h = s / 2.0
        return np.array([[y - h, z + h, 0.0], [y + h, z + h, 0.0], [y + h, z - h, 0.0], [y - h, z - h, 0.0]])

    def dock_from_board(self, R_cam_B, t_cam_B):
        """T_cam_B -> T_cam_D"""
        R_cam_D = R_cam_B @ self.R_D_B.T
        t_cam_D = t_cam_B.reshape(3) + R_cam_D @ np.array([self.offset, 0.0, 0.0])
        return R_cam_D, t_cam_D


def load_boards(path, scale=1.0):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg['dictionary']))
    return dictionary, {name: Board(name, b, scale) for name, b in cfg['boards'].items()}


# ---------------------------------------------------------------------------------------------
# PnP

def solve_board(obj, img, camera_matrix, dist_coeffs, square_size=None):
    """board 점들 -> (rvec, tvec, 재투영 RMS px, ambiguity). 실패하면 None.
    square_size: marker 하나만 보일 때 그 한 변 (obj 는 그 marker 의 모서리 4개)"""
    obj = np.asarray(obj, np.float64).reshape(-1, 3)
    img = np.asarray(img, np.float64).reshape(-1, 1, 2)
    if square_size is not None:
        # IPPE_SQUARE 는 원점이 가운데인 정사각형만 받는다: marker 자리로 풀고 B 로 옮긴다
        center = obj.mean(axis=0)
        ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj - center, img, camera_matrix, dist_coeffs,
                                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        rvecs = list(rvecs)
        tvecs = [t.reshape(3) - cv2.Rodrigues(r)[0] @ center for r, t in zip(rvecs, tvecs)]
    else:
        ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
    errors = np.asarray(errors).reshape(-1)
    order = np.argsort(errors)
    rvec, tvec = np.asarray(rvecs[order[0]], np.float64).reshape(3, 1), np.asarray(tvecs[order[0]], np.float64).reshape(3, 1)
    ambiguity = float(errors[order[1]] / max(errors[order[0]], 1e-9)) if len(errors) > 1 else 0.0
    if square_size is None:
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, camera_matrix, dist_coeffs, rvec, tvec)
    projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))
    return rvec, tvec, rms, ambiguity


def small_rotation(R, R0):
    """R = exp([phi]) R0 의 phi (카메라 축 둘레의 작은 각). 아주 작은 각은 Rodrigues(acos)로 구하면 0 이 되므로
    반대칭 부분으로 구한다"""
    d = R @ R0.T
    return 0.5 * np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])


def pose_covariance(board, obj, rvec, tvec, camera_matrix, dist_coeffs, sigma_px):
    """모서리 잡음 sigma_px -> D 의 [위치 x y z, 작은 회전 rx ry rz] (카메라 frame) 의 6x6 공분산"""
    _, jac = cv2.projectPoints(np.asarray(obj, np.float64).reshape(-1, 3), rvec, tvec, camera_matrix, dist_coeffs)
    J = jac[:, :6]                                        # d(영상 점) / d(rvec, tvec)
    info = J.T @ J / (sigma_px ** 2)
    cov_params = np.linalg.inv(info + np.eye(6) * 1e-12)

    def output(params):
        R_B = cv2.Rodrigues(params[:3].reshape(3, 1))[0]
        R_D, t_D = board.dock_from_board(R_B, params[3:])
        return R_D, t_D

    p0 = np.concatenate([rvec.reshape(3), tvec.reshape(3)])
    R0, t0 = output(p0)
    A = np.zeros((6, 6))
    eps = 1e-6
    for i in range(6):
        p = p0.copy()
        p[i] += eps
        R, t = output(p)
        A[:3, i] = (t - t0) / eps
        A[3:, i] = small_rotation(R, R0) / eps
    return A @ cov_params @ A.T


# ---------------------------------------------------------------------------------------------
# 깊이 평면 (qr_detector.py 와 같은 방법: 1/z 를 x', y' 의 일차식으로)

def fit_plane(points):
    """N x 3 점 (카메라 optical frame) -> (단위 법선, 잔차 RMS). 깊이 잡음은 시선을 따라 생기므로 1/z 로 맞춘다"""
    for _ in range(2):
        a = np.column_stack([points[:, 0] / points[:, 2], points[:, 1] / points[:, 2], np.ones(len(points))])
        coef, *_ = np.linalg.lstsq(a, 1.0 / points[:, 2], rcond=None)
        scale = np.linalg.norm(coef)
        normal, d = coef / scale, 1.0 / scale
        residual = np.abs(points @ normal - d)
        keep = residual < max(3.0 * float(np.median(residual)), 0.002)
        if keep.all() or keep.sum() < 30:
            break
        points = points[keep]
    return normal, float(np.sqrt(np.mean((points @ normal - d) ** 2))), points


def depth_points(depth, polygon, scale, camera_matrix, shrink=0.9, max_points=4000):
    """컬러 영상의 다각형(볼록 껍질) 안쪽의 깊이 점들 -> N x 3"""
    hull = cv2.convexHull(polygon.astype(np.float32)).reshape(-1, 2)
    center = hull.mean(axis=0)
    inner = (center + (hull - center) * shrink) * scale
    mask = np.zeros(depth.shape[:2], np.uint8)
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


def apply_depth_normal(R_cam_B, points, cov_rot):
    """판의 z 축(법선)을 깊이 평면의 법선으로 바꾼다 (x 는 PnP 의 x 를 평면에 투영). 기울기 두 축의 분산은
    평면 맞춤에서, 판 안에서 도는 축의 분산은 PnP 에서 -> (R_cam_B, 3x3 회전 공분산, 잔차 RMS)"""
    normal, rms, used = fit_plane(points)
    if normal @ used.mean(axis=0) > 0:              # z_B 는 카메라 쪽
        normal = -normal
    x = R_cam_B[:, 0] - (R_cam_B[:, 0] @ normal) * normal
    x /= np.linalg.norm(x)
    R_new = np.column_stack([x, np.cross(normal, x), normal])
    centered = used - used.mean(axis=0)
    spread = math.sqrt(max(float(np.mean(np.sum(centered ** 2, axis=1))), 1e-6))
    sigma_tilt = max(rms, 0.0005) / (spread * math.sqrt(len(used) / 2.0))
    z = R_new[:, 2]
    var_inplane = float(z @ cov_rot @ z)
    cov = R_new @ np.diag([sigma_tilt ** 2, sigma_tilt ** 2, var_inplane]) @ R_new.T
    return R_new, cov, rms


def quaternion_from_matrix(R):
    rvec, _ = cv2.Rodrigues(R)
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.reshape(3) / angle
    s = math.sin(angle / 2.0)
    return axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)


# ---------------------------------------------------------------------------------------------
# ROS 노드

def main():
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from bt_mission_msgs.msg import ArucoDetection

    class ArucoDetector(Node):

        def __init__(self):
            super().__init__('aruco_detector')
            default_boards = ''
            try:
                from ament_index_python.packages import get_package_share_directory
                default_boards = os.path.join(get_package_share_directory('bt_mission'), 'config', 'aruco_boards.yaml')
            except Exception:  # noqa: BLE001  (패키지 밖에서 돌릴 때)
                pass
            boards_file = self.declare_parameter('boards_file', default_boards).value
            image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
            info_topic = self.declare_parameter('camera_info_topic', '/camera/color/camera_info').value
            depth_topic = self.declare_parameter('depth_topic', '').value
            self.depth_scale = self.declare_parameter('depth_scale', 0.001).value
            self.board_scale = self.declare_parameter('board_scale', 1.0).value
            self.pixel_sigma = self.declare_parameter('pixel_sigma', 0.5).value
            self.min_marker_px = self.declare_parameter('min_marker_px', 12.0).value
            self.optical_frame = self.declare_parameter('optical_frame', '').value
            self.publish_debug = self.declare_parameter('debug_image', True).value

            dictionary, self.boards = load_boards(boards_file, self.board_scale)
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params.cornerRefinementWinSize = 5
            params.cornerRefinementMaxIterations = 50
            params.cornerRefinementMinAccuracy = 0.01
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            self.bridge = CvBridge()
            self.camera_matrix = None
            self.dist_coeffs = None
            self.depth = None
            self.pub = self.create_publisher(ArucoDetection, 'dock/aruco', 10)
            self.debug_pub = self.create_publisher(Image, 'dock/aruco_debug', 1) if self.publish_debug else None
            self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
            if depth_topic:
                self.create_subscription(Image, depth_topic, lambda m: setattr(self, 'depth', m),
                                         qos_profile_sensor_data)
            self.msg_type = ArucoDetection
            self.get_logger().info(
                f'ArUco detector: {image_topic}, boards {list(self.boards)} from {boards_file} '
                f'(scale {self.board_scale}), normal from {"depth " + depth_topic if depth_topic else "PnP"}')

        def on_info(self, msg):
            self.camera_matrix = np.array(msg.k, np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, np.float64) if len(msg.d) else np.zeros(5)

        def depth_for(self, stamp, shape):
            msg = self.depth
            if msg is None:
                return None
            if abs((msg.header.stamp.sec - stamp.sec) + (msg.header.stamp.nanosec - stamp.nanosec) * 1e-9) > 0.1:
                return None
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * self.depth_scale
            scale = depth.shape[1] / shape[1]
            if abs(depth.shape[0] / shape[0] - scale) > 0.01:
                return None
            return depth, scale

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
            height, width = gray.shape
            corners, ids, _ = self.detector.detectMarkers(gray)
            found = {}
            if ids is not None:
                for c, i in zip(corners, ids.reshape(-1)):
                    c = c.reshape(4, 2)
                    side = float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))
                    if (c.min() < SIDE_MARGIN or (c[:, 0] > width - 1 - SIDE_MARGIN).any()
                            or (c[:, 1] > height - 1 - SIDE_MARGIN).any() or side < self.min_marker_px):
                        continue
                    found[int(i)] = c
            depth = self.depth_for(msg.header.stamp, gray.shape) if found else None
            for board in self.boards.values():
                used = sorted(i for i in found if i in board.markers)
                if not used:
                    continue
                obj = np.vstack([board.corners(i) for i in used])
                img = np.vstack([found[i] for i in used])
                square = board.markers[used[0]][2] if len(used) == 1 else None
                solved = solve_board(obj, img, self.camera_matrix, self.dist_coeffs, square)
                if solved is None:
                    continue
                rvec, tvec, rms, ambiguity = solved
                sigma = max(self.pixel_sigma, rms)
                cov = pose_covariance(board, obj, rvec, tvec, self.camera_matrix, self.dist_coeffs, sigma)
                R_B = cv2.Rodrigues(rvec)[0]
                source, plane_rms = self.msg_type.NORMAL_PNP, 0.0
                if depth is not None:
                    points = depth_points(depth[0], img, depth[1], self.camera_matrix)
                    if len(points) >= 200:
                        R_B, cov_rot, plane_rms = apply_depth_normal(R_B, points, cov[3:, 3:])
                        cov[3:, 3:] = cov_rot
                        cov[:3, 3:] = cov[3:, :3] = 0.0
                        source = self.msg_type.NORMAL_DEPTH
                R_D, t_D = board.dock_from_board(R_B, tvec)
                out = self.msg_type()
                out.header.stamp = msg.header.stamp
                out.header.frame_id = self.optical_frame or msg.header.frame_id
                out.board = board.name
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (float(v) for v in t_D)
                (out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z,
                 out.pose.pose.orientation.w) = quaternion_from_matrix(R_D)
                out.pose.covariance = [float(v) for v in cov.reshape(-1)]
                out.marker_ids = used
                out.reprojection_error = rms
                out.ambiguity = ambiguity
                out.normal_source = source
                out.plane_rms = plane_rms
                self.pub.publish(out)
                if self.debug_pub is not None:
                    cv2.aruco.drawDetectedMarkers(image, [found[i].reshape(1, 4, 2) for i in used],
                                                  np.array(used).reshape(-1, 1))
                    # 축은 D 의 원점(접점 면, 카메라 바로 앞일 수도 있다) 대신 보인 marker 들의 가운데에 그린다
                    center_B = np.mean([[board.markers[i][0], board.markers[i][1], 0.0] for i in used], axis=0)
                    center = R_B @ center_B + tvec.reshape(3)
                    cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, cv2.Rodrigues(R_D)[0],
                                      center.reshape(3, 1), 0.05, 2)
                    sx, sy = math.sqrt(cov[0, 0]), math.sqrt(cov[2, 2])
                    label = (f'{board.name} {len(used)} markers, board {float(np.linalg.norm(center)):.2f} m, '
                             f'rms {rms:.2f}px, dock sigma x {sx * 1000:.1f} z {sy * 1000:.1f} mm, amb {ambiguity:.0f}')
                    cv2.putText(image, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
                out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                out.header = msg.header
                self.debug_pub.publish(out)

    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
