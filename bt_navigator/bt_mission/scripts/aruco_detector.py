#!/usr/bin/env python3
"""
카메라 영상에서 dock 의 ArUco marker (dock 하나에 하나) 를 찾아, 로봇(base) 좌표계에서 본 dock 의 자세
(바닥 위 x, y, yaw) 와 그 불확실도(공분산)를 dock/aruco (bt_mission_msgs/ArucoDetection) 로 발행한다.
도킹 BT 노드(DetectArucoDock, ArucoServo, ArucoRefine)가 이것을 받는다 (DOCKING.md).

  ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True              # 시뮬레이션
  ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera          # RealSense (realsense-ros 4.x)

1. marker 찾기: OpenCV ArUco. 모서리는 서브픽셀로 다듬는다 (CORNER_REFINE_SUBPIX).
   영상 끝에 걸린 것, 너무 작은 것은 모서리를 믿을 수 없어 버린다.
2. 6자유도 PnP (IPPE_SQUARE): 정사각형 하나의 PnP 는 해가 둘이다. 비스듬히 보거나 멀면 두 해의 재투영 오차가
   비슷해서, 오차가 작은 쪽을 고르면 가끔 반대쪽(법선이 시선 둘레로 뒤집힌 것)을 고른다. 또 marker 면의 기울기
   (법선) 는 네 모서리만으로는 잘 정해지지 않아서, 0.3 m 앞의 접점 자리로 옮기면 옆 오차가 몇 mm ~ 몇 cm 가 된다.
3. 바닥 사전분포를 둔 6자유도 MAP (이 노드의 핵심): dock 은 바닥에 서 있고, 카메라가 로봇 어디에 달렸는지(TF),
   marker 의 높이는 안다. 그렇다고 "기울기 0, 높이 0" 을 못박으면(3자유도) 카메라 높이가 1 cm, pitch 가 1 도만
   틀려도 옆 오차가 수 cm 가 된다 (test_aruco_detector.py). 그래서 base 에서 본 D 의 6자유도
   p = (x, y, yaw, z, roll, pitch) 를 풀되, z, roll, pitch 에 사전분포 N(0, σ_z²), N(0, σ_tilt²) 를 둔다:
     p* = argmin_p  Σ‖π(K, T_cam_base · T_base_D(p) · c_i) − u_i‖² / σ_px²  +  z²/σ_z² + roll²/σ_tilt² + pitch²/σ_tilt²
   (Levenberg-Marquardt, IPPE 두 해에서 시작). 영상이 기울기를 잘 말해 주면(가까이) 영상을 따르고, 잘 말해 주지
   못하면(멀리, 정면) 사전분포가 뒤집힌 해와 기울기-방향 섞임을 막는다. 두 시작점이 같은 자리로 모이면 후보 하나,
   다른 자리로 모이면 둘 다 낸다(비용 순). 고르는 것은 BT 의 필터가 한다 (지금까지의 추정에 가까운 쪽).
4. 공분산: 정보 행렬 JᵀJ (영상 8 + 사전 3 행) 의 역에서 (x, y, yaw) 부분 (나머지는 marginalize).
   σ_px = max(pixel_sigma, 재투영 RMS). 멀거나 비스듬하면 크고, 가까우면 작다. BT 의 필터가 이것으로 측정마다
   무게를 다르게 준다.
5. 진단: 추정한 dock 의 기울기(tilt_deg)와 바닥 높이(height_offset). 가까이서 늘 2 ~ 3 도, 1 cm 를 넘으면
   카메라 TF(특히 pitch, 높이), marker 높이가 config 와 다르다는 뜻이다 (경고를 낸다).

6. 깊이 (선택, depth_topic): marker 가 붙은 면의 깊이 점들에 평면을 맞춰 방향(yaw)을 재고, MAP 에 관측 한 줄로
   더한다. marker 가 작게 보이는 거리에서 방향이 좋아진다. 깊이가 유효하지 않은 가까운 거리에서는 저절로 빠진다.

dictionary 와 docks 는 config/docking.yaml (config_file) 에서 읽는다. 도킹 BT 노드와 같은 파일이다.
OpenCV 4.5 (Ubuntu 22.04 의 python3-opencv) 와 4.7 이상(ArucoDetector) 을 모두 쓴다.
"""

import math
import os

import cv2
import numpy as np
import yaml

SIDE_MARGIN = 6          # 영상 끝에서 이 px 안에 모서리가 있는 marker 는 버린다
DISTINCT_XY = 0.005      # 두 해가 이보다 가까우면 [m] (그리고 DISTINCT_YAW 안이면) 같은 해로 본다
DISTINCT_YAW = math.radians(0.5)


# ---------------------------------------------------------------------------------------------
# OpenCV 판 차이 (4.5/4.6: 함수, 4.7+: ArucoDetector 클래스)

def aruco_dictionary(name):
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('this OpenCV has no aruco module: apt install python3-opencv '
                           '(or pip install opencv-contrib-python)')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(dictionary, marker_id, side):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    return cv2.aruco.drawMarker(dictionary, marker_id, side)


class MarkerDetector:

    def __init__(self, dictionary):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        # 기본값 3 이면, 붙은 자리(0.33 m)에서 크게 보이는 marker 를 OpenCV 가 후보에서 빼 버리는 일이 있었다
        # (Gazebo 영상, test/data/gazebo_docked.jpg). 영상 끝에 걸린 marker 는 usable() 이 따로 거른다
        params.minDistanceToBorder = 0
        self.dictionary, self.params = dictionary, params
        self.detector = cv2.aruco.ArucoDetector(dictionary, params) if hasattr(cv2.aruco, 'ArucoDetector') else None

    def detect(self, gray):
        """-> {marker id: 4 x 2 모서리 (왼위, 오른위, 오른아래, 왼아래)}"""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2).astype(np.float64) for c, i in zip(corners, ids.reshape(-1))}


# ---------------------------------------------------------------------------------------------
# dock 과 marker

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class DockMarker:
    """dock 좌표계 D 의 marker. 모서리는 로봇이 dock 을 바라볼 때의 OpenCV 순서 (영상 오른쪽 = +y_D, 위 = +z_D)"""

    R_D_M = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # marker 좌표계 (x 오른쪽, y 위, z 앞)

    def __init__(self, name, cfg, scale=1.0):
        self.name = name
        self.marker_id = int(cfg['marker_id'])
        self.size = cfg['size'] * scale
        self.offset = cfg['offset']
        self.height = cfg['height']
        self.lateral = cfg.get('lateral', 0.0)
        h = self.size / 2.0
        x, y, z = -self.offset, self.lateral, self.height
        self.corners_D = np.array([[x, y - h, z + h], [x, y + h, z + h], [x, y + h, z - h], [x, y - h, z - h]])
        self.t_D_M = np.array([x, y, z])


def load_docks(path, scale=1.0):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg['dictionary'], {name: DockMarker(name, d, scale) for name, d in cfg['docks'].items()}


# ---------------------------------------------------------------------------------------------
# 자세

class Camera:
    """내부값과 로봇에 달린 자리 (T_base_cam: optical frame -> base)"""

    def __init__(self, K, dist, R_base_cam, t_base_cam):
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.dist = np.asarray(dist, np.float64).reshape(-1) if dist is not None and len(dist) else np.zeros(5)
        self.R_base_cam = np.asarray(R_base_cam, np.float64)
        self.t_base_cam = np.asarray(t_base_cam, np.float64).reshape(3)
        self.R_cam_base = self.R_base_cam.T
        self.t_cam_base = -self.R_cam_base @ self.t_base_cam

    def project(self, points_base):
        p = points_base @ self.R_cam_base.T + self.t_cam_base
        if (p[:, 2] < 0.01).any():
            return None
        image, _ = cv2.projectPoints(p, np.zeros(3), np.zeros(3), self.K, self.dist)
        return image.reshape(-1, 2)


def project_dock(q, marker, camera):
    """base 에서 본 D 의 자세 q = (x, y, yaw) -> marker 모서리 4개의 영상 좌표"""
    p = marker.corners_D @ rot_z(q[2]).T + np.array([q[0], q[1], 0.0])
    return camera.project(p)


def ippe(corners, marker, camera):
    """6자유도 PnP 의 두 해 -> [(T_cam_M 의 R, t, 재투영 RMS)]"""
    h = marker.size / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj, corners.reshape(4, 1, 2), camera.K, camera.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return []
    errors = np.asarray(errors).reshape(-1)
    return [(cv2.Rodrigues(r)[0], np.asarray(t).reshape(3), float(e)) for r, t, e in zip(rvecs, tvecs, errors)]


def dock_from_marker(R_cam_M, t_cam_M, marker, camera):
    """6자유도 해 (카메라에서 본 marker) -> base 에서 본 D 의 (x, y, yaw) (높이와 기울기는 버린다)"""
    R_base_M = camera.R_base_cam @ R_cam_M
    t_base_M = camera.R_base_cam @ t_cam_M + camera.t_base_cam
    R_base_D = R_base_M @ marker.R_D_M.T
    t_base_D = t_base_M - R_base_D @ marker.t_D_M
    return np.array([t_base_D[0], t_base_D[1], math.atan2(R_base_D[1, 0], R_base_D[0, 0])])


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def dock_rotation(p):
    """p = (x, y, yaw, z, roll, pitch) -> R_base_D"""
    return rot_z(p[2]) @ rot_y(p[5]) @ rot_x(p[4])


def project_dock6(p, marker, camera):
    return camera.project(marker.corners_D @ dock_rotation(p).T + np.array([p[0], p[1], p[3]]))


def dock6_from_marker(R_cam_M, t_cam_M, marker, camera):
    """6자유도 해 (카메라에서 본 marker) -> base 에서 본 D 의 p = (x, y, yaw, z, roll, pitch)"""
    R_base_M = camera.R_base_cam @ R_cam_M
    t_base_M = camera.R_base_cam @ t_cam_M + camera.t_base_cam
    R = R_base_M @ marker.R_D_M.T
    t = t_base_M - R @ marker.t_D_M
    return np.array([t[0], t[1], math.atan2(R[1, 0], R[0, 0]), t[2], math.atan2(R[2, 1], R[2, 2]),
                     math.asin(-max(-1.0, min(1.0, R[2, 0])))])


class Prior:
    """dock 이 바닥에 서 있다는 믿음의 세기. sigma_z [m], sigma_tilt [rad]"""

    def __init__(self, sigma_z=0.01, sigma_tilt=math.radians(1.5)):
        self.sigma = np.array([sigma_z, sigma_tilt, sigma_tilt])


def refine_map(p0, corners, marker, camera, prior, sigma_px, iterations=30, yaw_obs=None):
    """바닥 사전분포를 둔 6자유도 Levenberg-Marquardt -> (p, 비용, 재투영 RMS px, 정규화 Jacobian). 실패하면 None.
    yaw_obs = (yaw, sigma): 깊이 평면에서 잰 dock 의 방향. 있으면 관측 한 줄을 더한다 (Jacobian 12 x 6)"""
    def residual(p):
        image = project_dock6(p, marker, camera)
        if image is None:
            return None
        rows = [(image - corners).reshape(-1) / sigma_px, p[3:] / prior.sigma]
        if yaw_obs is not None:
            rows.append([angle(p[2] - yaw_obs[0]) / yaw_obs[1]])
        return np.concatenate(rows)

    def jacobian(p, r):
        J = np.zeros((len(r), 6))
        for i in range(6):
            dp = np.zeros(6)
            dp[i] = 1e-7
            ri = residual(p + dp)
            if ri is None:
                return None
            J[:, i] = (ri - r) / 1e-7
        return J

    p = np.array(p0, np.float64)
    r = residual(p)
    if r is None:
        return None
    lam = 1e-3
    for _ in range(iterations):
        J = jacobian(p, r)
        if J is None:
            return None
        H = J.T @ J
        try:
            step = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), J.T @ r)
        except np.linalg.LinAlgError:
            return None
        r_new = residual(p + step)
        if r_new is not None and r_new @ r_new < r @ r:
            p, r, lam = p + step, r_new, max(lam * 0.3, 1e-9)
            if np.max(np.abs(step)) < 1e-10:
                break
        else:
            lam *= 10.0
            if lam > 1e6:
                break
    J = jacobian(p, r)
    if J is None:
        return None
    p[2], p[4], p[5] = angle(p[2]), angle(p[4]), angle(p[5])
    rms = float(math.sqrt(np.mean(r[:8] ** 2) * 2.0) * sigma_px)
    return p, float(r @ r), rms, J


def estimate(corners, marker, camera, pixel_sigma=0.5, prior=None, yaw_obs=None):
    """모서리 4개 -> dict(hypotheses=[(q=(x, y, yaw), 3x3 공분산, 재투영 rms)], ambiguity, tilt, height, distance).
    yaw_obs = (yaw, sigma): 깊이 평면에서 잰 방향 (depth_plane_yaw). 실패하면 None"""
    prior = prior or Prior()
    solutions = ippe(corners, marker, camera)
    if not solutions:
        return None
    refined = []
    for R, t, _ in solutions:
        out = refine_map(dock6_from_marker(R, t, marker, camera), corners, marker, camera, prior, pixel_sigma,
                         yaw_obs=yaw_obs)
        if out is not None:
            refined.append(out)
    if not refined:
        return None
    refined.sort(key=lambda o: o[1])
    hypotheses, kept = [], []
    for p, cost, rms, J in refined:
        if any(np.hypot(*(p[:2] - k[0][:2])) < DISTINCT_XY and abs(angle(p[2] - k[0][2])) < DISTINCT_YAW for k in kept):
            continue
        # 재투영 오차가 pixel_sigma 보다 크면 그만큼 영상 행을 덜 믿는다
        J = J.copy()
        J[:8] *= pixel_sigma / max(pixel_sigma, rms)
        try:
            cov6 = np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            continue
        cov = cov6[:3, :3]
        kept.append((p, cost))
        hypotheses.append((p[:3].copy(), 0.5 * (cov + cov.T), rms))
    if not hypotheses:
        return None
    best = kept[0][0]
    ambiguity = kept[1][1] / max(kept[0][1], 1e-9) if len(kept) > 1 else 0.0
    distance = float(np.linalg.norm(min(solutions, key=lambda s: s[2])[1]))
    return {'hypotheses': hypotheses, 'ambiguity': ambiguity, 'tilt': float(math.hypot(best[4], best[5])),
            'height': float(best[3]), 'distance': distance, 'depth_used': yaw_obs is not None}


def depth_plane_yaw(depth, corners, image_width, camera, min_range=0.2, max_range=3.0, grow=1.3,
                    min_points=150, floor_sigma=math.radians(0.3)):
    """marker 가 붙은 면의 방향을 깊이 영상에서: marker 둘레(흰 여백까지, grow 배)의 깊이 점들에 평면을 맞춘다.
    모서리 넷의 원근보다 훨씬 많은 점으로 방향을 재므로, marker 가 작게 보이는 거리(0.3 ~ 1 m)에서 방향이 좋아진다.
    depth: 컬러에 맞춘 깊이 [m] (크기는 달라도 화각은 같아야 한다). min_range 보다 가까운 점은 버린다
    (스테레오 깊이 카메라는 가까우면 못 잰다. D435 는 848 x 480 에서 약 0.2 m).
    -> (base 에서 본 dock x 축의 yaw, sigma) 또는 None (점이 모자라면)"""
    scale = depth.shape[1] / float(image_width)
    center = corners.mean(axis=0)
    polygon = (center + (corners - center) * grow) * scale
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, np.round(polygon).astype(np.int32), 1)
    v, u = np.nonzero(mask)
    z = depth[v, u].astype(np.float64)
    ok = np.isfinite(z) & (z > min_range) & (z < max_range)
    if ok.sum() < min_points:
        return None
    u, v, z = u[ok], v[ok], z[ok]
    fx, fy = camera.K[0, 0] * scale, camera.K[1, 1] * scale
    cx, cy = camera.K[0, 2] * scale, camera.K[1, 2] * scale
    xn, yn = (u - cx) / fx, (v - cy) / fy
    # 깊이 잡음은 시선 방향이므로 1/z 를 (x', y') 의 일차식으로 맞춘다: n . (x', y', 1) = d / z
    for _ in range(2):
        A = np.column_stack([xn, yn, np.ones(len(z))])
        coef, *_ = np.linalg.lstsq(A, 1.0 / z, rcond=None)
        residual = 1.0 / z - A @ coef
        keep = np.abs(residual) < max(3.0 * np.median(np.abs(residual)), 1e-4)
        if keep.all() or keep.sum() < min_points:
            break
        xn, yn, z = xn[keep], yn[keep], z[keep]
    normal = coef / np.linalg.norm(coef)
    if normal[2] > 0:                                  # 카메라를 보는 쪽으로
        normal = -normal
    axis = camera.R_base_cam @ normal                  # dock 의 x 축 (설비에서 로봇 쪽)
    if abs(axis[2]) > 0.5:
        return None                                    # 서 있는 면이 아니다
    # 방향의 불확실도: 깊이 잔차 / (면의 가로 폭 * sqrt(점 수)). 깊이의 고정 편향이 있으므로 floor_sigma 아래로는 믿지 않는다
    width = (xn.max() - xn.min()) * float(np.mean(z))
    rms = float(np.sqrt(np.mean((1.0 / z - np.column_stack([xn, yn, np.ones(len(z))]) @ coef) ** 2))) * float(np.mean(z)) ** 2
    sigma = max(floor_sigma, 3.5 * rms / max(width, 1e-3) / math.sqrt(len(z)))
    return math.atan2(axis[1], axis[0]), sigma


def border_distance(corners, width, height):
    """marker 모서리에서 영상 끝까지 가장 가까운 거리 [px]"""
    return float(min(corners[:, 0].min(), corners[:, 1].min(), width - 1 - corners[:, 0].max(),
                     height - 1 - corners[:, 1].max()))


def usable(corners, width, height, min_px):
    side = float(np.mean(np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)))
    inside = (corners.min() >= SIDE_MARGIN and (corners[:, 0] <= width - 1 - SIDE_MARGIN).all()
              and (corners[:, 1] <= height - 1 - SIDE_MARGIN).all())
    return inside and side >= min_px, side


def matrix_from_quaternion(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---------------------------------------------------------------------------------------------
# ROS 노드

def main():
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from bt_mission_msgs.msg import ArucoDetection, DockHypothesis

    class ArucoDetector(Node):

        def __init__(self):
            super().__init__('aruco_detector')
            default_config = ''
            try:
                from ament_index_python.packages import get_package_share_directory
                default_config = os.path.join(get_package_share_directory('bt_mission'), 'config', 'docking.yaml')
            except Exception:  # noqa: BLE001  (패키지 밖에서 돌릴 때)
                pass
            # dictionary 와 docks 는 도킹 BT 노드와 같은 파일에서 (config/docking.yaml)
            docks_file = os.path.expanduser(self.declare_parameter('config_file', default_config).value or default_config)
            image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
            info_topic = self.declare_parameter('camera_info_topic', '/camera/color/camera_info').value
            self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
            self.optical_frame = self.declare_parameter('optical_frame', '').value
            self.marker_scale = self.declare_parameter('marker_scale', 1.0).value
            self.pixel_sigma = self.declare_parameter('pixel_sigma', 0.5).value
            self.min_marker_px = self.declare_parameter('min_marker_px', 16.0).value
            self.prior = Prior(self.declare_parameter('prior_height_sigma', 0.01).value,
                               math.radians(self.declare_parameter('prior_tilt_sigma_deg', 1.5).value))
            self.tilt_warn = math.radians(self.declare_parameter('tilt_warn_deg', 3.0).value)
            self.height_warn = self.declare_parameter('height_warn', 0.015).value
            self.publish_debug = self.declare_parameter('debug_image', True).value
            # 영상 stamp 에 더하는 시간 [s]. 카메라 드라이버의 stamp 가 실제 찍은 때보다 늦으면 음수 (DOCKING.md 8-1)
            self.stamp_offset = self.declare_parameter('stamp_offset', 0.0).value
            # 깊이 (컬러에 맞춘 것, realsense 의 aligned_depth_to_color). 비우면 쓰지 않는다
            depth_topic = self.declare_parameter('depth_topic', '').value
            self.depth_scale = self.declare_parameter('depth_scale', 0.001).value      # 16UC1 -> m
            self.depth_min_range = self.declare_parameter('depth_min_range', 0.2).value
            self.depth_yaw_sigma = math.radians(self.declare_parameter('depth_yaw_sigma_deg', 0.3).value)
            self.depth = None

            dictionary_name, self.docks = load_docks(docks_file, self.marker_scale)
            self.by_id = {d.marker_id: d for d in self.docks.values()}
            self.detector = MarkerDetector(aruco_dictionary(dictionary_name))
            self.bridge = CvBridge()
            self.info = None
            self.camera = None
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(ArucoDetection, 'dock/aruco', 10)
            self.debug_pub = self.create_publisher(Image, 'dock/aruco_debug', 1) if self.publish_debug else None
            self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
            if depth_topic:
                self.create_subscription(Image, depth_topic, lambda m: setattr(self, 'depth', m),
                                         qos_profile_sensor_data)
            self.msg_types = ArucoDetection, DockHypothesis
            self.get_logger().info(
                f'ArUco detector (OpenCV {cv2.__version__}): {image_topic}, docks '
                f'{ {n: d.marker_id for n, d in self.docks.items()} } from {docks_file} (size x {self.marker_scale}), '
                f'poses in {self.base_frame}, yaw from depth: {depth_topic or "off"}')

        def on_info(self, msg):
            self.info = msg

        def depth_for(self, stamp):
            """영상과 같은 때(0.1 s 안)의 깊이 [m]. 없으면 None"""
            msg = self.depth
            if msg is None:
                return None
            if abs((msg.header.stamp.sec - stamp.sec) + (msg.header.stamp.nanosec - stamp.nanosec) * 1e-9) > 0.1:
                return None
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * self.depth_scale
            return depth

        def camera_for(self, frame):
            """내부값 + TF (base <- camera optical). 로봇에 고정이라 한 번 찾으면 계속 쓴다"""
            if self.camera is not None or self.info is None:
                return self.camera
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'no TF {self.base_frame} <- {frame}: {e}', throttle_duration_sec=5.0)
                return None
            q, p = t.transform.rotation, t.transform.translation
            self.camera = Camera(np.array(self.info.k).reshape(3, 3), np.array(self.info.d),
                                 matrix_from_quaternion(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
            self.get_logger().info(
                f'camera {frame} at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m in {self.base_frame}')
            return self.camera

        def on_image(self, msg):
            try:
                self.detect(msg)
            except (cv2.error, ValueError, np.linalg.LinAlgError) as e:   # 영상 하나가 이상해도 노드는 살아 있게
                self.get_logger().warn(f'skipped a frame: {e}', throttle_duration_sec=5.0)

        def detect(self, msg):
            camera = self.camera_for(self.optical_frame or msg.header.frame_id)
            if camera is None:
                return
            ArucoDetection, DockHypothesis = self.msg_types
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            found = self.detector.detect(gray)
            depth = self.depth_for(msg.header.stamp) if found else None
            stamp = msg.header.stamp
            if self.stamp_offset:
                stamp = (rclpy.time.Time.from_msg(stamp) + rclpy.duration.Duration(seconds=self.stamp_offset)).to_msg()
            for marker_id, corners in found.items():
                marker = self.by_id.get(marker_id)
                if marker is None:
                    continue
                ok, side = usable(corners, width, height, self.min_marker_px)
                if not ok:
                    continue
                yaw_obs = None
                if depth is not None:
                    yaw_obs = depth_plane_yaw(depth, corners, width, camera, self.depth_min_range,
                                              floor_sigma=self.depth_yaw_sigma)
                result = estimate(corners, marker, camera, self.pixel_sigma, self.prior, yaw_obs)
                if result is None:
                    continue
                if result['distance'] < 0.8 and (result['tilt'] > self.tilt_warn
                                                 or abs(result['height']) > self.height_warn):
                    self.get_logger().warn(
                        f"{marker.name}: the dock looks tilted {math.degrees(result['tilt']):.1f} deg and "
                        f"{result['height'] * 1000:+.0f} mm off the floor: check the camera TF (pitch, height), "
                        'the marker height and size in the config', throttle_duration_sec=5.0)
                out = ArucoDetection()
                out.header.stamp = stamp
                out.header.frame_id = self.base_frame
                out.dock = marker.name
                out.marker_id = marker_id
                out.marker_px = side
                out.camera_distance = result['distance']
                out.border_px = border_distance(corners, width, height)
                out.depth_used = result['depth_used']
                out.ambiguity = result['ambiguity']
                out.tilt_deg = math.degrees(result['tilt'])
                out.height_offset = result['height']
                for q, cov, rms in result['hypotheses']:
                    h = DockHypothesis()
                    h.x, h.y, h.yaw = (float(v) for v in q)
                    h.covariance = [float(v) for v in cov.reshape(-1)]
                    h.reprojection_error = rms
                    out.hypotheses.append(h)
                self.pub.publish(out)
                if self.debug_pub is not None:
                    self.draw(image, corners, marker, camera, result)
            if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
                out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                out.header = msg.header
                self.debug_pub.publish(out)

        def draw(self, image, corners, marker, camera, result):
            q, cov, rms = result['hypotheses'][0]
            cv2.polylines(image, [np.round(corners).astype(np.int32)], True, (0, 255, 0), 2)
            # 바닥 위 자세(x, y, yaw)로 다시 그린 모서리 (빨강). 초록과 겹치면 config 와 TF 가 맞다
            projected = project_dock(q, marker, camera)
            if projected is not None:
                for p in projected:
                    cv2.circle(image, (int(round(p[0])), int(round(p[1]))), 3, (0, 0, 255), -1)
            # 로봇이 dock 좌표계에서 어디 있나 (= q 의 역)
            c, s = math.cos(q[2]), math.sin(q[2])
            rx, ry = -(c * q[0] + s * q[1]), -(-s * q[0] + c * q[1])
            lines = [f'{marker.name} id {marker.marker_id}: {result["distance"]:.2f} m, rms {rms:.2f}px, '
                     f'tilt {math.degrees(result["tilt"]):.1f}deg z {result["height"] * 1000:+.0f}mm, '
                     f'{len(result["hypotheses"])} hyp' + (' +depth' if result['depth_used'] else ''),
                     f'robot in dock: x {rx * 1000:.0f} y {ry * 1000:+.0f} mm, yaw {math.degrees(angle(-q[2])):+.1f} deg',
                     f'sigma x {math.sqrt(cov[0, 0]) * 1000:.1f} y {math.sqrt(cov[1, 1]) * 1000:.1f} mm '
                     f'yaw {math.degrees(math.sqrt(cov[2, 2])):.2f} deg']
            for i, line in enumerate(lines):
                cv2.putText(image, line, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

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
