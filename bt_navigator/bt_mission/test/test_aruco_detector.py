"""aruco_detector.py 의 계산 (marker 찾기, 6자유도 PnP, 바닥 사전분포를 둔 MAP, 공분산) 시험.

합성 영상: marker 그림을 알려진 자세의 카메라로 영상에 그린다. 모서리가 화소 아래(서브픽셀)까지 맞게
6배 크게 그린 뒤 줄인다 (그냥 그리면 테두리가 화소 격자에 붙어 0.3 px 쯤 틀린 모서리가 나온다).
"""

import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
DOCKS = os.path.join(HERE, '..', 'config', 'aruco_docks.yaml')


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ad = load('aruco_detector', os.path.join(HERE, '..', 'scripts', 'aruco_detector.py'))

W, H = 848, 480
F = W / 2 / math.tan(1.211 / 2)                  # D435 컬러 848 x 480, 수평 화각 69.4 도
K = np.array([[F, 0, (W - 1) / 2], [0, F, (H - 1) / 2], [0, 0, 1.0]])
# 시뮬레이션의 메카넘 (robots/make_mecanum.py): 카메라가 base 앞 0.15, 가운데, 높이 0.13 m. optical: x 오른쪽, y 아래, z 앞
R_BASE_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BASE_CAM = np.array([0.15, 0.0, 0.13])
PPM = 3000.0                                      # marker 그림의 해상도 [px/m]
SS = 6                                            # 크게 그리는 배수
DOCKED = (0.17, 0.0, math.pi)                     # 붙은 자리: 로봇 중심이 접점 면 0.17 m 앞, dock 을 본다


@pytest.fixture(scope='module')
def scene():
    name, docks = ad.load_docks(DOCKS)
    marker = docks['dock_a']
    dictionary = ad.aruco_dictionary(name)
    side = int(round(marker.size * PPM))
    quiet = side // 4
    texture = np.full((side + 2 * quiet, side + 2 * quiet), 255, np.uint8)
    texture[quiet:quiet + side, quiet:quiet + side] = ad.marker_image(dictionary, marker.marker_id, side)
    camera = ad.Camera(K, np.zeros(5), R_BASE_CAM, T_BASE_CAM)
    return marker, texture, quiet, ad.MarkerDetector(dictionary), camera


def dock_in_base(robot):
    """로봇 자세 (dock 좌표계에서 x, y, yaw) -> base 에서 본 dock (x, y, yaw)"""
    x, y, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([-(c * x + s * y), -(-s * x + c * y), ad.angle(-yaw)])


def render(scene, robot, seed=0, blur=0.6, noise=2.0):
    marker, texture, quiet, _, camera = scene
    q = dock_in_base(robot)
    R_cam_D = camera.R_cam_base @ ad.rot_z(q[2])
    t_cam_D = camera.R_cam_base @ np.array([q[0], q[1], 0.0]) + camera.t_cam_base
    # 그림의 화소 (u, v) (화소 중심 = 정수) -> D. marker 왼쪽 위 테두리가 u = v = quiet - 0.5
    y0 = marker.lateral - marker.size / 2 - (quiet - 0.5) / PPM
    z0 = marker.height + marker.size / 2 + (quiet - 0.5) / PPM
    A = np.array([[0.0, 0.0, -marker.offset], [1 / PPM, 0.0, y0], [0.0, -1 / PPM, z0]])
    homography = K @ (R_cam_D @ A + np.column_stack([np.zeros((3, 2)), t_cam_D]))
    S = np.array([[SS, 0, (SS - 1) / 2], [0, SS, (SS - 1) / 2], [0, 0, 1.0]])
    big = cv2.warpPerspective(texture, S @ homography, (W * SS, H * SS), flags=cv2.INTER_LINEAR, borderValue=110)
    image = cv2.resize(big, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32)
    rng = np.random.default_rng(seed)
    if blur > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur)
    image = image + rng.normal(0, noise, image.shape)
    return np.clip(image, 0, 255).astype(np.uint8)


def solve(scene, robot, seed=0, camera=None):
    marker, _, _, detector, true_camera = scene
    found = detector.detect(render(scene, robot, seed))
    assert marker.marker_id in found
    corners = found[marker.marker_id]
    return corners, ad.estimate(corners, marker, camera or true_camera)


def lateral_error_at_docked(q, truth):
    """추정으로 붙으면 로봇이 옆으로 얼마나 어긋나나 [m]: 붙은 자리(D 에서 0.17 m 앞)를 두 자세로 base 에 놓고 비교"""
    def docked_point(p):
        return p[:2] + ad.rot_z(p[2])[:2, :2] @ np.array([DOCKED[0], 0.0])
    normal = np.array([-math.sin(truth[2]), math.cos(truth[2])])
    return float((docked_point(q) - docked_point(truth)) @ normal)


def test_marker_frame_and_projection(scene):
    marker, _, _, _, camera = scene
    # 모서리: 왼위가 -y_D (로봇이 dock 을 볼 때 왼쪽), 위
    assert np.allclose(marker.corners_D[0], [-0.30, -0.06, 0.215])
    assert np.allclose(marker.corners_D[2], [-0.30, 0.06, 0.095])
    # 붙은 자리에서 marker 는 영상 가운데 근처에, 왼위 모서리가 왼쪽 위에
    p = ad.project_dock(dock_in_base(DOCKED), marker, camera)
    assert p[0, 0] < p[1, 0] and p[0, 1] < p[3, 1]
    assert 0 < p[:, 0].min() and p[:, 0].max() < W and 0 < p[:, 1].min() and p[:, 1].max() < H


def test_detector_works_on_this_opencv(scene):
    marker, _, _, detector, _ = scene
    image = np.full((200, 200), 255, np.uint8)
    image[40:160, 40:160] = ad.marker_image(ad.aruco_dictionary('DICT_4X4_50'), marker.marker_id, 120)
    found = detector.detect(image)
    assert list(found) == [marker.marker_id]
    assert np.allclose(found[marker.marker_id][0], [39.5, 39.5], atol=0.6)     # 왼위 모서리 (화소 경계)


def test_docked_pose_is_sub_millimetre(scene):
    truth = dock_in_base(DOCKED)
    corners, result = solve(scene, DOCKED)
    q, cov, rms = result['hypotheses'][0]
    assert len(result['hypotheses']) == 1                   # 가까우면 두 해가 한 자리로 모인다
    assert abs(q[0] - truth[0]) < 0.002 and abs(q[1] - truth[1]) < 0.001
    assert abs(math.degrees(ad.angle(q[2] - truth[2]))) < 0.2
    assert abs(lateral_error_at_docked(q, truth)) < 0.001
    # 공분산이 말하는 "붙은 자리의 옆 오차" (영상 하나, pixel_sigma 0.5 px 로 보수적으로). marker 하나로는 방향(yaw)이
    # 원근(사다리꼴)으로만 보여서 가장 약한 축이다: 0.47 m 지렛대(marker -> 붙은 자리)로 옆 오차가 된다.
    # 그래서 마지막에는 서서 여러 장을 합친다 (ArucoRefine: 30 장이면 1/5.5)
    g = np.array([-math.sin(q[2]), math.cos(q[2]), DOCKED[0]])
    assert math.sqrt(g @ cov @ g) < 0.007 and rms < 0.5
    assert math.degrees(result['tilt']) < 0.5 and abs(result['height']) < 0.003


def rot_x(a):
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def free_pose(scene, corners, camera):
    """비교용: 6자유도 PnP 에서 오차가 작은 해를 그대로 바닥에 내린 것"""
    R, t, _ = min(ad.ippe(corners, scene[0], camera), key=lambda s: s[2])
    return ad.dock_from_marker(R, t, scene[0], camera)


def errors(scene, robot, camera, frames=6):
    """프레임마다 (MAP, 6자유도 PnP) 의 |방향 오차| [rad], |붙은 자리의 옆 오차| [m] 평균"""
    truth = dock_in_base(robot)
    out = []
    for seed in range(frames):
        corners, result = solve(scene, robot, seed, camera)
        for q in (result['hypotheses'][0][0], free_pose(scene, corners, camera or scene[4])):
            out.append((abs(ad.angle(q[2] - truth[2])), abs(lateral_error_at_docked(q, truth))))
    out = np.array(out).reshape(frames, 2, 2).mean(axis=0)
    return out[0], out[1]


def test_floor_prior_stops_the_flip_far_away(scene):
    # 1.3 m 정면: 6자유도 PnP 는 기울기와 방향이 섞여 몇 도 틀린다 (두 해가 비슷하다). 바닥 사전분포가 그것을 줄인다
    (map_yaw, map_lat), (free_yaw, free_lat) = errors(scene, (1.3, 0.0, math.pi), None, frames=8)
    assert map_yaw < 0.8 * free_yaw and map_lat < 0.8 * free_lat


def test_wrong_camera_extrinsics_do_not_break_it(scene):
    # 카메라가 TF 보다 1 cm 높거나, 1 도 숙여 있어도: "바닥 위 3자유도" 로 못박으면 옆으로 수 cm 틀리지만
    # (기울기 0, 높이 0 을 맞추려고 방향을 비튼다) 사전분포로 두면 영상이 이긴다
    low = ad.Camera(K, np.zeros(5), R_BASE_CAM, T_BASE_CAM - np.array([0.0, 0.0, 0.01]))
    pitched = ad.Camera(K, np.zeros(5), R_BASE_CAM @ rot_x(math.radians(1.0)), T_BASE_CAM)
    for camera in (low, pitched):
        for robot in (DOCKED, (0.70, 0.05, math.pi)):
            (map_yaw, map_lat), _ = errors(scene, robot, camera, frames=3)
            assert math.degrees(map_yaw) < 0.3 and map_lat < 0.003, (robot, math.degrees(map_yaw), map_lat)
    # 가까이서는 진단이 그 오차를 드러낸다 (dock 이 바닥 아래로 1 cm 가라앉은 것처럼, 1 도 기운 것처럼 보인다)
    _, result = solve(scene, DOCKED, camera=low)
    assert abs(result['height'] + 0.01) < 0.004
    _, result = solve(scene, DOCKED, camera=pitched)
    assert abs(math.degrees(result['tilt']) - 1.0) < 0.4


def test_covariance_is_honest_and_grows_with_distance(scene):
    robot = (1.0, 0.25, math.radians(195))                  # 비스듬히, 멀리
    truth = dock_in_base(robot)
    _, result = solve(scene, robot)
    q, cov, _ = result['hypotheses'][0]
    sigma = np.sqrt(np.diag(cov))
    error = np.array([q[0] - truth[0], q[1] - truth[1], ad.angle(q[2] - truth[2])])
    assert (np.abs(error) < 3 * sigma + np.array([0.002, 0.002, 0.002])).all()
    near = solve(scene, DOCKED)[1]['hypotheses'][0][1]
    assert cov[1, 1] > 10 * near[1, 1] and cov[2, 2] > 10 * near[2, 2]


def test_detects_the_big_marker_up_close_in_gazebo(scene):
    # Gazebo 에서 붙은 자리의 영상 (marker 한 변 222 px, 잘라서 JPEG). OpenCV 기본 minDistanceToBorder (3) 로는 못 찾았다
    image = cv2.imread(os.path.join(HERE, 'data', 'gazebo_docked.jpg'), cv2.IMREAD_GRAYSCALE)
    found = scene[3].detect(image)
    assert list(found) == [0]
    side = np.linalg.norm(found[0][1] - found[0][0])
    assert 215 < side < 230
