"""aruco_detector.py 의 계산(board, PnP, 공분산) 시험. 합성 영상: 판 그림을 알려진 카메라 자세로 영상에 그린다"""

import importlib.util
import math
import os

import cv2
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BOARDS = os.path.join(HERE, '..', 'config', 'aruco_boards.yaml')


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ad = load('aruco_detector', os.path.join(HERE, '..', 'scripts', 'aruco_detector.py'))
mk = load('make_aruco_dock', os.path.join(HERE, '..', 'tools', 'make_aruco_dock.py'))

W, H = 848, 480
F = W / 2 / math.tan(1.211 / 2)                  # D435 컬러 848 x 480, 수평 화각 69.4 도
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
DIST = np.zeros(5)
PPM = 3000.0


@pytest.fixture(scope='module')
def scene():
    dictionary, boards = ad.load_boards(BOARDS)
    with open(BOARDS) as f:
        cfg = yaml.safe_load(f)['boards']['dock_a']
    texture = mk.board_image(cfg, dictionary, PPM)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return boards['dock_a'], cfg['plate'], texture, cv2.aruco.ArucoDetector(dictionary, params)


def camera_in_dock(x, y, z, yaw):
    """dock 좌표계에서 (x, y, z) 에 있고 수평으로 yaw 쪽을 보는 카메라 -> (R_cam_D, t_cam_D)"""
    view = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    R_D_cam = np.column_stack([right, np.cross(view, right), view])
    return R_D_cam.T, -R_D_cam.T @ np.array([x, y, z])


def render_and_solve(scene, R_cam_D, t_cam_D, only=None, seed=0):
    board, plate, texture, detector = scene
    R_cam_B = R_cam_D @ ad.Board.R_D_B
    t_cam_B = R_cam_D @ np.array([-board.offset, 0.0, 0.0]) + t_cam_D
    S = np.array([[1 / PPM, 0, plate['y_min']], [0, -1 / PPM, plate['z_max']], [0, 0, 1.0]])
    homography = K @ np.column_stack([R_cam_B[:, 0], R_cam_B[:, 1], t_cam_B]) @ S
    image = cv2.warpPerspective(texture, homography, (W, H), flags=cv2.INTER_AREA, borderValue=110)
    rng = np.random.default_rng(seed)
    image = np.clip(cv2.GaussianBlur(image, (0, 0), 0.6) + rng.normal(0, 3, image.shape), 0, 255).astype(np.uint8)
    corners, ids, _ = detector.detectMarkers(image)
    found = {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.reshape(-1))}
    used = sorted(i for i in found if i in board.markers and (only is None or i in only))
    obj = np.vstack([board.corners(i) for i in used])
    img = np.vstack([found[i] for i in used])
    rvec, tvec, rms, ambiguity = ad.solve_board(obj, img, K, DIST, board.markers[used[0]][2] if len(used) == 1 else None)
    R_D, t_D = board.dock_from_board(cv2.Rodrigues(rvec)[0], tvec)
    cov = ad.pose_covariance(board, obj, rvec, tvec, K, DIST, max(0.5, rms))
    # 오차를 실제 dock 좌표계로: 위치 (x, y), 방향 (D 의 x 축이 바닥에서 돈 각)
    R_D_cam = R_cam_D.T
    error = R_D_cam @ (t_D - t_cam_D)
    x_axis = R_D_cam @ R_D[:, 0]
    return used, error, math.degrees(math.atan2(x_axis[1], x_axis[0])), cov, R_D_cam


def test_board_frames(scene):
    board = scene[0]
    c = board.corners(0)                          # 큰 marker: 가운데 0.10 m, 높이 0.155 m
    assert np.allclose(c[0], [-0.05, 0.205, 0.0]) and np.allclose(c[2], [0.05, 0.105, 0.0])
    # 판 바로 앞에서 정면으로 본 카메라: D 의 원점은 판에서 board_offset 앞
    R, t = board.dock_from_board(np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 0.5]))
    assert np.allclose(R[:, 0], [0, 0, -1])        # D 의 x (방 쪽) = 카메라 쪽 = 카메라 -z
    assert np.allclose(t, [0, 0, 0.5 - board.offset])


def test_docked_pose_is_sub_millimetre(scene):
    # 붙은 자리: 카메라가 접점 면 0.031 m 앞, 옆 0.047 m, 높이 0.127 m (시뮬레이션의 waffle)
    R, t = camera_in_dock(0.031, 0.047, 0.127, math.pi)
    used, error, yaw, cov, R_D_cam = render_and_solve(scene, R, t)
    assert used == [0, 1, 2, 3, 4]
    assert abs(error[0]) < 0.001 and abs(error[1]) < 0.001
    assert abs(yaw) < 0.1
    assert math.sqrt((R_D_cam @ cov[:3, :3] @ R_D_cam.T)[1, 1]) < 0.002


def test_oblique_view_and_covariance(scene):
    R, t = camera_in_dock(0.70, 0.25, 0.127, math.radians(195))
    used, error, yaw, cov, R_D_cam = render_and_solve(scene, R, t)
    assert len(used) == 5
    sigma = np.sqrt(np.diag(R_D_cam @ cov[:3, :3] @ R_D_cam.T))
    assert abs(error[0]) < 3 * sigma[0] + 0.001 and abs(error[1]) < 3 * sigma[1] + 0.001
    assert abs(yaw) < 1.5
    near = render_and_solve(scene, *camera_in_dock(0.031, 0.047, 0.127, math.pi))[3]
    assert near[1, 1] < cov[1, 1]                  # 가까울수록 공분산이 작다


def test_single_marker_is_honest_about_its_yaw(scene):
    # marker 하나를 정면에서 보면 방향이 몇 도 틀린다 (평면 정사각형 PnP 의 약점, QR 과 같다).
    # D 의 원점은 판에서 0.3 m 앞이라 그만큼 옆으로 틀리지만, 공분산이 그것을 말해 주어 필터가 덜 믿는다
    R, t = camera_in_dock(0.40, 0.0, 0.127, math.pi)
    used, error, yaw, cov, R_D_cam = render_and_solve(scene, R, t, only={0})
    assert used == [0]
    sigma_lat = math.sqrt((R_D_cam @ cov[:3, :3] @ R_D_cam.T)[1, 1])
    assert abs(error[1]) < 3 * sigma_lat
    board_lat = math.sqrt((R_D_cam @ render_and_solve(scene, R, t)[3][:3, :3] @ R_D_cam.T)[1, 1])
    assert sigma_lat > 3 * board_lat               # 판 전체를 볼 때보다 훨씬 덜 믿는다
