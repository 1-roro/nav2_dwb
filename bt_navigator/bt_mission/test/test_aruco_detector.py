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
DOCKS = os.path.join(HERE, '..', 'config', 'docking.yaml')


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ad = load('aruco_detector', os.path.join(HERE, '..', 'scripts', 'aruco_detector.py'))

# RealSense D455 컬러를 1280 x 720 으로: 가로 화각 약 90 도 (fx = 640 px), 세로 약 59 도
W, H = 1280, 720
F = W / 2 / math.tan(math.radians(90.0) / 2)
K = np.array([[F, 0, (W - 1) / 2], [0, F, (H - 1) / 2], [0, 0, 1.0]])
# 시뮬레이션의 메카넘 (robots/make_mecanum.py): 카메라가 base 앞 0.15, 가운데, 높이 0.13 m. optical: x 오른쪽, y 아래, z 앞
R_BASE_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BASE_CAM = np.array([0.15, 0.0, 0.13])
PPM = 6000.0                                      # marker 그림의 해상도 [px/m]
SS = 4                                            # 크게 그리는 배수
QUIET = 0.02                                      # 흰 여백 [m]
# 붙은 자리: 로봇 앞면이 설비(marker 면) 0.15 m 앞 = 중심은 0.31 m 앞. 카메라 <-> marker 0.16 m
DOCKED = (0.31, 0.0, math.pi)
DICTIONARY, ALL_DOCKS = ad.load_docks(DOCKS)
_plates = {}


def plate(dock):
    """dock 의 marker 를 모두 담은 흰 판의 그림 (한 장에 인쇄한 것과 같다) -> (그림, 왼쪽 끝 y_D, 위쪽 끝 z_D)"""
    if dock.name not in _plates:
        y0 = min(m.lateral - m.size / 2 for m in dock.markers) - QUIET
        y1 = max(m.lateral + m.size / 2 for m in dock.markers) + QUIET
        z0 = min(m.height - m.size / 2 for m in dock.markers) - QUIET
        z1 = max(m.height + m.size / 2 for m in dock.markers) + QUIET
        texture = np.full((int(round((z1 - z0) * PPM)), int(round((y1 - y0) * PPM))), 255, np.uint8)
        for m in dock.markers:
            side = int(round(m.size * PPM))
            col, row = int(round((m.lateral - m.size / 2 - y0) * PPM)), int(round((z1 - m.height - m.size / 2) * PPM))
            texture[row:row + side, col:col + side] = ad.marker_image(
                ad.aruco_dictionary(DICTIONARY), m.marker_id, side)
        _plates[dock.name] = (texture, y0, z1)
    return _plates[dock.name]


@pytest.fixture(scope='module')
def scene():
    """marker 하나짜리 dock_a: (marker, dock, -, detector, camera)"""
    dock = ALL_DOCKS['dock_a']
    camera = ad.Camera(K, np.zeros(5), R_BASE_CAM, T_BASE_CAM)
    return dock.markers[0], dock, None, ad.MarkerDetector(ad.aruco_dictionary(DICTIONARY)), camera


def dock_in_base(robot):
    """로봇 자세 (dock 좌표계에서 x, y, yaw) -> base 에서 본 dock (x, y, yaw)"""
    x, y, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([-(c * x + s * y), -(-s * x + c * y), ad.angle(-yaw)])


def render(scene, robot, seed=0, blur=0.6, noise=2.0, dock=None):
    """로봇이 robot (D 에서 x, y, yaw) 에 있을 때 카메라가 보는 dock 의 marker 판"""
    camera = scene[4]
    texture, y0, z1 = plate(dock or scene[1])
    offset = (dock or scene[1]).markers[0].offset
    q = dock_in_base(robot)
    R_cam_D = camera.R_cam_base @ ad.rot_z(q[2])
    t_cam_D = camera.R_cam_base @ np.array([q[0], q[1], 0.0]) + camera.t_cam_base
    # 그림의 화소 (u, v) (화소 중심 = 정수) -> D: y = y0 + (u + 0.5) / PPM, z = z1 - (v + 0.5) / PPM
    A = np.array([[0.0, 0.0, -offset], [1 / PPM, 0.0, y0 + 0.5 / PPM], [0.0, -1 / PPM, z1 - 0.5 / PPM]])
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


def solve_dock(scene, dock, robot, seed=0, only=None):
    """marker 가 여럿인 dock: 보인 marker 전부로 (only 가 있으면 그 번호만: 나머지가 가려진 경우)"""
    found = scene[3].detect(render(scene, robot, seed, dock=dock))
    observations = [(m, found[m.marker_id]) for m in dock.markers
                    if m.marker_id in found and (only is None or m.marker_id in only)
                    and ad.usable(found[m.marker_id], W, H, 16.0)[0]]
    return observations, ad.estimate_dock(observations, scene[4]) if observations else None


def lateral_error_at_docked(q, truth):
    """추정으로 붙으면 로봇이 옆으로 얼마나 어긋나나 [m]: 붙은 자리(D 에서 0.17 m 앞)를 두 자세로 base 에 놓고 비교"""
    def docked_point(p):
        return p[:2] + ad.rot_z(p[2])[:2, :2] @ np.array([DOCKED[0], 0.0])
    normal = np.array([-math.sin(truth[2]), math.cos(truth[2])])
    return float((docked_point(q) - docked_point(truth)) @ normal)


def test_marker_frame_and_projection(scene):
    marker, _, _, _, camera = scene
    # 모서리: 왼위가 -y_D (로봇이 dock 을 볼 때 왼쪽), 위
    assert np.allclose(marker.corners_D[0], [0.0, -0.03, 0.16])
    assert np.allclose(marker.corners_D[2], [0.0, 0.03, 0.10])
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
    # 0.75 m 정면 (marker 64 px): 6자유도 PnP 는 기울기와 방향이 섞여 2 도 가까이 틀린다. 바닥 사전분포가 그것을 줄인다
    (map_yaw, map_lat), (free_yaw, free_lat) = errors(scene, (0.75, 0.0, math.pi), None, frames=8)
    assert map_yaw < 0.8 * free_yaw and map_lat < 0.8 * free_lat


def test_wrong_camera_extrinsics_do_not_break_it(scene):
    # 카메라가 TF 보다 1 cm 높거나, 1 도 숙여 있어도: "바닥 위 3자유도" 로 못박으면 옆으로 수 cm 틀리지만
    # (기울기 0, 높이 0 을 맞추려고 방향을 비튼다) 사전분포로 두면 영상이 이긴다
    low = ad.Camera(K, np.zeros(5), R_BASE_CAM, T_BASE_CAM - np.array([0.0, 0.0, 0.01]))
    pitched = ad.Camera(K, np.zeros(5), R_BASE_CAM @ rot_x(math.radians(1.0)), T_BASE_CAM)
    for camera in (low, pitched):
        for robot in (DOCKED, (0.40, 0.02, math.pi)):
            (map_yaw, map_lat), _ = errors(scene, robot, camera, frames=3)
            assert math.degrees(map_yaw) < 0.3 and map_lat < 0.003, (robot, math.degrees(map_yaw), map_lat)
    # 가까이서는 진단이 그 오차를 드러낸다 (dock 이 바닥 아래로 1 cm 가라앉은 것처럼, 1 도 기운 것처럼 보인다)
    _, result = solve(scene, DOCKED, camera=low)
    assert abs(result['height'] + 0.01) < 0.004
    _, result = solve(scene, DOCKED, camera=pitched)
    assert abs(math.degrees(result['tilt']) - 1.0) < 0.4


def test_covariance_is_honest_and_grows_with_distance(scene):
    robot = (0.75, 0.10, math.radians(190))                 # 비스듬히, 멀리 (marker 60 px)
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


def test_border_distance_says_how_close_the_marker_is_to_leaving_the_image(scene):
    marker, _, _, _, camera = scene
    centred = ad.project_dock(dock_in_base(DOCKED), marker, camera)
    assert 225 < ad.border_distance(centred, W, H) < 255          # 붙은 자리: 위아래로 240 px (60 mm) 남는다
    # 옆으로 10 cm 벗어나면 화면 끝으로 간다 (0.16 m 앞에서 1 cm = 40 px. 좌우로는 13 cm 까지 남는다)
    shifted = ad.project_dock(dock_in_base((0.31, 0.10, math.pi)), marker, camera)
    assert ad.border_distance(shifted, W, H) < 140
    assert ad.border_distance(shifted, W, H) < ad.border_distance(centred, W, H)


def depth_image(scene, robot, noise=0.001, seed=0):
    """marker 가 붙은 면(dock 의 x = 0 평면)의 깊이 영상 [m]. 컬러와 같은 화각, 해상도는 절반 (시뮬레이션과 같다)"""
    _, _, _, _, camera = scene
    q = dock_in_base(robot)
    normal = camera.R_cam_base @ ad.rot_z(q[2])[:, 0]               # 평면의 법선 (카메라 좌표)
    point = camera.R_cam_base @ np.array([q[0], q[1], 0.0]) + camera.t_cam_base
    w, h = W // 2, H // 2
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.stack([(u - K[0, 2] / 2) / (K[0, 0] / 2), (v - K[1, 2] / 2) / (K[1, 1] / 2), np.ones((h, w))], axis=-1)
    z = (normal @ point) / (rays @ normal)
    rng = np.random.default_rng(seed)
    return (z + rng.normal(0, noise, z.shape)).astype(np.float32)


def test_depth_plane_gives_the_heading_where_the_marker_is_small(scene):
    # 0.75 m 앞 (marker 64 px): 모서리 넷으로는 방향이 0.5 도 안팎 틀리고 공분산은 몇 도다.
    # 면의 깊이 점들에 평면을 맞추면 0.3 도 안으로 재고, MAP 에 넣으면 방향 오차와 공분산이 같이 줄어든다
    marker, _, _, _, camera = scene
    robot = (0.75, 0.0, math.pi)
    truth = dock_in_base(robot)
    plain, fused = [], []
    for seed in range(6):
        corners, result = solve(scene, robot, seed)
        obs = ad.depth_plane_yaw(depth_image(scene, robot, seed=seed), corners, W, camera)
        assert obs is not None and abs(ad.angle(obs[0] - truth[2])) < math.radians(0.3)
        with_depth = ad.estimate(corners, marker, camera, yaw_obs=obs)
        assert with_depth['depth_used'] and not result['depth_used']
        plain.append((abs(ad.angle(result['hypotheses'][0][0][2] - truth[2])), result['hypotheses'][0][1][2, 2]))
        fused.append((abs(ad.angle(with_depth['hypotheses'][0][0][2] - truth[2])), with_depth['hypotheses'][0][1][2, 2]))
    plain, fused = np.mean(plain, axis=0), np.mean(fused, axis=0)
    # (정면에서는 모서리만으로도 0.1 도 안팎이 나올 때가 있어서 비율은 느슨하게 본다)
    assert fused[0] < 0.8 * plain[0] and math.degrees(fused[0]) < 0.2
    assert fused[1] < 0.05 * plain[1]
    # 붙은 자리 (0.16 m) 는 깊이 카메라가 못 재는 거리다: 버리고 영상만으로 한다
    corners, _ = solve(scene, DOCKED)
    assert ad.depth_plane_yaw(depth_image(scene, DOCKED), corners, W, camera, min_range=0.2) is None


# ---------------------------------------------------------------------------------------------
# marker 가 둘인 dock (docking.yaml 의 dock_b: 60 mm 둘, 중심 간격 160 mm)

def test_config_reads_one_or_several_markers_per_dock(tmp_path):
    assert ALL_DOCKS['dock_a'].marker_ids == [0]
    two = ALL_DOCKS['dock_b']
    assert two.marker_ids == [2, 3]
    assert [m.lateral for m in two.markers] == [-0.08, 0.08] and all(m.size == 0.06 for m in two.markers)
    # 왼쪽 marker 의 왼위 모서리: y = -0.08 - 0.03, z = 0.13 + 0.03
    assert np.allclose(two.markers[0].corners_D[0], [0.0, -0.11, 0.16])
    # marker 번호가 두 dock 에 겹치면 읽지 않는다 (어느 dock 인지 알 수 없다)
    bad = tmp_path / 'bad.yaml'
    bad.write_text('dictionary: DICT_4X4_50\ndocks:\n  a: {marker_id: 1, size: 0.06, height: 0.13}\n'
                   '  b: {size: 0.06, height: 0.13, markers: [{id: 1, lateral: -0.08}, {id: 2, lateral: 0.08}]}\n')
    with pytest.raises(ValueError):
        ad.load_docks(str(bad))


def test_two_markers_fit_in_the_view_at_the_docked_pose(scene):
    # 붙은 자리 (카메라 <-> 면 0.16 m) 에서 가로 시야는 0.32 m. marker 바깥 끝이 가운데에서 0.11 m 이므로 좌우 50 mm (200 px)
    camera, two = scene[4], ALL_DOCKS['dock_b']
    for m in two.markers:
        p = ad.project_dock(dock_in_base(DOCKED), m, camera)
        assert ad.border_distance(p, W, H) > 180
    # 옆으로 3 cm, 방향 2 도 틀어져도 둘 다 화면 안
    for m in two.markers:
        p = ad.project_dock(dock_in_base((0.31, 0.03, math.pi + math.radians(2.0))), m, camera)
        assert ad.border_distance(p, W, H) > 30
    observations, result = solve_dock(scene, two, DOCKED)
    assert len(observations) == 2 and abs(result['distance'] - 0.16) < 0.003      # 카메라 <-> marker 면


def test_two_markers_give_the_heading_one_cannot(scene):
    two = ALL_DOCKS['dock_b']
    rng = np.random.default_rng(1)
    for distance, limit_deg in ((0.75, 0.35), (1.0, 0.8)):
        single, double = [], []
        for seed in range(8):
            robot = (distance, rng.uniform(-0.03, 0.03), math.pi + rng.uniform(-0.12, 0.12))
            truth = dock_in_base(robot)
            _, one = solve(scene, robot, seed)
            observations, both = solve_dock(scene, two, robot, seed)
            assert len(observations) == 2
            assert len(both['hypotheses']) == 1                        # 뒤집힌 해가 남지 않는다
            single.append(ad.angle(one['hypotheses'][0][0][2] - truth[2]))
            double.append(ad.angle(both['hypotheses'][0][0][2] - truth[2]))
            # 공분산도 그만큼 작다고 말한다 (필터가 그만큼 믿는다)
            assert both['hypotheses'][0][1][2, 2] < 0.2 * one['hypotheses'][0][1][2, 2]
        rms = lambda v: math.degrees(math.sqrt(np.mean(np.square(v))))      # noqa: E731
        assert rms(double) < limit_deg and rms(double) < 0.5 * rms(single), (distance, rms(single), rms(double))


def test_one_of_two_markers_is_enough_to_keep_going(scene):
    # 하나가 가려지거나 화면 밖으로 나가도 남은 하나로 자세가 나온다 (marker 하나짜리 dock 과 같은 정밀도로)
    two = ALL_DOCKS['dock_b']
    for robot in (DOCKED, (0.56, 0.02, math.pi)):
        truth = dock_in_base(robot)
        for only in ([2], [3]):
            observations, result = solve_dock(scene, two, robot, only=only)
            assert len(observations) == 1
            q = result['hypotheses'][0][0]
            assert abs(lateral_error_at_docked(q, truth)) < 0.004 and abs(q[0] - truth[0]) < 0.003
    # 로봇이 옆으로 12 cm 비껴 있으면 한쪽 marker 는 화면 끝에 걸려 버려지고, 다른 하나로 푼다
    observations, result = solve_dock(scene, two, (0.31, 0.12, math.pi))
    assert len(observations) == 1 and result is not None
