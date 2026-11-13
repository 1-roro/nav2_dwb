# ArUco marker 하나를 본 결과. aruco_detector.py 가 dock/aruco 로 발행하고, 도킹 BT 노드가 받는다 (DOCKING.md).
#
# marker 하나(정사각형 하나)의 PnP 는 두 해(IPPE)가 비슷한 오차를 내는 경우가 있다 (평면 자세의 뒤집힘).
# 그래서 두 해에서 각각 "dock 은 바닥에 서 있다" 는 사전분포를 둔 6자유도 MAP 으로 다시 맞추고, 다른 자리로
# 모이면 둘 다 후보로 보낸다. 고르는 것은 BT 의 필터가 한다 (지금까지의 추정에 가까운 쪽).

std_msgs/Header header              # stamp: 영상을 찍은 시각, frame_id: 로봇 base frame (base_footprint)
string dock                         # config/aruco_docks.yaml 의 dock 이름
int32 marker_id
float32 marker_px                   # 영상에서 marker 한 변의 길이 [px]
float32 camera_distance             # 카메라 ↔ marker 중심 [m]
DockHypothesis[] hypotheses         # MAP 비용이 작은 순. 두 해가 같은 자리로 모이면 1개
float32 ambiguity                   # 두 번째 / 첫 번째 후보의 MAP 비용 (클수록 확실). 0: 후보가 하나
float32 tilt_deg                    # 첫 후보의 dock 기울기 (바닥 평면에서) [deg]. 가까이서 늘 2 ~ 3 도를 넘으면
                                    # 카메라 TF 의 pitch / roll 이 틀렸다
float32 height_offset               # 첫 후보의 dock 바닥 높이 (base 의 z = 0 에서) [m]. 가까이서 늘 1 cm 를 넘으면
                                    # 카메라 높이, marker 높이(config), base frame 의 높이가 틀렸다
