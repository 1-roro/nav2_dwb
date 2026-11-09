# ArUco dock board 하나를 본 결과. aruco_detector.py 가 dock/aruco 로 발행하고, ArUco 도킹 BT 노드가 받는다
# (ARUCO_DOCKING.md). 판 위의 marker 여러 개의 모서리를 한꺼번에 PnP 로 풀어 dock 좌표계의 자세를 낸다.
#
# dock 좌표계 D (config/aruco_boards.yaml): 원점 = 바닥에서 충전 접점 면의 가운데, x = dock 에서 방 쪽, z = 위.

std_msgs/Header header                    # stamp: 영상을 찍은 시각, frame_id: 카메라 optical frame
string board                              # board(dock) 이름
geometry_msgs/PoseWithCovariance pose     # 카메라 optical frame 에서 본 D 의 원점과 방향.
                                          # covariance: [x y z rx ry rz] (카메라 frame, 회전은 카메라 축 둘레의 작은 각)
                                          # 모서리 위치 잡음(pixel_sigma)을 PnP 의 Jacobian 으로 옮긴 것
int32[] marker_ids                        # 이번 풀이에 쓴 marker
float32 reprojection_error                # 모서리 재투영 오차 RMS [px]
float32 ambiguity                         # 평면 PnP 의 두 번째 해 재투영 오차 / 첫 번째 해 (클수록 확실). 0: 해가 하나
uint8 NORMAL_PNP = 0
uint8 NORMAL_DEPTH = 1
uint8 normal_source                       # 판의 기울기를 어디서 구했나
float32 plane_rms                         # NORMAL_DEPTH 일 때 깊이 점들의 평면 잔차 RMS [m]
