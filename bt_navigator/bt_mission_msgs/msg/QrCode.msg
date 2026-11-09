# 카메라 영상에서 찾은 QR 하나. qr_detector.py 가 dock/qr 로 발행하고, docking BT 노드가 받는다
#
# pose 는 header.frame_id (카메라 optical frame: x 오른쪽, y 아래, z 앞)에서 본 QR 의 중심과 방향이다.
#   QR 좌표계 (OpenCV ArUco 마커와 같다): 원점 = QR 중심, x = 인쇄된 QR 을 바로 봤을 때 오른쪽, y = 위,
#   z = QR 면에서 앞으로(로봇이 다가오는 쪽으로) 나오는 법선.

std_msgs/Header header       # stamp: 영상을 찍은 시각, frame_id: 카메라 optical frame
string text                  # QR 내용
geometry_msgs/Pose pose
float32 size                 # pose 를 계산할 때 쓴 QR 한 변 [m] (바깥 finder pattern 모서리 사이, 여백 제외)
float32 pixel_size           # 영상에서 QR 한 변의 평균 길이 [px]. 작을수록 pose 가 덜 정확하다
float32 reprojection_error   # PnP 재투영 오차의 평균 [px]

# 방향(QR 면의 법선)을 어디서 구했나
uint8 NORMAL_PNP = 0         # QR 네 모서리(PnP)만으로. 멀리서 작게 보이면 법선이 시선 둘레로 뒤집혀 10~20 도 틀리기도 한다
uint8 NORMAL_DEPTH = 1       # 깊이 영상(RealSense)에서 QR 안의 점들에 평면을 맞춰서. 위치는 PnP 그대로
uint8 normal_source
float32 plane_rms            # NORMAL_DEPTH 일 때 평면에서 점들이 떨어진 거리의 RMS [m]
