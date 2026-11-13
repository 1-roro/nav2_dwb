# ArUco marker 하나로 구한 dock 자세 후보 하나 (ArucoDetection.hypotheses). 로봇(base) 좌표계, 바닥 평면 위의 2D.
#
# dock 좌표계 D (config/aruco_docks.yaml): 원점 = 바닥에서 충전 접점 면의 가운데, x = dock 에서 방 쪽, z = 위.
# (x, y, yaw) = D 의 원점과 x 축 방향을 base 에서 본 것.

float64 x
float64 y
float64 yaw
float64[9] covariance           # (x, y, yaw) 3x3, 행 우선. 모서리 잡음과 바닥 사전분포의 정보 행렬 (JᵀJ)⁻¹ 에서
float64 reprojection_error      # 이 자세로 모서리 4개를 다시 투영했을 때의 RMS [px]
