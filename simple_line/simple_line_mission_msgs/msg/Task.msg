# 미션 BT (task_loop.xml) 에 보내는 작업 하나. 토픽: mission/task
#
# 로봇은 waypoints 를 차례로 지나 마지막 점(작업 지점)에서 멈춘다.
# 무엇을 할지는 action 으로 정하고, BT 가 action 을 보고 동작을 고른다.

string id                                 # 작업 이름 (로그와 mission/status 에 씀). 비우면 BT 가 붙인다
geometry_msgs/PoseStamped[] waypoints     # 경유점들 + 마지막이 작업 지점. 1개 이상, frame 은 보통 map

string action                             # 아래 중 하나. 그 외 값이면 이동만 한다
string ACTION_GO = "go"                   #   이동만
string ACTION_INSPECT = "inspect"         #   도착 후 정지 상태로 duration 초 동안 작업
string ACTION_SCAN = "scan"               #   가는 동안 0.5m 마다 스캔
string ACTION_LOOK_AROUND = "look_around" #   도착 후 제자리에서 한 바퀴 회전

float64 duration                          # inspect 의 작업 시간 [s]
