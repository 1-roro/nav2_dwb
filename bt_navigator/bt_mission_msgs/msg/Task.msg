# 미션 BT 에 보내는 작업 하나. ExecuteTask 액션의 goal 에 담겨 task_server 로 간다 (task.xml)
#
# 로봇은 waypoints 의 점마다 멈춰서 위치와 방향을 맞춘 뒤(도착) 다음 점으로 간다.
# 마지막 점에 도착하면 action 에 따라 작업한다. BT 가 action 을 보고 동작을 고른다.

string id                                 # 작업 이름 (로그와 mission/status 에 씀). 비우면 BT 가 붙인다
geometry_msgs/PoseStamped[] waypoints     # 들를 점들. 1개 이상, frame 은 보통 map.
                                          # orientation 은 그 점에 도착했을 때 로봇이 볼 방향

string action                             # 아래 중 하나. 그 외 값(빈 값 포함)이면 이동만 한다
string ACTION_GO = "go"                   #   이동만
string ACTION_INSPECT = "inspect"         #   마지막 점에 도착한 뒤 정지 상태로 duration 초 동안 작업
string ACTION_SCAN = "scan"               #   이동하는 동안 0.5m 마다 스캔
string ACTION_LOOK_AROUND = "look_around" #   마지막 점에 도착한 뒤 제자리에서 한 바퀴 회전

float64 duration                          # inspect 의 작업 시간 [s]
