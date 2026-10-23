# 미션 BT 에 보내는 작업 하나. 토픽 mission/task (task_loop.xml), 또는 ExecuteTask 액션의 goal (task.xml)
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

string obstacle                           # 가는 길에 장애물이 있으면 어떻게 할지. 아래 중 하나, 빈 값이면 stop
string OBSTACLE_STOP = "stop"             #   멈춰서 기다렸다가, 길이 비면 다시 간다 (stop & go, 레이저 /scan 필요)
string OBSTACLE_AVOID = "avoid"           #   돌아서 간다 (장애물을 보는 planner / controller 와 navigate_avoid.xml)
string OBSTACLE_NONE = "none"             #   장애물을 보지 않는다 (레이저가 없을 때. 장애물에 부딪힐 수 있다)
