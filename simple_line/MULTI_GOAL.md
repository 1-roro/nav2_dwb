# 여러 goal 경유하기

`simple_line` 의 planner / controller 로, Nav2 명령 **하나**에 여러 goal 을 담아 도는 방법을 정리한다. 장애물은 고려하지 않는다.

| 하고 싶은 것 | 쓰는 Nav2 기능 | 실행 | 장 |
|---|---|---|---|
| 경유점은 멈추지 않고 지나 최종 goal 까지 | `NavigateThroughPoses` (bt_navigator) | `send_waypoints.py` | 1~4 |
| goal 마다 멈추고 방향까지 맞추며 차례로 | `FollowWaypoints` (waypoint_follower) | `follow_waypoints.py` | 5 |

경유, 정지 작업, 주행 중 작업, 실패 처리가 섞인 미션은 이 명령들을 상위 작업 계층(BT)에서 조합한다.
그 내용은 [`bt_navigator/`](../bt_navigator/README.md) 에 따로 정리했다. 작업을 실행 중에 받아 점마다 멈추는 BT 다.

목차

- [0. 빌드와 시뮬레이션 실행](#0-빌드와-시뮬레이션-실행)
- [1. 경유 경로: NavigateThroughPoses 가 처리되는 흐름](#1-경유-경로-navigatethroughposes-가-처리되는-흐름)
- [2. send_waypoints.py: 터미널에서 경유 경로 보내기](#2-send_waypointspy-터미널에서-경유-경로-보내기)
- [3. 꺾은선을 따라가기 위한 컨트롤러 동작](#3-꺾은선을-따라가기-위한-컨트롤러-동작)
- [4. 경유 경로의 동작 특성과 한계](#4-경유-경로의-동작-특성과-한계)
- [5. 멈춰 가며 경유하기: waypoint_follower](#5-멈춰-가며-경유하기-waypoint_follower)
- [6. 확인한 것](#6-확인한-것)
- [7. 파일](#7-파일)

---

## 0. 빌드와 시뮬레이션 실행

```bash
cd ~/nav2_ws/src/nav2_dwb && git pull
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select simple_line_planner simple_line_controller simple_line_bringup
source install/setup.bash
```

```bash
# 터미널 1: Gazebo + Nav2 + RViz (로봇은 (-2,-1) 에서 +x 를 보고 시작)
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch simple_line_bringup simple_line.launch.py
```

이후 장의 명령은 모두 **터미널 2** 에서 실행한다.

- 좌표는 모두 빈 방(6m x 6m, 벽은 ±3m) map frame 기준이다.
- waypoint_follower 와 behavior_server 는 이 launch 로 같이 뜬다.

---

## 1. 경유 경로: NavigateThroughPoses 가 처리되는 흐름

Nav2 에는 "pose 여러 개를 한 번에 받아 그 사이를 이어 가는" 액션이 이미 있다. bt_navigator 의 **`NavigateThroughPoses`** 다.
그래서 새 서버를 만들지 않고, 이 액션에 맞춰 각 요소가 제 역할을 하도록 구성했다.

```
send_waypoints.py ──(NavigateThroughPoses: poses[])──▶ bt_navigator
                                                         │  기본 BT: navigate_through_poses_w_replanning_and_recovery.xml
                                                         │   ├ 3초마다 (RateController 0.333Hz)
                                                         │   │   ├ RemovePassedGoals: 로봇 0.7m 안의 앞쪽 goal 삭제 (마지막 goal 은 남김)
                                                         │   │   └ ComputePathThroughPoses(남은 goals) ──▶ planner_server
                                                         │   └ FollowPath(path) ─────────────────────▶ controller_server
                                                         │
planner_server ── [로봇→g1], [g1→g2], ... 구간마다 GridBased(SimpleLinePlanner)::createPlan() 호출 후 이어 붙임
controller_server ── FollowPath(SimpleLineController) 가 꺾은선 경로를 추종, 끝점(최종 goal)에서만 도착 판정
```

| 단계 | 담당 | 이 저장소에서 |
|---|---|---|
| goal 여러 개를 한 번에 보내기 | `send_waypoints.py` (2장) | 새로 작성 |
| 지나간 goal 지우기, 주기적 재계획 | Nav2 기본 BT (1-1) | 그대로 사용 |
| 구간별 직선 → 하나의 꺾은선 | planner_server + `SimpleLinePlanner` (1-2) | 그대로 사용 (코드 변경 없음) |
| 꺾은선 따라가기 | `SimpleLineController` | 두 가지 동작을 넣었다 (3장) |

### 1-1. 재계획 주기(0.333Hz)와 지난 goal 반경(0.7m)은 어디에 있나

이 저장소의 코드에는 없다. **Nav2 를 설치할 때 같이 깔리는 기본 BT 파일**에 들어 있다.

bt_navigator 는 `default_nav_through_poses_bt_xml` 파라미터로 BT 파일을 고른다. `simple_line_params.yaml` 은 이 값을
정하지 않으므로, bt_navigator 가 기본값을 쓴다 (`nav2_bt_navigator/src/navigators/navigate_through_poses.cpp`):

```cpp
if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
  node->declare_parameter<std::string>("default_nav_through_poses_bt_xml",
    pkg_share_dir + "/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml");
}
```

그 파일이 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` 이고,
여기에 주기와 반경이 적혀 있다:

```xml
<RateController hz="0.333">                     <!-- 3초에 한 번만 아래를 실행 = 재계획 주기 -->
  ...
  <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
  <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
```

```bash
cat /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
```

goal 하나짜리 `NavigateToPose` 는 Nav2 기본으로는 `navigate_to_pose_w_replanning_and_recovery.xml` 을 쓰는데, 이 저장소의 launch 는 그것에서 회전, 후진 recovery 만 뺀 `simple_line_bringup/behavior_trees/navigate_to_pose_amr.xml` 로 바꿔 둔다 ([AMR.md 6장](AMR.md#6-막혔을-때-회전-후진-없이-기다리기)). 둘 다 주기는 `<RateController hz="1.0">` (1초) 이다. 5장의 waypoint_follower 도 내부에서 `NavigateToPose` 를 쓴다.

바꾸고 싶으면 이 XML 을 패키지로 복사해 수정한 뒤, `simple_line_params.yaml` 에 경로를 지정한다.

```yaml
bt_navigator:
  ros__parameters:
    default_nav_through_poses_bt_xml: "/절대/경로/my_through_poses.xml"
```

### 1-2. planner 는 왜 그대로 쓸 수 있나

Humble 의 `planner_server::computePlanThroughPoses()` 는 goal 목록을 돌면서 이렇게 한다.

```cpp
for (i = 0; i < goals.size(); ++i) {
  curr_start = (i == 0) ? 로봇 위치 : concat_path.poses.back();   // 앞 구간의 끝 = 이번 구간의 시작
  curr_goal  = goals[i];
  curr_path  = getPlan(curr_start, curr_goal, planner_id);        // = SimpleLinePlanner::createPlan()
  concat_path.poses.insert(end, curr_path.poses.begin(), curr_path.poses.end());
}
```

구간마다 플러그인의 `createPlan(start, goal)` 을 불러 이어 붙인다. 그래서 직선 하나만 그릴 줄 아는 `SimpleLinePlanner` 로도
경로 전체가 **꺾은선**이 된다. 구간 경계에는 같은 점이 두 번 들어가지만, 길이 0 인 구간이라 추종에는 영향이 없다.

---

## 2. send_waypoints.py: 터미널에서 경유 경로 보내기

`simple_line_bringup/scripts/send_waypoints.py`. 터미널 인자로 경유 경로를 바로 시험할 때 쓴다.

```bash
# 경유 goal 2개 + 최종 goal 1개 (map frame, x,y[,yaw_deg])
ros2 run simple_line_bringup send_waypoints.py 1.0,-1.0 1.0,1.5 -1.5,1.5,180
```

```
start (-2.00, -1.00)
  1. via   (1.00, -1.00, 90 deg)
  2. via   (1.00, 1.50, 180 deg)
  3. final (-1.50, 1.50, 180 deg)
  passed waypoint 1/3  (distance remaining 4.80 m)
  passed waypoint 2/3  (distance remaining 2.30 m)
result: SUCCEEDED, final xy error 0.24 m
```

| 기능 | 내용 |
|---|---|
| 입력 | `x,y` 또는 `x,y,yaw_deg` 를 순서대로 나열. 마지막이 최종 goal. `-1.5,0.5` 처럼 `-` 로 시작하는 좌표도 그대로 받는다 |
| yaw 자동 채움 | 중간 goal: **다음 goal 을 향하는 방향**, 최종 goal: **마지막 구간의 진행 방향** |
| 사전 검사 | 최종 goal 이 출발점이나 앞 구간과 goal 허용오차(+0.1m) 안으로 겹치면 거부 (4장, `--force` 로 무시) |
| 전송 | `NavigateThroughPoses` 한 번 |
| 진행 표시 | 피드백의 `number_of_poses_remaining` 이 줄어들 때마다 `passed waypoint k/n` |
| 결과 | `SUCCEEDED / ABORTED / CANCELED` 와 최종 위치 오차. Ctrl+C 로 취소 |

> 중간 goal 의 yaw 는 사실 주행에 쓰이지 않는다. 컨트롤러는 경로 점의 위치만 쓰고, 방향은 **최종 goal** 의 것만 쓴다.
> 그래도 RViz 에 자연스럽게 표시되도록 다음 goal 방향으로 채운다.

RViz 로도 할 수 있다.

1. Navigation 2 패널의 **Waypoint / Nav Through Poses Mode** 를 누른다.
2. Nav2 Goal 로 점을 여러 개 찍는다.
3. **Start Nav Through Poses** 를 누른다.

그러면 같은 `NavigateThroughPoses` 액션이 나간다. 다만 이 경우 위의 사전 검사는 없다.

---

## 3. 꺾은선을 따라가기 위한 컨트롤러 동작

`SimpleLineController` (`simple_line_controller/src/simple_line_controller.cpp`) 에는 꺾은선을 위한 동작이 두 가지 들어 있다.
직선 하나일 때는 필요 없지만, 여러 goal 을 이은 꺾은선에서는 이것이 없으면 문제가 생긴다.
아래 "없을 때" 는 이 두 동작을 넣기 전 컨트롤러로 잰 값이다. 직선 하나짜리 경로에서는 동작이 바뀌지 않는다 (6-1).

### 3-1. 가장 가까운 경로 점을 앞쪽 일정 거리 안에서만 찾기

컨트롤러는 매 주기 로봇에 가장 가까운 경로 점을 찾고, 그 앞부분(지나온 부분)을 잘라낸다.

**경로 전체에서 찾을 때의 문제**: 경로가 자기 자신과 교차하면, 교차점에 도착했을 때 **나중 구간**의 점이 더 가깝게 잡힌다. 그러면 그 사이 구간을 통째로 잘라 버린다.

```
 (-0.5,1.5) ◀──────── (1.5,1.5)
     │                    ▲
     │                    │
 ────┼────────────────────┘   ← 첫 구간 (y=-1) 을 달리다 x=-0.5 에서
 start                          마지막 구간 (x=-0.5) 의 점을 잡아버림
     │
     ▼ final (-0.5,-2.2)
```

이 동작이 없을 때 (교차 경로 실측): 로봇이 교차점에서 남쪽으로 **0.47m** 꺾였다. 3초 뒤 BT 재계획으로 경로가 다시 그려지고 나서야 돌아왔다.

**동작**: 남은 경로의 앞에서부터, 경로 길이로 `max_robot_pose_search_dist` 까지만 찾는다 (기본값: local costmap 반폭 1.5m).
RPP 의 같은 이름 파라미터와 같은 방식이다.

```cpp
double travelled = 0.0;
for (size_t i = 0; i < plan_.size(); ++i) {
  if (i > 0) {
    travelled += 거리(plan_[i-1], plan_[i]);
    if (travelled > search_dist) break;      // 경로를 따라 이만큼만 앞을 본다
  }
  ... 가장 가까운 점 갱신 ...
}
```

결과: 같은 경로에서 첫 구간 이탈이 **0.47m → 0.08m**, 소요 시간은 50.7s → 46.9s 가 됐다.

### 3-2. goal 까지 거리 = 남은 경로 길이

컨트롤러는 "goal 까지 거리" 를 두 곳에 쓴다.

- 이 거리가 goal 허용오차(xy_goal_tolerance, 0.05m) 이하이면 → 도착한 것으로 보고 제자리에서 goal yaw 로 회전
- 이 거리로 감속을 시작한다 (지금 브레이크를 걸면 서는 거리가 남은 거리에 닿을 때, [AMR.md 4-1](AMR.md#4-1-s-커브와-goal-앞-감속-includesimple_line_controllermotion_profilehpp))

**직선 거리로 잴 때의 문제**: 꺾은선에서는 최종 goal 이 **가까이 보여도 아직 돌아야 할 구간이 남아 있을 수 있다.**

- 출발점으로 돌아오는 경로: 출발하자마자 "도착" 으로 보고 goal yaw 로 회전한다. 방향까지 맞으니 Nav2 가 도착 처리한다.
  결과적으로 **로봇이 움직이지도 않고 6.5초 만에 SUCCEEDED** 가 됐다 (실측).
- 최종 goal 근처를 중간에 지나가는 경로: 그 근처에서 괜히 감속한다 (실측 최저 속도 0.188 m/s).

**동작**: 로봇 → 남은 경로의 첫 점 → … → 마지막 점까지의 **경로 길이**로 잰다.

```cpp
double goal_dist = 거리(로봇, plan_.front());
for (size_t i = 1; i < plan_.size(); ++i) goal_dist += 거리(plan_[i-1], plan_[i]);
```

직선 하나에서는 로봇이 경로 위에 있으므로 직선 거리와 같다. 꺾은선에서는 돌아야 할 구간이 모두 더해진다.

결과: 출발점으로 돌아오는 경로를 끝까지 돌고 도착했다 (33.9s). 최종 goal 근처를 지날 때도 감속하지 않았다 (0.250 m/s).

이 장의 수치(3-1, 3-2)는 제자리 회전 정렬과 goal 허용오차를 바꾸기 전([README](README.md) 의 컨트롤러 설명)에 잰 값이다.
두 동작이 있을 때와 없을 때의 차이를 보려는 것이라 그대로 두었다. 지금 설정의 결과는 6-1 에 있다.

---

## 4. 경유 경로의 동작 특성과 한계

### 모서리에서

다음 구간이 꺾이면 lookahead(0.6m) 지점의 carrot 이 다음 구간으로 넘어간다.

- carrot 방향이 45°(`rotate_to_heading_min_angle`) 이하 → 호를 그리며 모서리를 돈다
- 45° 초과 (예: 90° 모서리) → 모서리 근처에서 멈추고, 다음 구간 쪽으로 **방향이 다 맞을 때까지** 제자리 회전한 뒤
  출발한다. 경유 goal 이라서 멈추는 것이 아니라 컨트롤러 동작이다
  (실측 1.2초)

### 경유 goal 을 정확히 밟지는 않는다

BT 의 `RemovePassedGoals` 는 로봇이 goal **0.7m 안**에 들어오면 그 goal 을 지운다. 지운 뒤 다음 재계획(최대 3초 뒤)에서는
**그 순간의 로봇 위치 → 다음 goal** 직선이 새로 그려지므로, 모서리를 가로질러 간다.

- 실측으로 경유 goal 을 0.23~0.61m 거리로 지나갔다 (재계획 타이밍에 따라 달라짐).
- 더 가깝게 지나가야 하면 `RemovePassedGoals` 의 `radius` 를 줄인 BT 를 만들어 `default_nav_through_poses_bt_xml` 로 지정한다 (1-1).

### 최종 goal 이 앞 구간과 겹치면 일찍 끝날 수 있다 (Humble)

controller_server 의 도착 판정(`SimpleGoalChecker`)은 남은 경로와 상관없이 **로봇과 경로 끝점(= 최종 goal)의 위치·방향 차이**만 본다.

- 최종 goal 이 출발점이나 앞 구간과 xy 허용오차(0.05m) 안으로 겹친다.
- 그 지점에서 로봇의 **방향까지** 최종 goal 과 맞는다.

두 조건이 맞으면 중간 goal 을 다 돌기 전에 SUCCEEDED 가 된다. 컨트롤러 쪽에서는 막을 수 없는 Nav2 Humble 의 동작이다.

| 경로 (출발점 = 최종 goal 위치) | 최종 goal yaw | 결과 |
|---|---|---|
| (1,-1) → (1,1) → (-2,-1) | -146° (출발 방향 0° 와 다름) | 끝까지 돌고 SUCCEEDED (33.9s) |
| (1,-1) → (1,1) → (-2,-1) | 0° (출발 방향과 같음) | **움직이지 않고 1.9초 만에 SUCCEEDED** |

`send_waypoints.py` 는 위치가 겹치기만 해도 미리 거부한다 (방향은 실행 중에 정해지므로). 피하는 방법은 셋이다.

- 최종 goal 을 조금 옮긴다.
- 경로를 두 번에 나눠 보낸다.
- 위험을 알고 `--force` 로 보낸다.

각 goal 에서 멈추고 방향까지 맞춘 뒤 다음 goal 로 가는 방식(5장)은 goal 마다 따로 판정하므로 이 문제가 없다.

---

## 5. 멈춰 가며 경유하기: waypoint_follower

`simple_line_bringup/scripts/follow_waypoints.py`. 목표 위치를 **파이썬 코드 안에서** 정하고 실행한다.
`FollowWaypoints` 액션(waypoint_follower)을 쓰며, goal 마다 멈추고 방향까지 맞춘 뒤 다음 goal 로 간다.

### 사용법

1. 파일 위쪽의 `WAYPOINTS` 를 고친다.

   ```python
   WAYPOINTS = [
       (1.0, -1.0, None),     # (x [m], y [m], yaw [deg]), map frame
       (1.0, 1.5, None),
       (-1.5, 1.5, None),
       (-2.0, -1.0, 0.0),     # 출발점으로 돌아와 +x 방향으로 정렬
   ]
   ```

   yaw 를 `None` 으로 두면 중간 goal 은 **다음 goal 을 향하는 방향**, 마지막 goal 은 **직전 지점에서 온 방향**으로 채운다.
   waypoint_follower 는 goal 마다 yaw 까지 맞추므로, 이렇게 채워 두면 도착 후 제자리 회전이 줄어든다.

2. 실행한다.

   ```bash
   # WAYPOINTS 를 고친 뒤 빌드 없이 바로 실행
   python3 ~/nav2_ws/src/nav2_dwb/simple_line/simple_line_bringup/scripts/follow_waypoints.py

   # 또는 설치된 것으로 실행 (colcon build --symlink-install 이면 원본을 고친 게 바로 반영된다)
   ros2 run simple_line_bringup follow_waypoints.py
   ```

```
start (-2.00, -1.00), 4 waypoints
  [1/4] heading to (1.00, -1.00, 90 deg)
  [1/4] done in 18.7 s
  [2/4] heading to (1.00, 1.50, 180 deg)
  [2/4] done in 15.8 s
  [3/4] heading to (-1.50, 1.50, -101 deg)
  [3/4] done in 15.6 s
  [4/4] heading to (-2.00, -1.00, 0 deg)
  [4/4] done in 16.0 s
result: SUCCEEDED in 65.9 s, missed waypoints: none
final error: xy 0.04 m, yaw 0.7 deg
```

### 명령이 C++ planner / controller 까지 가는 길

```
follow_waypoints.py ──(FollowWaypoints: poses[4])──▶ waypoint_follower
   waypoint_follower 가 goal 을 하나씩 꺼내 순서대로
   ──(NavigateToPose: pose[i])──▶ bt_navigator   (navigate_to_pose_amr.xml, 1초마다 재계획)
        ├ ComputePathToPose ──▶ planner_server    → SimpleLinePlanner::createPlan(로봇, goal[i])  : 직선 한 개
        └ FollowPath        ──▶ controller_server → SimpleLineController::computeVelocityCommands : 20Hz 로 cmd_vel
   goal[i] 도착 (위치 0.05m + yaw 0.05rad) → WaitAtWaypoint 0.2초 → goal[i+1] 로
```

실측 로그(헤드리스 테스트)도 이 순서 그대로다:

```
[waypoint_follower]: Received follow waypoint request with 4 waypoints.
[bt_navigator]: Begin navigating from current location (-2.00, -1.00) to (1.00, -1.00)
[controller_server]: Reached the goal!
[bt_navigator]: Goal succeeded
[waypoint_follower]: Arrived at 0'th waypoint, sleeping for 200 milliseconds
[waypoint_follower]: Handled task execution on waypoint 0, moving to next.
[bt_navigator]: Begin navigating from current location (0.79, -1.00) to (1.00, 1.50)
...
```

goal 하나하나는 `simple_line` 의 단일 goal 과 똑같이 처리된다. 그래서 이 방법을 위해 planner / controller 를 바꿀 필요가 없다.

### 스크립트가 하는 일

| 단계 | 코드 |
|---|---|
| 로봇 현재 위치 읽기 (TF `map → base_link`) | `robot_pose()` |
| `WAYPOINTS` 의 빈 yaw 채우기 | `fill_yaws()` |
| `FollowWaypoints` goal 만들기 (`PoseStamped` 목록, frame `map`) | `make_goal()` |
| 액션 전송 / 결과 기다리기 / Ctrl+C 시 취소 | `run()` |
| 진행 표시: feedback 의 `current_waypoint` 가 바뀔 때마다 출력 | `on_feedback()` |
| 결과: 상태, 실패한 goal 목록(`missed_waypoints`), 최종 위치 오차 | `run()` |

### waypoint_follower 설정 (`simple_line_params.yaml`)

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20                  # 진행 상태 확인 주기 [Hz]
    stop_on_failure: false         # goal 하나가 실패해도 다음 goal 로 계속 (실패한 index 는 missed_waypoints 로)
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200 # 각 goal 에 도착한 뒤 기다리는 시간 [ms]
```

값은 Humble 기본값과 같다. 바꿀 곳이 보이도록 명시해 두었다.
`enabled: false` 는 대기만 없앤다. goal 마다 멈추는 것은 그대로다 (같은 직선 위 goal 4개로 확인: 매 goal 에서 약 0.8초 정지).

### NavigateThroughPoses 와 비교

| | NavigateThroughPoses (1~4장) | FollowWaypoints (5장) |
|---|---|---|
| 경로 | 전체 goal 을 이은 꺾은선 하나 | goal 마다 직선 하나씩 |
| 중간 goal 에서 | 멈추지 않고 통과 (0.7m 안이면 지난 것으로 처리) | 멈추고 위치·yaw 를 맞춘 뒤 0.2초 대기 |
| 중간 goal 의 yaw | 쓰이지 않음 | 쓰임 |
| 재계획 주기 (기본 BT) | 3초 (0.333Hz) | 1초 (1Hz) |
| 최종 goal 이 출발점·앞 구간과 겹칠 때 | 일찍 끝날 수 있음 (4장) | 문제없음 |
| 중간 goal 하나가 실패하면 | 전체 실패 | 건너뛰고 계속 (`stop_on_failure: false`) |
| 같은 사각형 경로 소요 시간 | 35.7 s (경유 2 + 최종 1) | 65.9 s (goal 4개, 출발점 복귀 포함) |

---

## 6. 확인한 것

모두 같은 환경에서 확인했다.

- **서버**: 실제 Nav2 Humble 서버(map_server, planner_server, controller_server, behavior_server, bt_navigator, velocity_smoother, waypoint_follower). Nav2 기본 BT, 빈 방 맵.
- **로봇**: Gazebo 대신 `cmd_vel` 을 적분하는 가짜 로봇. AMCL 은 쓰지 않았다.
- **출발점**: 모두 (-2.0, -1.0, 0°).

Gazebo 자체(월드, 스폰, AMCL)에서는 돌려 보지 못했다.

### 6-1. 경유 경로 (NavigateThroughPoses, `send_waypoints.py`)

지금 설정(제자리 회전은 방향이 다 맞을 때까지, goal 허용오차 0.05m / 0.05rad)으로 잰 결과:

| 경로 (경유 → … → 최종) | 결과 | 최종 오차 | 경유점에 가장 가까이 간 거리 |
|---|---|---|---|
| 사각형: (1,-1) → (1,1.5) → (-1.5,1.5,180°) | SUCCEEDED 35.7s | 0.05 m | 0.23, 0.23 m |
| 교차: (1.5,-1) → (1.5,1.5) → (-0.5,1.5) → (-0.5,-2.2,-90°) | SUCCEEDED 51.3s | 0.03 m | 0.50, 0.40, 0.32 m |
| 지그재그: (-1,0) → (0,-1.5) → (1,0) → (2,-1.5) | SUCCEEDED 32.5s | 0.03 m | 0.46, 0.51, 0.25 m |
| 헤어핀: (1.5,-1) → (1.5,-0.5) → (-1.5,-0.5,180°) | SUCCEEDED 31.2s | 0.03 m | 0.36, 0.58 m |
| 최종 goal 근처 통과: (1,-1) → (1,0) → (-1,-0.55,180°) | SUCCEEDED 28.9s | 0.05 m | 0.23, 0.47 m |
| 출발점 복귀, yaw 다름: (1,-1) → (1,1) → (-2,-1) (`--force`) | 끝까지 돌고 SUCCEEDED 37.6s | 0.04 m | 0.23, 0.58 m |
| 출발점 복귀, yaw 같음: 위와 같고 최종 yaw 0° (`--force`) | 움직이지 않고 1.8s 만에 SUCCEEDED (Humble 한계, 4장) | | |

경유점은 여전히 0.23~0.58m 거리로 지나간다 (`RemovePassedGoals` 반경 0.7m, 4장). 점마다 정확히 서야 하면
`waypoint_follower`(5장)나, 점마다 `NavigateToPose` 를 보내는 BT([bt_navigator](../bt_navigator/README.md))를 쓴다.

아래는 3장의 두 동작을 넣기 전과 비교한 것으로, 제자리 회전 정렬과 허용오차를 바꾸기 전에 쟀다.
"없을 때" 는 3장의 두 동작이 없는 컨트롤러다.

| 경로 (경유 → … → 최종) | 3장 동작 있음 | 없을 때 |
|---|---|---|
| 사각형: (1,-1) → (1,1.5) → (-1.5,1.5,180°) | SUCCEEDED 32.0s | 32.1s |
| 교차: (1.5,-1) → (1.5,1.5) → (-0.5,1.5) → (-0.5,-2.2,-90°) | SUCCEEDED 46.9s, 교차점 이탈 0.08m | 50.7s, 교차점 이탈 0.47m |
| 지그재그: (-1,0) → (0,-1.5) → (1,0) → (2,-1.5) | SUCCEEDED 24.5s | |
| 헤어핀: (1.5,-1) → (1.5,-0.5) → (-1.5,-0.5,180°) | SUCCEEDED 26.7s | |
| 최종 goal 근처 통과: (1,-1) → (1,0) → (-1,-0.55,180°) | SUCCEEDED 24.8s, 근처 통과 속도 0.250 m/s | 26.2s, 0.188 m/s |
| 출발점 복귀, yaw 다름: (1,-1) → (1,1) → (-2,-1) (`--force`) | 끝까지 돌고 SUCCEEDED 33.9s | **움직이지 않고 6.5s 만에 SUCCEEDED** |
| 출발점 복귀, yaw 같음: 위와 같고 최종 yaw 0° (`--force`) | 움직이지 않고 1.9s 만에 SUCCEEDED (Humble 한계, 4장) | |

단일 goal 7개(정면, 등 뒤, 대각선, yaw 90° 차이, 0.15m, 제자리 회전, 스폰 복귀)도 3장 동작을 넣은 뒤 다시 돌렸다. 모두 SUCCEEDED, 소요 시간 차이는 1초 이내였다. 지금 설정의 단일 goal 결과는 [README](README.md#확인한-것) 에 있다.

### 6-2. waypoint_follower (`follow_waypoints.py`)

기본 `WAYPOINTS` (goal 4개, 출발점으로 같은 yaw 로 복귀)를 `ros2 run` 과 `python3` 직접 실행으로 한 번씩 돌렸다.

- 둘 다 SUCCEEDED, missed waypoint 없음.
- 지금 설정: 65.9 s, 최종 오차 xy 0.04 m, yaw 0.7°.
  (회전 정렬과 허용오차를 바꾸기 전: 54.0 s / 53.2 s, 최종 오차 xy 0.21 / 0.23 m, yaw 1.8 / 5.9°)
- NavigateThroughPoses 에서 1.9초 만에 끝나 버리던 "출발점 복귀 + 같은 yaw" 경우도 끝까지 돈다.

---

## 7. 파일

```
simple_line/
├── simple_line_controller/
│   └── src/simple_line_controller.cpp       3장의 두 동작 (max_robot_pose_search_dist, 남은 경로 길이)
└── simple_line_bringup/
    ├── params/simple_line_params.yaml       max_robot_pose_search_dist, waypoint_follower 설정 (5장)
    └── scripts/
        ├── send_waypoints.py                2장: NavigateThroughPoses 를 터미널에서
        └── follow_waypoints.py              5장: FollowWaypoints, 위치는 파일 안의 WAYPOINTS
```
