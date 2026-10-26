# simple_line: 직선 경로 planner + controller (ROS 2 Humble, C++)

장애물을 돌아가지 않는 가장 단순한 형태의 Nav2 플러그인 두 개와 실행 설정이다.
길에 장애물이 있으면 Nav2 의 collision_monitor 가 로봇을 줄이고 세웠다가, 치우면 다시 보낸다 (아래 "장애물 앞에서 멈췄다 가기").

실제 AMR 로 옮기는 데 필요한 것(지도 만들기와 AMCL, 부드러운 가감속, 장애물 감속, 실제 로봇 개발 방식)은
[AMR.md](AMR.md) 에 정리했다.

```
simple_line/
├── AMR.md                    실제 AMR 로 옮기기 (지도, AMCL, 가감속, 장애물 감속, 개발 방식)
├── simple_line_planner/      nav2_core::GlobalPlanner  — start → goal 직선 경로 (지도의 장애물을 지나면 거절)
├── simple_line_controller/   nav2_core::Controller     — pure pursuit + 제자리 회전 + S 커브 가감속
└── simple_line_bringup/      launch(시뮬레이션 / 내비게이션 / RViz), 파라미터, 빈 방(6m x 6m)과 창고(10m x 8m) 월드와 지도,
                              navigation BT, 여러 goal 경유 스크립트, 장애물 도구
```

## Planner — `simple_line_planner::SimpleLinePlanner`

`src/simple_line_planner.cpp` 의 `createPlan()`.

1. start 와 goal 사이를 `interpolation_resolution` 간격으로 나눈다.
   점 개수는 `ceil(거리 / resolution)` 이라 간격이 resolution 을 넘지 않는다.
2. 중간 점의 orientation 은 모두 직선의 진행 방향 `atan2(dy, dx)` 이다.
3. 마지막 점은 goal 그대로다 (goal 의 orientation 유지).
4. `fail_on_obstacle` 이면 직선이 global costmap 에서 로봇이 닿는 칸(cost 253 이상, 로봇 반지름 안)을 지날 때
   경로를 만들지 않는다 (`the line crosses an obstacle in the map at (x, y)`). 장애물을 돌아가지는 않는다.
   로봇이 이미 그런 칸에 서 있으면 출발 쪽의 그 칸들은 빼고 본다. global costmap 은 지도 층만 쓰므로
   지도에 없는 장애물(사람)은 여기서 보지 않는다 ([AMR.md 3장](AMR.md#3-지도-위에서-점-명령-주기)).

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `interpolation_resolution` | 0.1 | 점 간격 [m] |
| `fail_on_obstacle` | true | 직선이 지도의 장애물을 지나면 실패 |
| `allow_unknown` | true | 지도에서 모르는 칸(회색)과 지도 밖은 지나가도 된다 |

## Controller — `simple_line_controller::SimpleLineController`

`src/simple_line_controller.cpp` 의 `computeVelocityCommands()`. 매 주기(기본 20Hz):

1. **경로 정리**: 로봇 pose 를 경로 frame(map)으로 옮기고, 로봇에 가장 가까운 점 앞부분을 잘라낸다.
   가장 가까운 점은 경로 앞쪽 `max_robot_pose_search_dist` 안에서만 찾는다 (꺾은선 경로가 자기 자신과
   교차할 때 나중 구간으로 건너뛰지 않도록).
   남은 경로 중 local costmap 범위(기본 3m x 3m → 반경 1.5m) 안의 점만 로봇 좌표계로 바꾼다.
2. **carrot 선택**: 로봇에서 `lookahead_dist` 이상 떨어진 첫 점. 없으면 마지막 점.
3. **명령 계산** (위에서부터 먼저 맞는 것):
   - goal 까지 남은 **경로 길이** ≤ goal checker 의 xy 허용오차 → 제자리에서 goal yaw 로 회전
   - 제자리 회전 중 → carrot 쪽으로 계속 회전. 방향 오차가 `rotate_to_heading_tolerance` 안으로 들어오고
     회전이 멈추면(`rotate_settled_angular_vel` 이하) 끝낸다
   - 그 외 → pure pursuit: 곡률 `k = 2·y / L²` (carrot 의 로봇 좌표 (x, y), `L² = x² + y²`), `w = v·k`

   제자리 회전은 이럴 때 시작한다.
   - 주행 중에 carrot 방향이 `rotate_to_heading_min_angle` 보다 크게 틀어졌을 때 (꺾인 경로)
   - **서 있다가 출발할 때** 방향이 `rotate_to_heading_tolerance` 보다 틀어져 있을 때

   한 번 시작한 회전은 방향이 다 맞을 때까지 계속한다. 덜 돈 채로 출발하면 남은 오차를 pure pursuit 가
   호를 그리며 줄이므로 경로가 휜다. 그래서 다 돈 뒤에 출발해서 직선으로 간다.
   회전 속도는 남은 각도에서 감속해 멈출 수 있는 속도 `sqrt(a·|θ|)` 를 넘지 않는다 (`a` = `max_angular_accel`).
   감속도를 한계의 절반(`a/2`)으로 계획해서, velocity_smoother 를 거치며 생기는 지연에도 목표 방향을 지나치지 않는다.
4. **직선 속도 프로파일** (`include/simple_line_controller/motion_profile.hpp`, [AMR.md 4장](AMR.md#4-가감속))
   - 가속 → 순항 → 감속. 가속도는 `max_linear_accel` / `max_linear_decel`, 가속도가 바뀌는 빠르기는
     `max_linear_jerk` 로 제한한다 (S 커브).
   - goal 앞 감속: 지금 브레이크를 걸면 `min_approach_linear_vel` 까지 줄이는 데 가는 거리가 도착 범위까지 남은
     거리에 닿으면 감속을 시작한다. 그 속도로 도착 범위에 들어가 선다.
   - 가감속은 바로 전 주기에 낸 명령에서 이어 간다 (odom 의 잡음을 타지 않는다). 새 goal 로 다시 시작할 때만
     로봇의 실제 속도에서 시작한다.
   - 달리다가 제자리 회전을 해야 하면(꺾인 경로) 먼저 곧게 감속해서 선 뒤 돈다.
5. **제한**
   - 제자리 회전 시 각가속도 `max_angular_accel`
   - `|w|` 가 `max_angular_vel` 을 넘으면 v 도 같은 비율로 줄여 곡률(지나가는 원호)을 유지
   - `setSpeedLimit()` (speed filter 등)이 건 속도 제한 반영

RViz 확인용 토픽: `/simple_line/local_plan` (잘라 낸 경로), `/simple_line/carrot` (lookahead 점).

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `desired_linear_vel` | 0.25 | 직진 속도 [m/s] |
| `max_angular_vel` | 1.0 | 각속도 상한 [rad/s] |
| `lookahead_dist` | 0.6 | carrot 거리 [m] |
| `rotate_to_heading_min_angle` | 0.785 | 이보다 틀어져 있으면 제자리 회전 [rad] |
| `rotate_to_heading_angular_vel` | 1.0 | 제자리 회전 속도 [rad/s] |
| `rotate_to_heading_tolerance` | 0.03 | 제자리 회전을 끝내는 방향 오차 [rad]. 서 있다가 출발할 때 이보다 틀어져 있으면 먼저 회전 |
| `rotate_settled_angular_vel` | 0.05 | 회전이 이 각속도 아래로 멈춘 뒤에 출발 [rad/s] |
| `max_linear_accel` | 0.5 | 가속도 [m/s²] |
| `max_linear_decel` | 0.5 | 감속도 [m/s²] |
| `max_linear_jerk` | 0.0 | 가속도가 바뀌는 빠르기 [m/s³]. 0 이하면 제한 없음 (사다리꼴). 이 저장소 설정은 1.0 |
| `max_angular_accel` | 3.2 | [rad/s²] |
| `min_approach_linear_vel` | 0.05 | goal 앞 감속의 하한. 이 속도로 도착 범위에 들어간다 [m/s] |
| `transform_tolerance` | 0.1 | TF 대기 시간 [s] |
| `max_robot_pose_search_dist` | -1.0 | 가장 가까운 경로 점을 찾을 범위 [m]. 0 이하면 local costmap 반폭 |

여러 goal 을 도는 방법:

- [MULTI_GOAL.md](MULTI_GOAL.md): 명령 하나로 경유하기 (`NavigateThroughPoses`, `waypoint_follower`), 꺾은선을 위한 컨트롤러 동작
- [`../bt_navigator/`](../bt_navigator/README.md): 작업(들를 점 + 정지 작업, 주행 중 작업)을 액션으로 받아 실행하는 BT.
  이 폴더의 planner / controller / launch 위에서 상위 작업 BT 가 Nav2 명령을 조합한다

## 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo

mkdir -p ~/nav2_ws/src && cd ~/nav2_ws/src
git clone https://github.com/1-roro/nav2_dwb.git
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select simple_line_planner simple_line_controller simple_line_bringup
source install/setup.bash
```

`--packages-select` 를 빼면 같은 저장소의 `straight_line/`, `bt_navigator/` 패키지들도 같이 빌드된다. 패키지 이름이 달라서 같이 빌드해도 충돌하지 않는다.

## 실행

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch simple_line_bringup simple_line.launch.py                        # 빈 방
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse       # 창고 (지도 maps/warehouse)
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse slam:=True   # 창고 지도를 새로 만들기
```

- Gazebo + TurtleBot3 waffle + Nav2 + RViz 가 뜬다. 빈 방이면 로봇은 (-2.0, -1.0), 창고면 (-4.0, -2.5) 에서
  +x 를 보고 시작하고, AMCL 초기 자세도 같은 값으로 준다.
- RViz 의 **Nav2 Goal** 로 목표를 찍으면 된다. 1초마다 현재 위치에서 직선을 다시 긋는다.
- 터미널에서 보낼 때:
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}}"
  ```

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `world` | `empty_room` | `empty_room`, `warehouse`, 또는 `.world` 파일 경로 |
| `map` | `maps/<world>.yaml` | 지도 yaml |
| `slam` | False | True: 지도를 새로 만든다 (slam_toolbox). 저장은 `ros2 run nav2_map_server map_saver_cli -f <이름>` |
| `x_pose` / `y_pose` / `yaw` | 월드마다 | 스폰 위치 (AMCL 초기 자세도 같이 바뀜) |
| `headless` / `use_rviz` | False / True | |

launch 는 두 덩어리다 ([AMR.md 1장](AMR.md#1-launch-구조)).

- `simple_line.launch.py`: 시뮬레이션 로봇(Gazebo, 스폰, robot_state_publisher)을 띄우고 아래 둘을 부른다.
- `navigation.launch.py`: 로봇과 무관한 내비게이션. SLAM 또는 AMCL, planner / controller / behavior / bt_navigator,
  collision_monitor, velocity_smoother. 실제 로봇에서도 이 파일을 그대로 쓴다.
  설치된 Humble 기본 `nav2_params.yaml` 위에 `params/simple_line_params.yaml` 을 덮어쓴다.
- `rviz.launch.py`: Nav2 기본 RViz 화면에 collision_monitor 의 SlowZone(주황), StopZone(빨강)을 더한다.

플러그인 이름은 BT 가 부르는 `GridBased` / `FollowPath` 그대로다. navigation BT 는 Nav2 기본 트리에서 회전, 후진
recovery 만 뺀 `behavior_trees/navigate_to_pose_amr.xml` 을 쓴다 ([AMR.md 6장](AMR.md#6-막혔을-때-회전-후진-없이-기다리기)).

goal 에 도착했다고 보는 범위(goal checker)도 줄였다. Humble 기본값(xy 0.25m / yaw 0.25rad)이면 로봇이 goal
20cm 남짓 앞에서 멈춘다.

| 파라미터 (`controller_server.general_goal_checker`) | Humble 기본 | 여기서 | 설명 |
|---|---|---|---|
| `xy_goal_tolerance` | 0.25 | 0.05 | goal 위치에서 이 거리 안이면 도착 [m] |
| `yaw_goal_tolerance` | 0.25 | 0.05 | goal 방향과 이 각도 안이면 도착 [rad] (약 3°) |

더 정확히 서야 하면 더 줄인다. 다만 위치 추정(AMCL) 오차보다 작게 하면 도착 판정이 늦어지거나 안 날 수 있다.

### 장애물 앞에서 멈췄다 가기 (collision_monitor)

planner 와 controller 는 지도에 없는 장애물을 보지 않는다. 대신 navigation.launch.py 가 Nav2 의 `collision_monitor` 를
Nav2 와 velocity_smoother 사이에 둔다.

```
controller_server, behavior_server ─/cmd_vel_nav─▶ collision_monitor ─/cmd_vel_safe─▶ velocity_smoother ─/cmd_vel─▶ 로봇
```

- 로봇 앞 1.0 m x 폭 0.6 m(SlowZone, RViz 주황)에 레이저 점이 들어오면 속도를 0.3 배로, 앞 0.5 m x 폭 0.5 m
  (StopZone, RViz 빨강)에 들어오면 0 으로 바꾼다. 비면 Nav2 의 속도를 그대로 넘긴다.
- 그 변화는 velocity_smoother 의 감속도(0.6 m/s²)로 이어져서 로봇이 부드럽게 줄이고 선다 ([AMR.md 5장](AMR.md#5-장애물-앞에서-감속하고-서기)).
- Nav2 goal 은 취소되지 않는다. 치우면 같은 경로로 다시 간다.
- 서 있는 시간도 progress checker 에 들어가므로 `movement_time_allowance` 를 30초로 늘렸다 (Humble 기본 10초).

장애물은 `obstacle.py` 로 놓고 치운다 (Gazebo 의 `/spawn_entity`, `/delete_entity` 서비스).

```bash
ros2 run simple_line_bringup obstacle.py put 0,-1     # (0,-1) 에 원기둥 (반지름 0.15 m)
ros2 run simple_line_bringup obstacle.py remove       # 놓은 것 모두 치우기
ros2 run simple_line_bringup obstacle.py click        # RViz 의 Publish Point 로 찍으면 놓고, 다시 찍으면 치운다
```

설정 값, 오래 막혔을 때의 동작, 확인한 결과는 [bt_navigator 6장](../bt_navigator/README.md#6-장애물-멈췄다-가기-stop--go) 에 있다.

## 확인한 것

- Nav2 1.1.18 (Humble) 로 빌드, 경고 없음.
- 실제 Nav2 Humble 서버(map_server, planner_server, controller_server, behavior_server, bt_navigator,
  velocity_smoother)와 Nav2 기본 BT 에 두 플러그인을 올리고, Gazebo 대신 cmd_vel 을 적분하는 가짜 로봇으로
  goal 7개를 이어서 보냈다. 전부 SUCCEEDED, 최종 오차는 xy 0.05m / yaw 1.2° 이내.

  | goal | 시간 | 최종 오차 xy / yaw | (회전 정렬, 허용오차 변경 전) |
  |---|---|---|---|
  | 정면 3m | 15.2 s | 0.05 m / 0.0° | 12.5 s |
  | 정반대(등 뒤) 3m | 18.9 s | 0.05 m / 0.2° | 14.6 s |
  | 왼쪽 대각선 | 17.2 s | 0.05 m / 0.2° | 13.3 s |
  | 진행 방향과 yaw 90° 다름 | 15.3 s | 0.03 m / 0.0° | 11.4 s |
  | 0.15m 짧은 이동 | 3.6 s | 0.03 m / 0.4° | 2.0 s |
  | 제자리 회전 180° | 4.0 s | 0.03 m / 1.2° | 3.7 s |
  | 스폰 위치로 복귀 | 25.9 s | 0.04 m / 1.2° | 21.2 s |

  예전보다 느린 것은 goal 에 더 가까이(0.25m → 0.05m) 가서 서고, 다 돈 뒤에 출발하기 때문이다.
- **출발할 때 회전** (서 있다가 방향이 틀어진 goal 로 출발, 4가지):

  | 출발할 때 틀어진 각도 | 출발 순간 남은 방향 오차 | 직선에서 벗어난 최대 거리 | 최종 오차 xy / yaw |
  |---|---|---|---|
  | 19° | 0.4° (전: 19.3°) | 0.1 cm (전: 7.5 cm) | 0.035 m / 0.1° |
  | 53° | 1.5° (전: 42.9°) | 0.6 cm (전: 15.5 cm) | 0.044 m / 0.3° |
  | 90° | 0.1° (전: 32.4°) | 0.0 cm (전: 13.1 cm) | 0.046 m / 0.0° |
  | 180° | 0.4° (전: 40.8°) | 0.2 cm (전: 14.2 cm) | 0.049 m / 0.1° |

  "전" 은 바꾸기 전 컨트롤러다. 그때는 방향이 45° 안으로 들어오면 바로 출발해서 남은 오차만큼 호를 그렸다.
  감속 계획을 한계 가속도 그대로(`a`) 잡았을 때는 목표 방향을 3~6° 지나쳤다가 되돌아왔고, 절반(`a/2`)으로 잡은 뒤에는
  지나치지 않았다.
- Gazebo 자체(월드 로드, 스폰, AMCL)는 돌려 보지 못했다.
