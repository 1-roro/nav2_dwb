# simple_line: 직선 경로 planner + controller (ROS 2 Humble, C++)

장애물을 고려하지 않는 가장 단순한 형태의 Nav2 플러그인 두 개와 실행 설정이다.

```
simple_line/
├── simple_line_planner/      nav2_core::GlobalPlanner  — start → goal 직선 경로
├── simple_line_controller/   nav2_core::Controller     — pure pursuit + 제자리 회전
└── simple_line_bringup/      launch, 파라미터, 빈 방 월드(6m x 6m), 여러 goal 경유 스크립트
```

## Planner — `simple_line_planner::SimpleLinePlanner`

`src/simple_line_planner.cpp` 의 `createPlan()`.

1. start 와 goal 사이를 `interpolation_resolution` 간격으로 나눈다.
   점 개수는 `ceil(거리 / resolution)` 이라 간격이 resolution 을 넘지 않는다.
2. 중간 점의 orientation 은 모두 직선의 진행 방향 `atan2(dy, dx)` 이다.
3. 마지막 점은 goal 그대로다 (goal 의 orientation 유지).

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `interpolation_resolution` | 0.1 | 점 간격 [m] |

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
4. **제한**
   - goal 까지 남은 경로 길이가 `approach_dist` 안이면 `v · 거리 / approach_dist` 로 감속 (하한 `min_approach_linear_vel`)
   - 선가속도 `max_linear_accel`, 제자리 회전 시 각가속도 `max_angular_accel`
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
| `max_linear_accel` | 0.5 | [m/s²] |
| `max_angular_accel` | 3.2 | [rad/s²] |
| `approach_dist` | 0.6 | goal 감속 시작 거리 [m] |
| `min_approach_linear_vel` | 0.05 | 감속 하한 [m/s] |
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

ros2 launch simple_line_bringup simple_line.launch.py
```

- Gazebo(빈 방) + TurtleBot3 waffle + Nav2 + RViz 가 뜬다. 로봇은 (-2.0, -1.0) 에서 +x 를 보고 시작하고,
  AMCL 초기 자세도 같은 값으로 설정되어 있다.
- RViz 의 **Nav2 Goal** 로 목표를 찍으면 된다. Nav2 기본 BT 를 그대로 쓰므로 1초마다 현재 위치에서 직선을 다시 긋는다.
- 터미널에서 보낼 때:
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}}"
  ```

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `world` / `map` | 빈 방 | 다른 월드를 쓸 때 (예: turtlebot3_world + 그 맵, 스폰 `-2.0, -0.5`) |
| `x_pose` / `y_pose` / `yaw` | -2.0 / -1.0 / 0.0 | 스폰 위치 (AMCL 초기 자세도 같이 바뀜) |
| `headless` / `use_rviz` | False / True | |

`simple_line.launch.py` 는 설치된 Humble 기본 `nav2_params.yaml` 위에
`params/simple_line_params.yaml` 을 덮어써서 `tb3_simulation_launch.py` 에 넘긴다.
플러그인 이름을 기본 BT 가 부르는 `GridBased` / `FollowPath` 그대로 두었기 때문에 BT 는 바꾸지 않는다.

같은 파일에 장애물을 피해 가는 planner `Avoid`(NavFn)와 controller `FollowPathAvoid`(Regulated Pure Pursuit)도
등록되어 있다. 기본 BT 는 이 둘을 부르지 않는다. bt_navigator 미션에서 작업을 `avoid` 로 보낼 때만
`navigate_avoid.xml` 트리를 통해 쓰인다 ([bt_navigator 6-3](../bt_navigator/README.md#6-3-돌아가게-하려면-avoid)).

goal 에 도착했다고 보는 범위(goal checker)도 줄였다. Humble 기본값(xy 0.25m / yaw 0.25rad)이면 로봇이 goal
20cm 남짓 앞에서 멈춘다.

| 파라미터 (`controller_server.general_goal_checker`) | Humble 기본 | 여기서 | 설명 |
|---|---|---|---|
| `xy_goal_tolerance` | 0.25 | 0.05 | goal 위치에서 이 거리 안이면 도착 [m] |
| `yaw_goal_tolerance` | 0.25 | 0.05 | goal 방향과 이 각도 안이면 도착 [rad] (약 3°) |

더 정확히 서야 하면 더 줄인다. 다만 위치 추정(AMCL) 오차보다 작게 하면 도착 판정이 늦어지거나 안 날 수 있다.

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
