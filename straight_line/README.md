# 직선 경로 Planner + Controller 단계별 실습 (ROS 2 Humble, C++)

`dwb_steps.py` 처럼 기능을 **한 단계씩 쌓고**, 중간에 **일부러 실패하는 단계**를 넣어
다음 단계가 왜 필요한지 눈으로 확인하는 실습이다.
planner 와 controller 를 각각 Nav2 플러그인(C++)으로 만들고, 단계는 파라미터 하나로 바꾼다.

```
straight_line/
├── straight_line_planner/      nav2_core::GlobalPlanner 플러그인   (단계 P0~P3)
├── line_follower_controller/   nav2_core::Controller 플러그인      (단계 C0~C4)
├── line_tutorial_sim/          Gazebo 없이 단계 조합을 돌려 보는 2D 시뮬레이터 + 검증 테스트
└── line_tutorial_bringup/      launch, 파라미터, BT, 전용 월드, 시나리오 실행기, 오차 모니터
```

제어 법칙과 경로 생성 코드는 ROS 의존성이 없는 헤더 두 개에 모여 있다.
플러그인과 시뮬레이터가 같은 코드를 쓰므로, 시뮬레이터에서 본 현상이 Nav2 에서도 그대로 나온다.

| 파일 | 내용 |
|---|---|
| `straight_line_planner/include/straight_line_planner/line_builder.hpp` | `buildLine()`: 단계별 직선 생성 |
| `straight_line_planner/src/straight_line_planner.cpp` | 플러그인 껍데기 + P3 costmap 충돌 검사 |
| `line_follower_controller/include/line_follower_controller/follower_core.hpp` | `localWindow()`, `computeCommand()`: 단계별 제어 법칙 |
| `line_follower_controller/src/line_follower_controller.cpp` | 플러그인 껍데기 (TF, 파라미터, 시각화, 예외) |

---

## 단계

### Planner — `straight_line_planner` (파라미터 `GridBased.step`)

| 단계 | 내용 | 관찰 포인트 |
|---|---|---|
| **P0** endpoints | `[start, goal]` 두 점만 | **의도된 실패.** 컨트롤러는 경로를 local costmap 범위(기본 3m x 3m → 반경 1.5m)로 잘라 쓴다. goal 이 그보다 멀면 잘려 나가고 start 한 점만 남는다. 따라갈 점이 로봇 위에 있으니 로봇이 움직이지 않는다. |
| **P1** interpolate | `interpolation_resolution` 간격 보간 (공식 튜토리얼 코드 그대로) | 경로는 촘촘해졌지만 **중간 pose 의 orientation 이 전부 yaw = 0 (map +x)** 이다. 위치만 쓰는 컨트롤러(C2~)는 상관없고, orientation 을 쓰는 C1 은 무너진다. |
| **P2** oriented | 모든 pose 의 yaw 를 진행 방향으로 | P1 의 결함을 고친다. |
| **P3** collision | 직선이 costmap 의 LETHAL/INSCRIBED 셀을 지나면 예외 | 기둥을 지나는 직선은 거부된다. 직선 planner 로는 여기까지가 한계이고, 돌아가는 경로가 필요하면 NavFn/Smac 이 필요하다. |

> 참고: Humble 의 NavFn 도 중간 pose 의 orientation 을 채우지 않는다 (`getPlanFromPotential()` 에서
> 전부 `w = 1`, 마지막 pose 만 goal). 즉 P1 과 같은 경로를 낸다.
> RPP 같은 Nav2 컨트롤러가 경로의 orientation 대신 위치만 쓰는 이유 중 하나다.

### Controller — `line_follower_controller` (파라미터 `FollowPath.step`)

| 단계 | 내용 | 관찰 포인트 |
|---|---|---|
| **C0** goal_seek | 경로를 무시하고 최종 goal 로 P 제어 | baseline. 빈 공간의 직선이면 이것만으로도 도착한다. 기둥이 있으면 그대로 부딪힌다. |
| **C1** heading_track | 가장 가까운 pose 의 orientation + 횡오차: `w = k_h·Δψ + k_e·e` | P2 와는 잘 맞는다. **P1 과 만나면 실패** (map +x 방향이 아닌 직선에서). |
| **C2** pure_pursuit | lookahead 점(carrot)으로 곡률 `k = 2y / L²`, `w = v·k` | orientation 이 필요 없다. **의도된 실패 2개**: ① goal 이 등 뒤면 `y ≈ 0` 이라 `k ≈ 0`, 그대로 앞으로 가 버린다. ② goal yaw 가 진행 방향과 다르면 goal 을 지나친 뒤 ①과 같은 이유로 돌아오지 못한다. |
| **C3** rotate | carrot 이 45° 넘게 틀어져 있으면 제자리 회전, goal xy 에 도착하면 goal yaw 로 제자리 회전 | C2 의 실패 2개를 고친다. |
| **C4** limits | 선가속도 제한, goal 접근 감속, `setSpeedLimit()` | 출발 시 명령 속도가 한 주기에 0 → 0.25 m/s 로 뛰던 것(5 m/s²)이 0.5 m/s² 로 준다. 사실상 mini RPP. |

### 시나리오 — 전용 월드 `line_room`

`line_tutorial_bringup/worlds/line_room.world`: 6m x 6m 방 + 기둥 1개(빨강) + 박스 1개.
turtlebot3_world 는 기둥 간격이 1.1m 라 통로의 좌우 여유가 10cm 남짓이다. 그래서 pure pursuit 가
회전 직후 그리는 작은 호만으로도 기둥에 닿는다. '제어 법칙의 결함'과 '맵이 좁아서 생긴 충돌'이
섞이지 않도록 넓은 방을 따로 만들었다. (`worlds/generate_line_room.py` 가 world 와 map 을 같이 만든다)

A = 스폰 위치 (-2.0, -1.0, 0°), B = A 에서 3m 앞 (1.0, -1.0, 0°)

| 이름 | 출발 → goal | 노리는 것 |
|---|---|---|
| S1_straight | A → B (0°) | 기준. map +x 방향 직선 |
| S2_behind | B → A (180°) | goal 이 등 뒤 (C2 실패) |
| S3_pillar | A → (1.0, 1.0, 34°) | 직선 한가운데 기둥 (P3) |
| S4_goal_yaw | A → B (90°) | goal yaw 가 진행 방향과 90° 다름 (C2 실패) |
| S5_offset | B → (1.3, 2.0, 84°) | 직선이 로봇 왼쪽 84° 방향 (P1 + C1 실패) |

---

## 0. 설치와 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo

mkdir -p ~/nav2_ws/src && cd ~/nav2_ws/src
git clone <이 저장소 주소> nav2_dwb
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

## 1. Gazebo 없이 먼저: `step_matrix`

```bash
ros2 run line_tutorial_sim step_matrix
```

플러그인과 같은 코드(`line_builder.hpp`, `follower_core.hpp`)를 2D 운동학 시뮬레이터에서 돌린다.
시뮬레이터는 바깥에서 Nav2 Humble 의 기본 동작을 흉내 낸다: 20Hz 제어 루프, stateful goal checker
(0.25m / 0.25rad), progress checker (10초에 0.5m), controller 의 `failure_tolerance` 0.3초,
velocity smoother (0.26 m/s, 1.0 rad/s), BT 의 1회 계획 또는 1Hz 재계획.

```
== A. planner steps, controller fixed to C4, plan once
                       S1_straight     S2_behind       S3_pillar       S4_goal_yaw     S5_offset
P0_endpoints           no_progress     no_progress     no_progress     no_progress     no_progress
P1_interpolate         ok  12.0s       ok  14.6s       collision       ok  13.5s       ok  13.0s
P2_oriented            ok  12.0s       ok  14.6s       collision       ok  13.5s       ok  13.0s
P3_collision           ok  12.0s       ok  14.6s       plan_reject     ok  13.5s       ok  13.0s

== B. controller steps, planner fixed to P2, plan once
                       S1_straight     S2_behind       S3_pillar       S4_goal_yaw     S5_offset
C0_goal_seek           ok  11.1s       ok  13.6s       collision       ok  13.6s       ok  12.0s
C1_heading_track       ok  11.1s       ok  14.8s       collision       ok  13.4s       ok  11.8s
C2_pure_pursuit        ok  11.1s       ctrl_fail@6.3s  collision       ctrl_fail@18.3s ok  12.0s
C3_rotate              ok  11.1s       ok  13.8s       collision       ok  12.6s       ok  12.2s
C4_limits              ok  12.0s       ok  14.6s       collision       ok  13.5s       ok  13.0s

== C. planner x controller cross-check, plan once
                       S1_straight     S2_behind       S3_pillar       S4_goal_yaw     S5_offset
P0 + C0                ok  11.1s       ok  13.6s       collision       ok  13.6s       ok  12.0s
P0 + C1                ok  11.1s       ctrl_fail@6.3s  ctrl_fail@6.3s  ctrl_fail@16.4s ctrl_fail@6.3s
P0 + C2                no_progress     no_progress     no_progress     no_progress     no_progress
P1 + C1                ok  11.1s       ctrl_fail@6.3s  ctrl_fail@21.5s ok  13.4s       ctrl_fail@6.5s
P2 + C1                ok  11.1s       ok  14.8s       collision       ok  13.4s       ok  11.8s
P1 + C2                ok  11.1s       ctrl_fail@6.3s  collision       ctrl_fail@18.3s ok  12.1s
P1 + C4                ok  12.0s       ok  14.6s       collision       ok  13.5s       ok  13.0s

== D. C4 ramps up and slows down near the goal (S1, P2, plan once)
                               outcome           time[s]   maxCTE   rmsCTE  maxCTE*  xy_err  yaw_err peak_acc peak_dec
C3_rotate                      reached              11.1     0.00     0.00     0.00    0.24     0.00      5.0      5.0
C4_limits                      reached              12.0     0.00     0.00     0.00    0.25     0.00      0.5      2.1

== E. replanning hides controller problems
                               outcome           time[s]   maxCTE   rmsCTE  maxCTE*  xy_err  yaw_err peak_acc peak_dec
P2 + C2 S4 / plan once         controller_failed    18.3     1.51     0.53     1.51    1.51     1.57      5.0      5.0
P2 + C2 S4 / replan 1Hz        collision            19.1     1.77     0.62     0.25    1.78     1.57      5.0      0.0
P1 + C1 S5 / plan once         controller_failed     6.5     1.51     0.92     1.51    3.14     1.30      5.0      5.0
P1 + C1 S5 / replan 1Hz        collision             7.1     1.76     1.02     0.25    3.35     1.47      5.0      0.0
P2 + C4 S5 / plan once         reached              13.0     0.14     0.07     0.14    0.25     0.01      0.5      2.1
P2 + C4 S5 / replan 1Hz        reached              13.0     0.16     0.10     0.10    0.25     0.08      0.5      2.1
```

- `no_progress`: progress checker 실패 (10초 동안 0.5m 를 못 움직임)
- `ctrl_fail`: 컨트롤러 예외가 0.3초 넘게 이어짐 (local costmap 안에 경로 pose 가 없음).
  Nav2 로그로는 `no path poses inside the local costmap` 경고 뒤 `Controller patience exceeded`
- `collision`: 기둥/벽/박스에 닿음 (컨트롤러에 충돌 회피가 없으므로 Gazebo 에서는 실제로 부딪힌다)
- `maxCTE` / `rmsCTE`: **처음 그은 직선**에서 벗어난 거리, `maxCTE*`: **가장 최근 경로**에서 벗어난 거리

한 조합만 자세히 보고 궤적을 CSV 로 뽑을 수도 있다.

```bash
ros2 run line_tutorial_sim step_matrix -s S4_goal_yaw -p 2 -c 2 --trace c2_s4.csv
ros2 run line_tutorial_sim step_matrix -s S4_goal_yaw -p 2 -c 2 --replan
```

`colcon test --packages-select line_tutorial_sim` 은 이 README 의 주장(위 표의 핵심 칸)을 테스트로 확인한다.
제어 법칙을 바꿨는데 테스트가 깨지면 설명도 같이 고쳐야 한다는 뜻이다.

## 2. Gazebo 실습 준비

```bash
# 터미널 1: Gazebo + Nav2 + RViz (+ 오차 모니터)
ros2 launch line_tutorial_bringup tutorial.launch.py

# 터미널 2: 시나리오 실행
ros2 run line_tutorial_bringup send_scenario.py --list
ros2 run line_tutorial_bringup send_scenario.py S1_straight
```

`tutorial.launch.py` 는 nav2_bringup 의 `tb3_simulation_launch.py` 를 그대로 쓴다. 설치된 Humble 기본
`nav2_params.yaml` 위에 `params/line_tutorial_overrides.yaml` 을 덮어써서 넘긴다.
플러그인 이름을 `GridBased` / `FollowPath` 그대로 두었기 때문에 BT 를 고치지 않아도 된다.

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `planner_step` | 2 | 0~3 |
| `controller_step` | 4 | 0~4 |
| `bt` | once | `once`: 한 번 계획 후 끝까지 추종, `replan`: 1Hz 재계획, `default`: Nav2 기본 BT |
| `planner` | line | `navfn` 이면 GridBased 자리에 NavFn 을 꽂는다 (비교용) |
| `controller` | line | `rpp` 면 FollowPath 자리에 RPP 를 꽂는다 (비교용) |
| `world` / `map` / `x_pose` / `y_pose` / `yaw` | line_room, A 지점 | 다른 월드를 쓸 때 |
| `headless` / `use_rviz` / `monitor` | False / True / True | |

`send_scenario.py` 가 하는 일은 두 가지다.
1. Nav2 기본 플러그인(NavFn + RPP)으로 로봇을 시나리오 출발 자세까지 옮긴다. 이때만 goal checker
   허용오차를 8cm / 3° 로 좁혔다가 되돌린다. 실험할 플러그인과는 상관없는 이동이다.
2. NavigateToPose 로 시나리오 goal 을 보낸다. 이 goal 은 bt_navigator 가 실험 중인
   GridBased / FollowPath 로 실행한다. `--bt replan` 으로 이 goal 만 재계획 BT 로 돌릴 수 있다.

**단계는 재시작 없이 바꿀 수 있다.** 두 플러그인 모두 매 호출마다 파라미터를 다시 읽는다.

```bash
ros2 param set /planner_server GridBased.step 1
ros2 param set /controller_server FollowPath.step 1
```

goal 이 끝날 때마다 `cte_monitor` 가 다음처럼 출력한다 (아래는 headless 통합 테스트에서 나온 실제 출력).

```
[cte_monitor]: goal SUCCEEDED in 12.3 s (1 plan(s) received)
  CTE vs original path : max 0.00 m, rms 0.00 m
  CTE vs latest path   : max 0.00 m
  final error          : xy 0.25 m, yaw 0.0 deg
  peak cmd accel       : 0.50 m/s^2
```

controller 로그에는 단계와 상태 전이가 찍힌다: `[C4_limits] state -> rotate_to_path -> pure_pursuit -> rotate_to_goal`.

RViz 에서 보면 좋은 것:
- `/plan` (Path): **Pose Style 을 Arrows 로** 바꾸면 P1 의 orientation 이 전부 +x 를 향하는 게 보인다
- `/line_follower/local_plan` (Path): 컨트롤러가 local costmap 범위로 잘라 낸 경로. P0 에서는 점 하나만 남는다
- `/line_follower/carrot` (PointStamped): pure pursuit 의 lookahead 점

## 3. 실습 순서

### B1. Planner 단계 (controller 는 C4 로 고정)

**P0: 두 점짜리 경로**

```bash
ros2 param set /planner_server GridBased.step 0
ros2 run line_tutorial_bringup send_scenario.py S1_straight
```
- 예상: 로봇이 출발하지 않는다. controller 로그는 `state -> carrot_at_robot`. 정확히 10초 뒤
  controller_server 가 `Failed to make progress` (progress checker) 를 내고 goal 이 ABORTED 된다.
- 왜: `localWindow()` 가 로봇에서 1.5m 넘는 첫 pose 에서 멈춘다. 3m 앞 goal 이 잘려서 start 한 점만 남는다.
- 해 볼 것: 1.5m 보다 가까운 goal 을 RViz 로 찍으면 P0 로도 간다.
  `step 0 + controller_step 0` (경로를 안 쓰는 C0)도 간다.

**P1 vs P2: orientation**

C4 로는 차이가 없다. pure pursuit 는 pose 의 위치만 쓰기 때문이다. RViz 에서 화살표만 비교하고,
차이는 B2 의 C1 에서 본다.

**P3: 충돌 검사**

```bash
ros2 param set /planner_server GridBased.step 2
ros2 run line_tutorial_bringup send_scenario.py S3_pillar     # 기둥에 부딪힌다
ros2 param set /planner_server GridBased.step 3
ros2 run line_tutorial_bringup send_scenario.py S3_pillar     # planner 가 거부한다
```
- P3 로그: `[GridBased] rejecting straight line: line is blocked 1.34... m from the start`, goal 은 바로 ABORTED.

### B2. Controller 단계 (planner 는 P2 로 고정)

**C0 / C1**

```bash
ros2 param set /controller_server FollowPath.step 1
ros2 run line_tutorial_bringup send_scenario.py S5_offset          # P2 + C1: 도착
ros2 param set /planner_server GridBased.step 1
ros2 run line_tutorial_bringup send_scenario.py S5_offset          # P1 + C1: 실패
ros2 run line_tutorial_bringup send_scenario.py S1_straight        # P1 + C1: 도착 (!)
```
- P1 + C1 / S5: 경로 orientation 이 0 이라 로봇이 map +x 를 보려고 하고, 횡오차 항과 줄다리기를 하다가
  경로를 벗어난다. 경로가 local costmap 밖으로 나가면 controller 가 실패한다.
- P1 + C1 / S1 이 되는 이유: S1 은 map +x 방향 직선이라 yaw = 0 이 우연히 맞는다.
  **결함이 있어도 시나리오에 따라 드러나지 않을 수 있다.** 그래서 시나리오를 여러 방향으로 만들었다.

**C2: pure pursuit 의 실패 두 개**

```bash
ros2 param set /planner_server GridBased.step 2
ros2 param set /controller_server FollowPath.step 2
ros2 run line_tutorial_bringup send_scenario.py S2_behind     # 앞으로 가 버린다
ros2 run line_tutorial_bringup send_scenario.py S4_goal_yaw   # goal 을 지나쳐 계속 간다
```
- S2: carrot 이 정확히 뒤에 있으면 `y = 0` 이라 곡률이 0 이다. 앞으로 1.5m 가면 경로가 local costmap 밖으로
  나가고 controller 가 실패한다.
- S4: goal xy 에 들어온 순간 goal checker 가 xy 를 통과 처리한다 (stateful). 하지만 yaw 가 90° 다르니
  끝나지 않는다. 지나친 뒤에는 goal 이 등 뒤에 있으므로 S2 와 같은 이유로 직진한다.

**C3 / C4**

```bash
ros2 param set /controller_server FollowPath.step 3     # S2, S4 둘 다 도착
ros2 param set /controller_server FollowPath.step 4     # cte_monitor 의 peak cmd accel: 5.0 -> 0.5
```

### B3. 재계획이 컨트롤러 문제를 가린다

기본 BT 는 1Hz 로 재계획한다. 직선 planner 는 매번 **로봇의 현재 위치 → goal** 직선을 다시 그리므로,
로봇이 경로에서 벗어나도 1초 뒤에는 새 경로 위에 서 있게 된다.

```bash
ros2 param set /controller_server FollowPath.step 2
ros2 run line_tutorial_bringup send_scenario.py S4_goal_yaw               # 1회 계획: controller 실패, 정지
ros2 run line_tutorial_bringup send_scenario.py S4_goal_yaw --bt replan   # 재계획: 실패 없이 벽까지 간다
```
- 1회 계획에서는 컨트롤러가 분명하게 실패하고 로봇이 멈춘다.
- 재계획에서는 컨트롤러 예외가 한 번도 나지 않는다. 로봇은 goal 을 지나 벽에 부딪힌다.
- cte_monitor 의 `CTE vs latest path` 는 최대 0.25m 로 작게 나온다. `1초 x 0.25 m/s` 이상 벗어날 틈이
  없기 때문이다. 반면 `CTE vs original path` 는 1.7m 이상이다.
  **최신 경로 기준으로만 재면 컨트롤러가 잘 따라가는 것처럼 보인다.**
- 정상 조합(P2 + C4, S5)에서도 재계획하면 원래 직선 기준 rms 오차가 0.07 → 0.10m 로 커진다.
  컨트롤러가 수렴하기 전에 기준선이 계속 바뀌기 때문이다.

### B4. Nav2 기본 플러그인과 비교

```bash
ros2 launch line_tutorial_bringup tutorial.launch.py controller:=rpp        # 직선 planner + RPP
ros2 launch line_tutorial_bringup tutorial.launch.py planner:=navfn         # NavFn + 우리 controller
```
- `controller:=rpp`: S1~S5 를 돌려 C4 와 비교해 본다 (RPP 는 충돌 검사와 곡률 기반 감속이 더 있다).
- `planner:=navfn` + `FollowPath.step 4` + S3: NavFn 이 기둥을 돌아가는 경로를 내고 우리 컨트롤러가 그걸 따라간다.
  `FollowPath.step 0` (경로 무시)으로 바꾸면 다시 기둥에 부딪힌다. 경로를 따라가야 하는 이유가 여기서 보인다.
  (C0 가 B2 에서 멀쩡했던 건, 빈 공간의 직선 경로가 '로봇 → goal' 방향과 같았기 때문이다)
  `FollowPath.step 1` 은 NavFn 경로의 orientation 이 전부 0 이라 P1 때처럼 무너진다.

## 4. 더 해 볼 것

- **C5 충돌 검사**: 앞으로 갈 원호를 local costmap 에 대 보고 막히면 멈추기 (RPP 의 `use_collision_detection`)
- 속도에 비례하는 lookahead (`use_velocity_scaled_lookahead_dist`)
- `closestIndex()` 의 탐색 범위 제한: 지금은 경로 전체를 훑는다. 되돌아오는 경로에서는 RPP 처럼
  `max_robot_pose_search_dist` 가 필요하다

## 부록 A. Humble 이후 버전에서 달라지는 것

- `GlobalPlanner::createPlan()`: Jazzy 부터 `std::function<bool()> cancel_checker` 인자가 추가된다
- 예외: Humble 의 `nav2_core` 에는 `PlannerException` 하나뿐이다. Iron 부터 `planner_exceptions.hpp`
  (`NoValidPathCouldBeFound` 등)와 `controller_exceptions.hpp` 가 생기고, 에러 코드가 action 결과에 실린다
- Kilted 부터 노드 타입이 `nav2::LifecycleNode` 로 바뀐다
- Jazzy 부터 Gazebo Classic 대신 Gazebo (gz sim) 를 쓰고, loopback 시뮬레이터가 생긴다

## 부록 B. 검증한 범위

- 세 C++ 패키지는 Nav2 1.1.18 (Humble) 헤더로 빌드했고, 단위/폐루프 테스트 22개가 통과한다
  (`colcon test`).
- 실제 Nav2 Humble 서버(map_server, planner_server, controller_server, bt_navigator,
  velocity_smoother, lifecycle_manager)에 두 플러그인을 올리고 `line_room` 맵으로 headless 통합 테스트를 했다.
  Gazebo 대신 cmd_vel 을 적분하는 가짜 로봇을 썼고(충돌 판정 없음), `send_scenario.py`(출발점 이동 포함)와
  `cte_monitor.py` 도 같이 돌렸다. 결과는 아래와 같고, 시뮬레이터 표와 성공/실패가 모두 일치했다
  (소요 시간 차이 0.3초 이내).

  | 케이스 | Nav2 결과 | step_matrix 예측 |
  |---|---|---|
  | P2+C4: S1 / S2 / S4 / S5 | SUCCEEDED 12.3 / 14.6 / 13.7 / 13.2 s | ok 12.0 / 14.6 / 13.5 / 13.0 s |
  | P3+C4: S3 | ABORTED 0.0 s (`line is blocked`) | plan_reject |
  | P0+C4: S1 | ABORTED 10.1 s (`Failed to make progress`) | no_progress |
  | P0+C0: S1 | SUCCEEDED 11.2 s | ok 11.1 s |
  | P1+C1: S5 / S1 | ABORTED 6.4 s / SUCCEEDED 11.2 s | ctrl_fail 6.5 s / ok 11.1 s |
  | P2+C1: S5 | SUCCEEDED 11.9 s | ok 11.8 s |
  | P2+C2: S2 / S4 | ABORTED 6.3 s / 18.3 s (`Controller patience exceeded`) | ctrl_fail 6.3 / 18.3 s |
  | P2+C3: S2 | SUCCEEDED 13.9 s | ok 13.8 s |
  | P2+C2: S4, 재계획 | 120초 동안 실패 없음, 27m 이탈. CTE vs latest 0.26m / vs original 26.98m | collision (벽) |
  | P2+C4: S5, 재계획 | SUCCEEDED 13.1 s, rms CTE 0.10m | ok 13.0 s, rms 0.10m |
  | NavFn + C4 / C0 / C1 / C2: S3 | 기둥 표면과 0.57m 거리로 통과 / 기둥 관통(-0.10m) / ABORTED / 통과 | (시뮬레이터에 NavFn 없음) |

  가짜 로봇에는 충돌이 없어서, 시뮬레이터에서 `collision` 인 케이스(P2+C4 의 S3 등)는 기둥/벽을 그대로 지나갔다.
- `tutorial.launch.py` 의 파라미터 병합(기본값, `planner:=navfn`, `controller:=rpp`, `bt:=default`)은
  Humble 의 `nav2_params.yaml` 로 따로 돌려 확인했다.
- Gazebo 와 `tutorial.launch.py` 의 Gazebo 부분(`line_room` 월드 로드, 스폰, AMCL)은 직접 돌려 보지 못했다.
  처음 실행할 때 문제가 생기면 이 부분부터 확인한다.
