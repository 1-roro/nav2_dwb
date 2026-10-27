# bt_navigator: BT 로 이동과 작업을 조합하는 미션

[simple_line/MULTI_GOAL.md](../simple_line/MULTI_GOAL.md) 의 방법은 Nav2 명령 하나로 여러 goal 을 돈다. 실제 작업에서는 "어디로 가서 무엇을 하라" 가 실행 중에 정해지고, 점에 서서 일하거나 움직이면서 일한다.

이런 일은 Nav2 위에 **상위 작업 계층**을 두고 맡긴다. 상위 작업 계층이 작업을 받아서, 단계마다 Nav2 명령을 골라 보낸다. 이 폴더는 상위 작업 계층을 BT(BehaviorTree.CPP)로 만든 예제다.

- **작업 하나** = 들를 점들 + 할 일 (`go`, `inspect`, `scan`, `look_around`)
- 로봇은 **점마다 멈춰서** 위치와 방향을 맞추고, 도착을 알린 뒤 다음 점으로 간다.
- 작업은 **액션**(`execute_task`)으로 받는다. 결과, 진행 상황(BT 상태 포함), 취소, 선점이 있다.
- 길에 장애물이 있으면 **멈춰서 기다렸다가** 치우면 다시 간다 (stop & go). Nav2 goal 은 그대로 두고 속도만 막는다. Nav2 의 collision_monitor 가 한다 (6장).

```
bt_navigator/
├── bt_mission/        미션 BT (task.xml), 직접 만든 BT 노드, 작업 액션 서버, 클라이언트
└── bt_mission_msgs/   작업 메시지와 액션 (Task.msg, ExecuteTask.action)
```

| 하고 싶은 것 | 실행 | 장 |
|---|---|---|
| 점과 작업을 실행 중에 액션으로 맡기기 (결과, 진행 상황, 취소, 선점) | `task_server` (`task.xml`)<br>+ `task_client.py` | 4 |
| 위와 같되, 서버를 Nav2 의 `BtActionServer` 와 lifecycle 노드로 | `task_server_lifecycle.launch.py`<br>+ `task_client.py` | 5 |
| 장애물이 있으면 멈췄다가 치우면 가기. 시뮬레이션에서 장애물을 놓고 치워 보기 | `simple_line.launch.py` 에 들어 있다<br>+ `obstacle.py click` | 6 |

- **명령 내리는 법**(입력 형식과 예시)은 [4-5](#4-5-명령-내리기-입력-형식과-예시) 에 모아 두었다.
- 아래층(planner, controller, 시뮬레이션 launch)은 [`simple_line/`](../simple_line/README.md) 것을 그대로 쓴다.
- 폴더 이름의 bt_navigator 는 Nav2 의 bt_navigator 노드를 바꾼다는 뜻이 아니다. Nav2 bt_navigator 는 그대로 두고, 이 폴더의 미션 BT 가 그 위에서 명령을 보낸다 (1장).

목차

- [0. 빌드와 시뮬레이션 실행](#0-빌드와-시뮬레이션-실행)
- [1. 구조: 미션 BT 는 Nav2 위에 한 층 더 올라간다](#1-구조-미션-bt-는-nav2-위에-한-층-더-올라간다)
- [2. BT 기초: 이 예제를 읽는 데 필요한 만큼](#2-bt-기초-이-예제를-읽는-데-필요한-만큼)
- [3. 코드로 구현하기 (C++)](#3-코드로-구현하기-c)
- [4. 액션으로 작업 맡기 (task_server, task.xml)](#4-액션으로-작업-맡기-task_server-taskxml)
- [5. Nav2 의 BtActionServer 로 만들기 (task_server_lifecycle)](#5-nav2-의-btactionserver-로-만들기-task_server_lifecycle)
- [6. 장애물: 멈췄다 가기 (stop & go)](#6-장애물-멈췄다-가기-stop--go)
- [7. 확인한 것](#7-확인한-것)
- [8. 어떤 방법을 쓸까](#8-어떤-방법을-쓸까)
- [9. 주의할 점 (Humble)](#9-주의할-점-humble)
- [10. 파일](#10-파일)

---

## 0. 빌드와 시뮬레이션 실행

```bash
cd ~/nav2_ws/src/nav2_dwb && git pull
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -y      # nav2_collision_monitor, nav2_simple_commander 등
colcon build --symlink-install --packages-select \
  simple_line_planner simple_line_controller simple_line_bringup \
  bt_mission_msgs bt_mission
source install/setup.bash
```

예전 이름(`simple_line_mission`, `simple_line_mission_msgs`)으로 빌드한 적이 있으면 그 결과물은 지운다. 남겨 두면 옛 패키지도 계속 설치된 것으로 보인다.

```bash
cd ~/nav2_ws && rm -rf build/simple_line_mission* install/simple_line_mission*
```

```bash
# 터미널 1: Gazebo + Nav2 + RViz (로봇은 (-2,-1) 에서 +x 를 보고 시작)
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch simple_line_bringup simple_line.launch.py
```

이후 장의 명령은 모두 **터미널 2** 이후에서 실행한다. 좌표는 모두 빈 방(6m x 6m, 벽은 ±3m) map frame 기준이다.

**가장 빨리 해 보기**: 작업을 기다리는 액션 서버를 띄우고, 작업 하나를 보낸다.

```bash
# 터미널 2
ros2 run bt_mission task_server
# 터미널 3: (0,-1) → (0,1) → (-2,-1) 을 차례로 들르고, 마지막 점에서 0° 를 보고 선다
ros2 run bt_mission task_client.py go 0,-1 0,1 -2,-1,0
```

입력 형식과 예시는 [4-5](#4-5-명령-내리기-입력-형식과-예시) 에 있다. 길에 장애물이 있으면 로봇이 멈춰서 기다린다. 장애물을 직접 놓고 치워 보는 법은 6-1 에 있다.

---

## 1. 구조: 미션 BT 는 Nav2 위에 한 층 더 올라간다

![layers](bt_mission/doc/layers.png)

| 계층 | 하는 일 | 이 예제에서 |
|---|---|---|
| **입력** (4장) | 어디로 가서 무엇을 할지 보냄 | `task_client.py` → `execute_task` 액션 |
| **상위 작업 계층** | 무엇을, 어떤 순서로, 실패하면 어떻게 | `task_server` 또는 `task_server_lifecycle` (`task.xml`) |
| **bt_navigator** | goal 까지 **어떻게 갈지** (재계획, recovery) | Nav2 기본 트리에서 회전, 후진 recovery 만 뺀 `navigate_to_pose_amr.xml` ([AMR.md 6장](../simple_line/AMR.md#6-막혔을-때-회전-후진-없이-기다리기)) |
| **서버** | 실제 경로 계산, 경로 추종(경로 위 장애물 앞에서 감속해서 서기 포함), 제자리 동작 | planner_server (SimpleLinePlanner), controller_server (SimpleLineController), behavior_server (Spin) |
| **속도 관문** (6장) | 갑자기 가까이 막히면 비상 정지(0), 가감속 한계 | collision_monitor (`/cmd_vel_nav` → `/cmd_vel_safe`), velocity_smoother (→ `/cmd_vel`) |

BT 가 두 개다.

- 이 예제의 **미션 BT** (`task.xml`): 받은 작업을 어떤 순서로, 어떻게 할지 정한다.
- bt_navigator 안의 **navigation BT** (`navigate_to_pose_amr.xml`, Nav2 기본 `navigate_to_pose_w_replanning_and_recovery.xml` 에서 recovery 만 바꿈): goal 하나까지 가는 방법을 정한다. 1초마다 경로를 다시 계산하고, 실패하면 기다렸다가 다시 한다. goal 이 올 때마다 새로 실행된다.

미션 BT 의 `NavigateToPose` 노드가 bt_navigator 에 액션을 보내면, bt_navigator 가 navigation BT 를 돌린다. 작업 로직을 navigation BT 에 섞지 않으니, 이동 실패 처리(재계획, recovery)와 작업 실패 처리(보고, 다음 작업)가 서로 엉키지 않는다.

장애물은 미션 BT 도 navigation BT 도 아닌 아래 층에서 다룬다 (6장). 경로 위에 장애물이 있으면 controller 가 그 앞에서 감속해서 서고(0 을 낸다), 갑자기 가까이 나타나면 collision_monitor 가 비상 정지한다. 위의 두 BT 는 goal 을 그대로 둔 채 계속 실행 중이다.

---

## 2. BT 기초: 이 예제를 읽는 데 필요한 만큼

### tick 과 상태

BT 는 루트부터 **tick** 을 한다. 실행 노드가 tick 을 주기적으로 반복하고(여기서는 100Hz), 매 tick 마다 루트에서 지금 실행할 노드까지 내려간다. tick 을 받은 노드는 셋 중 하나를 돌려준다.

| 상태 | 뜻 |
|---|---|
| `SUCCESS` | 끝났고 성공 |
| `FAILURE` | 끝났고 실패 |
| `RUNNING` | 아직 하는 중. 다음 tick 에 다시 물어봐 달라 |

예를 들어 `task.xml` 로 `inspect 0,-1 1.5,-1,90` 작업을 할 때, `go_inspect` 가 (0,-1) 까지 주행하는 약 11초 동안에도 BT 는 100Hz 로 tick 한다 (그림은 4-2).

1. 매 tick: `by_action → inspect → inspect_points → go_inspect` 로 내려가고, `go_inspect` 는 `RUNNING` 을 돌려준다.
2. (0,-1) 에 도착하면 `go_inspect` 가 `SUCCESS` 를 돌려준다.
3. 같은 tick 안에서 `inspect_points` 가 도착을 알리고, 다음 점 (1.5,-1) 으로 `go_inspect` 를 다시 시작한다.
4. 마지막 점에 도착하면 `inspect_points` 가 `SUCCESS` 를 돌려주고, `inspect` Sequence 가 다음 자식 `work` 를 시작한다.

### 노드 종류

| 종류 | 노드 | 동작 |
|---|---|---|
| 제어 | **Sequence** (→) | 자식을 왼쪽부터 차례로. 하나라도 FAILURE 면 바로 FAILURE. 모두 SUCCESS 면 SUCCESS |
| 제어 | **Parallel** (⇉) | 자식을 매 tick 모두 실행. `success_threshold` 개가 SUCCESS 면 SUCCESS. 나머지 RUNNING 자식은 **halt** (중단) |
| 제어 | **Switch3** (⇢) | `variable` 값이 `case_1..3` 중 맞는 자식 하나만 실행. 맞는 것이 없으면 마지막 자식 (4-3) |
| 데코레이터 | **ForEachWaypoint** (직접 만듦) | 점 목록의 점마다 자식을 한 번씩 다시 실행 (4-2) |
| 액션 | **Nav2 BT 노드** | `NavigateToPose`, `Spin`. Nav2 서버에 액션을 보내고, 끝날 때까지 RUNNING |
| 액션 | **직접 만든 노드** | `DoWork`, `ScanWhileMoving` (3장) |

### blackboard 와 포트

노드끼리 값을 주고받는 공용 저장소가 **blackboard** 다. XML 에서 `{이름}` 은 blackboard 변수를 뜻한다.

```xml
<!-- {waypoints} 는 서버가 goal 을 받을 때 받은 작업의 점 목록으로 채운다 -->
<ForEachWaypoint poses="{waypoints}" pose="{waypoint}">         <!-- waypoints 를 읽어 지금 갈 점을 waypoint 에 쓰고 -->
  <NavigateToPose goal="{waypoint}"/>                           <!-- waypoint 를 goal 로 읽는다 -->
</ForEachWaypoint>
<ScanWhileMoving interval="0.5"/>                               <!-- {} 가 없으면 적은 값 그대로 -->
```

`poses`, `pose`, `goal`, `interval` 같은 이름이 **포트**다. 각 노드 클래스가 어떤 포트를 갖는지 `providedPorts()` 로 선언한다. `{}` 없이 쓴 값(`"0.5"`)은 문자열 그대로 들어가고, 포트 타입(여기서는 `double`)으로 바뀐다.

---

## 3. 코드로 구현하기 (C++)

BT 를 코드로 쓰는 일은 세 가지다.

1. 필요한 **노드 종류를 만든다**. 직접 만들 노드만.
2. 실행 노드에서 노드 종류를 **factory 에 등록**하고, **blackboard** 를 채운다.
3. XML 로 **트리를 만들고 tick** 한다.

### 3-1. BT 노드 만들기 (`include/bt_mission/mission_nodes.hpp`, `src/mission_nodes.cpp`)
**결과 node 구조.**
```
ROS 노드 /task_server   (실행 파일 task_server. task_server_lifecycle 도 같다)
 └ 미션 BT
    ├ Sequence, Parallel, Switch3                  흐름만 정함. 밖으로 아무것도 안 보냄
    ├ ForEachWaypoint                              흐름 (점마다 다시 실행) + 보고
    ├ DoWork, ScanWhileMoving                      이 ROS 노드 안에서 직접 처리
    └ NavigateToPose, Spin                         다른 ROS 노드에 액션 goal 을 보내고 결과를 기다림
              │ 액션                         │ 액션 (spin)
              ▼                              ▼
ROS 노드 /bt_navigator                   ROS 노드 /behavior_server
 └ navigation BT (또 하나의 트리)
    ├ PipelineSequence, RecoveryNode, RateController   흐름
    └ ComputePathToPose, FollowPath                    다른 ROS 노드에 액션 goal
              │ 액션                  │ 액션
              ▼                       ▼
ROS 노드 /planner_server           ROS 노드 /controller_server
 └ 플러그인 SimpleLinePlanner         └ 플러그인 SimpleLineController
   (ROS 노드 아님, 서버가 부르는 클래스)  (20Hz 로 불려서 /cmd_vel_nav 토픽 발행)
```

아래 코드는 발췌다 (선언과 정의를 한곳에 모으고 일부 줄을 뺐다).

**한 번에 끝나는 노드** (`BT::SyncActionNode`): `tick()` 이 SUCCESS / FAILURE 를 바로 돌려준다. 이 패키지에는 지금 이 종류가 없어서, 예로 하나 든다.

```cpp
class BatteryOk : public BT::SyncActionNode              // 예: 배터리가 충분한지 (이 패키지에는 없다)
{
public:
  BatteryOk(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()                     // XML 에서 쓸 수 있는 속성
  {
    return {
      BT::InputPort<double>("level"),                      // 읽는 값
      BT::InputPort<double>("min", 0.2, "lowest level to go on"),
      BT::OutputPort<double>("margin"),                    // 쓰는 값
    };
  }

  BT::NodeStatus tick() override
  {
    double level = 0.0, min = 0.2;
    getInput("level", level);                              // XML 의 level="{battery}" 를 읽는다
    getInput("min", min);
    setOutput("margin", level - min);                      // blackboard 의 {margin} 에 쓴다
    return level >= min ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
```

**여러 tick 에 걸쳐 실행되는 노드** (`BT::StatefulActionNode`):

| 함수 | 언제 불리나 |
|---|---|
| `onStart()` | 처음 tick 될 때 |
| `onRunning()` | RUNNING 인 동안 매 tick |
| `onHalted()` | 실행 중에 부모가 중단시킬 때 |

```cpp
// DoWork: 정지 상태 작업 자리. 지금은 duration 초 동안 RUNNING
BT::NodeStatus DoWork::onStart()
{
  getInput("task", task_);
  getInput("duration", duration_);
  start_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;                           // 다음 tick 부터 onRunning()
}

BT::NodeStatus DoWork::onRunning()
{
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_;
  return elapsed.count() < duration_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}
```

- `tick()` 안에서 `sleep` 하면 안 된다. 그러면 그동안 트리 전체가 멈추고, Parallel 의 다른 자식도 tick 을 못 받는다. 오래 걸리는 일은 RUNNING 을 돌려주고 다음 tick 에 이어서 확인한다.
- 실제 작업(촬영, 집기, IO)은 `DoWork` 자리에서 서비스나 액션을 비동기로 부르고, 응답이 올 때까지 RUNNING 을 돌려주면 된다. Nav2 의 `nav2_behavior_tree::BtActionNode<ActionT>` / `BtServiceNode<ServiceT>` 템플릿을 상속하면 액션·서비스 호출 노드를 짧게 만들 수 있다. Nav2 BT 노드들이 그렇게 만들어져 있다.

**주행 중 작업 노드** (`ScanWhileMoving`):

- `onRunning()` 에서 TF 로 로봇 위치를 읽는다.
- `interval` 만큼 움직였으면 위치를 기록하고 `mission/scan_points` (`geometry_msgs/PoseArray`) 로 발행한다.
- 언제나 RUNNING 을 돌려준다. 끝내는 것은 Parallel 의 몫이다.

노드가 쓰는 ROS 노드와 TF 버퍼는 생성자에서 blackboard 로 받는다.

```cpp
ScanWhileMoving::ScanWhileMoving(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
```

**등록**: XML 태그 이름과 클래스를 묶는다.

```cpp
void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DoWork>("DoWork");
  factory.registerNodeType<ScanWhileMoving>("ScanWhileMoving");
  factory.registerNodeType<ForEachWaypoint>("ForEachWaypoint");  // 4-2
}
```

### 3-2. 실행 노드 (`src/task_server_main.cpp`)

발췌다 (예외 처리, 로그는 뺐다). 1, 2 의 본문은 `src/bt_common.cpp` 의 `registerAllNodes()`, `createBlackboard()` 에 있다.

```cpp
// 1. BT 노드 종류 등록
BT::BehaviorTreeFactory factory;
BT::SharedLibrary loader;
for (const char * lib : {"nav2_navigate_to_pose_action_bt_node",          // Nav2 가 제공하는 노드:
    "nav2_navigate_through_poses_action_bt_node",                         //   플러그인 .so 를 이름으로
    "nav2_spin_action_bt_node", "nav2_wait_action_bt_node"})               //   (bt_navigator 와 같은 방식)
{
  factory.registerFromPlugin(loader.getOSName(lib));
}
bt_mission::registerNodes(factory);                                        // 직접 만든 노드

// 2. blackboard: Nav2 BT 노드는 생성될 때 이 키들을 꼭 찾는다
auto blackboard = BT::Blackboard::create();
blackboard->set<rclcpp::Node::SharedPtr>("node", node);
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);      // 1000 / tick_hz
blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);
bt_mission::TaskServer::prepareBlackboard(blackboard);                     // 점 목록 항목 (3-3)

// 3. 트리 생성 (bt_navigator 가 active 가 된 뒤에). 트리는 한 번만 만들고 작업마다 다시 쓴다
waitForActive(node, "bt_navigator", g_stop);
BT::Tree tree = factory.createTreeFromFile(bt_xml, blackboard);

// 4. 액션 서버 + tick 루프
bt_mission::TaskServer server(node, tree, blackboard, log_transitions);
rclcpp::WallRate rate(tick_hz);                                            // 기본 100Hz
while (!g_stop) {                                                          // Ctrl-C 까지
  server.step();                                                           // 요청 처리 + tick 한 번 (4-4)
  rate.sleep();
}
server.stop("task_server shutting down");                                  // 실행 중인 작업을 멈추고 알린다
```

나머지 부분은 이렇다.

- **bt_navigator 대기 (`waitForActive`)**: Nav2 BT 노드는 트리를 만들 때 액션 서버를 찾는다. 또 bt_navigator 가 active 가 되기 전에 보낸 goal 은 거절되어, 작업이 바로 실패한다. 그래서 트리를 만들기 전에 `bt_navigator/get_state` 로 active 인지 확인한다.
- **Ctrl-C**: `rclcpp::init(..., SignalHandlerOptions::None)` 으로 rclcpp 의 기본 처리를 끄고 직접 받는다. rclcpp 가 먼저 종료되면 `haltTree()` 가 Nav2 에 취소 요청을 보낼 수 없기 때문이다.
- **TransitionLogger**: 노드 상태가 바뀔 때마다 `[bt] 이름 종류 이전 -> 다음` 을 출력한다. `BT::StatusChangeLogger` 를 상속한 작은 클래스다 (`src/bt_common.cpp`). 지금 RUNNING 인 노드와 마지막으로 실패한 노드도 기억해서, feedback 과 result 에 쓴다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `bt_xml` | `task.xml` | 실행할 XML. 파일 이름만 주면 `share/bt_mission/behavior_trees/` 에서 찾는다 |
| `tick_hz` | 100.0 | tick 주기 [Hz] |
| `log_transitions` | true | 노드 상태 변화 출력 |

### 3-3. Nav2 BT 노드를 쓸 때 알아둘 것

| XML 태그 | 플러그인 라이브러리 | 보내는 액션 | 주요 포트 |
|---|---|---|---|
| `NavigateThroughPoses` | `nav2_navigate_through_poses_action_bt_node` | `navigate_through_poses` | `goals` |
| `NavigateToPose` | `nav2_navigate_to_pose_action_bt_node` | `navigate_to_pose` | `goal` |
| `Spin` | `nav2_spin_action_bt_node` | `spin` | `spin_dist`, `time_allowance`, `is_recovery` |
| `Wait` | `nav2_wait_action_bt_node` | `wait` | `wait_duration` |

- **tick 은 100Hz 정도로.**
  - tick 을 10Hz 로 돌렸을 때, 로봇이 멈춘 뒤 BT 가 SUCCESS 를 받기까지 매번 0.9~1.1초가 걸렸다. 100Hz 에서는 이 지연이 사라졌다. 예전 예제 미션(좌표를 XML 에 적은 미션, 지금은 지웠다)의 시간이 63.7~65.5초에서 60.1~61.5초가 됐다.
  - 원인은 이렇게 보인다. Nav2 BT 노드는 tick 마다 액션 메시지를 조금씩(대개 하나)만 처리한다. bt_navigator 가 feedback 을 빠르게 보내서, 결과 메시지가 그 뒤에 밀렸다.
  - bt_navigator 자신도 기본 10ms(100Hz) 로 tick 한다.
- **점 목록 포트는 타입을 적지 않았다** (`ForEachWaypoint` 의 `poses`).
  - Nav2 Humble 1.1.18 이하의 `NavigateThroughPoses` BT 노드는 `goals` 포트 타입을 `PoseStamped` 로 잘못 선언하고 있다 (1.1.19 부터 `std::vector<PoseStamped>`).
  - 점 목록을 `std::vector<PoseStamped>` 로 선언하면, 같은 blackboard 변수를 `NavigateThroughPoses` 에도 넘기는 트리를 만들 때 타입이 다르다는 오류가 난다. 멈추지 않고 지나가는 가지를 더하는 경우다.
  - 같은 이유로 서버는 트리를 만들기 전에 `waypoints` 를 타입 없는 항목으로 만들어 둔다 (4-4 의 `prepareBlackboard()`).

---

## 4. 액션으로 작업 맡기 (`task_server`, `task.xml`)

실제 작업에서는 사람이나 관제 서버가 실행 중에 "어디로 가서 무엇을 하라" 를 보낸다. 이 예제는 그것을 **액션**으로 받는다. 액션에는 수락 여부, 진행 상황(feedback), 결과, 취소가 표준으로 있다. bt_navigator 가 `navigate_to_pose` 를 액션으로 받는 것과 같은 구조다.

```
task_client.py ──goal: ExecuteTask (Task)────▶ task_server (액션 서버 + BT 실행기)
   ◀── 수락 / 거절                               goal 마다 blackboard 를 채우고 task.xml 을 처음부터 tick
   ◀── feedback: BT 상태, 실행 중인 노드,         cancel  → haltTree → Nav2 goal 취소 → CANCELED
                 도착한 점 수                     새 goal → 지금 작업 ABORTED("preempted by …") → 새 작업
   ◀── result: success, message, BT 상태          트리 끝 → SUCCEEDED / ABORTED("<노드> failed at waypoint i/n")
   ── cancel ──▶                                    │ 점마다 NavigateToPose, Spin
                                                    ▼
                                         bt_navigator → planner / controller (그대로)
```

| 누가 | 무엇을 정하나 |
|---|---|
| 보내는 쪽 (`task_client.py`, 관제 서버) | **데이터**: 어디로(waypoints), 무엇을(action). 여러 작업의 순서 |
| 미션 BT (`task.xml`) | **정책**: action 별로 어떻게 움직이고 무엇을 할지, 실패하면 어떻게 할지 |
| 서버 (`task_server`) | 작업을 받아 트리를 돌리고, 진행 상황과 결과를 돌려준다. 취소, 선점 |
| 아래층 (bt_navigator, planner, controller) | 그대로. `NavigateToPose`, `Spin` 으로 부른다 |

| 액션이 주는 것 | 이 예제에서 |
|---|---|
| 받았는지 | goal 수락 / 거절 응답 (점이 없는 작업은 거절) |
| 진행 상황 | feedback (`task_id`, `bt_status`, `running`, `waypoints_reached`, `waypoints_total`, `elapsed`) |
| 결과 | goal 상태 SUCCEEDED / ABORTED / CANCELED + result (`success`, `message`, `bt_status`) |
| 취소 | cancel → BT halt → Nav2 goal 취소 |
| 실행 중에 새 작업 | **선점**: 지금 작업을 멈추고 새 작업. 차례대로 하려면 보내는 쪽이 하나 끝나면 다음을 보낸다 (`task_client.py` 가 그렇게 한다) |
| 명령줄 | `ros2 action send_goal --feedback` (4-5-5) |

### 4-1. ExecuteTask 액션과 Task 메시지

작업 하나 (`bt_mission_msgs/msg/Task.msg`):

```
string id                                 # 작업 이름 (로그와 mission/status 에 씀). 비우면 서버가 붙인다
geometry_msgs/PoseStamped[] waypoints     # 들를 점들. 1개 이상. orientation 은 그 점에 도착했을 때 볼 방향
string action                             # go | inspect | scan | look_around. 그 외 값이면 이동만
float64 duration                          # inspect 의 작업 시간 [s]
```

ROS 2 에서는 새 메시지 형식을 별도 패키지에 정의하는 것이 관례라서, 메시지만 담은 `bt_mission_msgs` 패키지를 따로 두었다.

액션 (`bt_mission_msgs/action/ExecuteTask.action`):

```
# ---- goal ----
Task task                     # 작업 내용 (Task.msg: id, waypoints, action, duration)
---
# ---- result ----
bool success
string message                # "done", 또는 실패/중단 이유
                              # ("go_inspect failed at waypoint 2/3", "preempted by t2", "canceled")
string bt_status              # 트리(task.xml 의 루트)가 마지막으로 돌려준 상태
                              #   SUCCESS | FAILURE | RUNNING (끝나기 전에 멈춤) | IDLE (tick 안 함)
---
# ---- feedback ----
string task_id
string bt_status              # 트리(루트)의 상태. 실행 중이므로 RUNNING
string running                # 지금 RUNNING 인 액션 노드 (이름 순, 예: "go_scan, scanning")
uint32 waypoints_reached      # 지금까지 도착한 점 수 (0 ~ waypoints_total)
uint32 waypoints_total        # 이 작업의 점 수
float32 elapsed               # 작업 시작 후 지난 시간 [s]
```

- goal 은 위의 `Task` 메시지를 그대로 담는다.
- goal 하나는 작업 하나다. 여러 작업의 순서와 조합은 보내는 쪽이 정한다. 미션 전체를 goal 하나로 보내면 "2번째 작업만 취소" 같은 일이 어렵다.
- 점에 도착할 때마다 `waypoints_reached` 가 1씩 는다. 클라이언트는 이 값이 바뀌면 "도착" 으로 출력한다.
- `bt_status` 는 BT 의 상태 그대로다 (SUCCESS / RUNNING / FAILURE). goal 상태(SUCCEEDED / ABORTED / CANCELED)는 액션의 상태라서 둘을 같이 보낸다. 대응은 4-2 에 있다.

### 4-2. task.xml

![task BT](bt_mission/doc/task.png)

작업 하나를 실행하는 트리다. 작업을 기다리고, 결과를 보고하고, 다음 작업을 받는 일은 서버가 맡는다. 그래서 트리에는 "작업 하나를 어떻게 하나" 만 남는다.

**`Switch3`** 가 `{action}` 값으로 가지 하나를 고른다.

| action | 가지 |
|---|---|
| `inspect` | 점을 차례로 들른 뒤 → `DoWork` (duration 초) |
| `scan` | `Parallel` (점을 차례로 들르기, `ScanWhileMoving`) |
| `look_around` | 점을 차례로 들른 뒤 → `Spin` 한 바퀴 |
| 그 외 (`go` 등) | 점을 차례로 들르기만 |

**점마다 멈추기: `ForEachWaypoint`**

"점을 차례로 들르기" 는 직접 만든 데코레이터 `ForEachWaypoint` 가 한다. 점마다 자식 `NavigateToPose` 를 한 번씩 실행한다.

```xml
<ForEachWaypoint name="go_points" poses="{waypoints}" pose="{waypoint}" task_id="{task_id}">
  <NavigateToPose name="go_only" goal="{waypoint}"/>
</ForEachWaypoint>
```

```cpp
BT::NodeStatus ForEachWaypoint::tick()       // src/mission_nodes.cpp, 발췌
{
  if (status() == BT::NodeStatus::IDLE) {    // 처음 tick: 점 목록을 읽고 첫 점을 {waypoint} 에
    getInput("poses", poses_);
    index_ = 0;
    setOutput("pose", poses_[0]);
  }
  while (true) {
    const auto child_status = child_node_->executeTick();   // NavigateToPose
    if (child_status == BT::NodeStatus::RUNNING) return BT::NodeStatus::RUNNING;
    haltChild();                             // 자식을 IDLE 로 (다음 점은 새 goal 로 시작)
    if (child_status == BT::NodeStatus::FAILURE) return BT::NodeStatus::FAILURE;
    ++index_;
    setOutput("reached", static_cast<int>(index_));
    reportArrival();                         // 로그 + mission/status "t1 arrived 2/3"
    if (index_ == poses_.size()) return BT::NodeStatus::SUCCESS;
    setOutput("pose", poses_[index_]);       // 다음 점. 같은 tick 에 바로 출발
  }
}
```

- 점 하나에 `NavigateToPose` 하나다. 그래서 로봇은 점마다 멈춰서 **위치와 방향을 맞추고**(goal checker: 0.05m, 약 3°), 도착을 알린 뒤 다음 점으로 간다.
- 중간 점에서 실패하면 그 자리에서 FAILURE 다. 남은 점으로는 가지 않는다.
- 도착을 알릴 때 TF 로 읽은 실제 로봇 위치와의 거리도 로그에 남긴다 (`[waypoint] t1 arrived 1/3 (0.00, -1.00), robot 0.035 m away`).
- `reached` 출력에는 지금까지 도착한 점 수를 쓴다. 서버는 이 값을 feedback 으로 보낸다.

**`NavigateThroughPoses` 를 쓰지 않는 이유**: `NavigateThroughPoses` 는 점들을 이은 꺾은선 하나로 달린다. 앞의 점은 0.7m 안에 들어오면 지난 것으로 치고, 그 자리에서 다음 점으로 새 경로를 긋는다 ([MULTI_GOAL.md 4장](../simple_line/MULTI_GOAL.md#경유-goal-을-정확히-밟지는-않는다)). 실측으로 경유점을 0.23~0.58m 거리로 지나갔다. 그래서 점마다 도착해야 하는 작업 명령에는 맞지 않다. 멈추지 않고 지나가도 되는 점이라면 `NavigateThroughPoses` 가 맞다.


- **트리 결과와 goal 상태**: 트리가 SUCCESS 면 goal 은 SUCCEEDED, FAILURE 면 ABORTED 다. 트리 상태는 result 의 `bt_status` 로도 간다.

  | 언제 | goal 상태 | `bt_status` |
  |---|---|---|
  | 실행 중 (feedback) | - | RUNNING |
  | 트리가 SUCCESS | SUCCEEDED | SUCCESS |
  | 트리가 FAILURE (이동 실패 등) | ABORTED | FAILURE |
  | 끝나기 전에 취소 | CANCELED | RUNNING |
  | 끝나기 전에 선점, 서버 종료 | ABORTED | RUNNING (`task_server_lifecycle` 의 선점은 빈 값, 5-3) |
- **blackboard**: `{waypoints}` `{task_id}` `{action}` `{duration}` 은 서버가 goal 을 받을 때 채운다. `{waypoint}` (지금 가는 점) 와 `{waypoints_reached}` 는 `ForEachWaypoint` 가 쓴다.
- **장애물**: 트리는 장애물을 다루지 않는다. 길이 막히면 Nav2 아래의 collision_monitor 가 로봇을 세우고, `NavigateToPose` 는 그동안 RUNNING 으로 기다린다 (6장).

### 4-3. 명령마다 BT 가 다르게 움직이는 이유

`go 0,-1 0,1.5` 를 보내면 `NavigateToPose` 가 두 번 실행되고, `inspect:5 1,0,0` 을 보내면 한 번 실행된 뒤 5초 작업을 한다. 트리가 명령에 맞춰 바뀌는 것처럼 보이지만, **트리 모양은 바뀌지 않는다.** 바뀌는 것은 blackboard 에 든 값이다.

```
서버가 시작할 때 task.xml 로 트리를 한 번 만든다. 이후 모든 goal 이 같은 트리를 쓴다.

goal 도착 ─▶ 서버가 blackboard 에 쓴다  (TaskServer::start → writeTask, src/bt_common.cpp)
              {action}="inspect"  {waypoints}=[(1,0)]  {duration}=5  {task_id}="t2"
          ─▶ 루트부터 tick
              Switch3            {action} 을 읽고 "inspect" 가지 하나만 실행
               └ Sequence
                  ├ ForEachWaypoint  {waypoints} 를 읽고 점 수(1)만큼 자식을 다시 실행
                  │   └ NavigateToPose   {waypoint} 를 읽어 goal 로 보낸다
                  └ DoWork           {duration} 을 읽고 5초
          ─▶ 끝나면 haltTree(): 모든 노드를 IDLE 로 (다음 goal 은 새 값으로 처음부터)
```

| 보낸 명령 | blackboard | 실행되는 노드 (차례대로) |
|---|---|---|
| `go 0,-1 0,1.5` | action=`go`, waypoints=2개 | Switch3 → (그 외 가지) ForEachWaypoint → NavigateToPose (0,-1) → 도착 → NavigateToPose (0,1.5) → 도착 |
| `inspect:5 1,0,0` | action=`inspect`, waypoints=1개, duration=5 | Switch3 → inspect 가지 → ForEachWaypoint → NavigateToPose (1,0) → 도착 → DoWork 5초 |
| `look_around -1,1.5` | action=`look_around`, waypoints=1개 | Switch3 → look_around 가지 → ForEachWaypoint → NavigateToPose (-1,1.5) → 도착 → Spin |

이렇게 되는 이유는 세 가지다.

1. **포트를 `{이름}` 으로 적으면 blackboard 값을 가리킨다.** `variable="{action}"` 은 "blackboard 의 action 값" 이라는 뜻이다 (2장). 노드는 실행할 때 `getInput()` 으로 그 순간의 값을 읽는다. 그래서 같은 노드라도 들어 있는 값에 따라 다르게 동작한다.
2. **노드가 그 값으로 무엇을 할지 정한다.**
   - `Switch3` 는 시작할 때 `{action}` 과 `case_1..3` 을 비교해서 가지 하나만 실행한다.
   - `ForEachWaypoint` 는 `{waypoints}` 의 점 수만큼 자식을 다시 실행하고, 매번 `{waypoint}` 에 다음 점을 써서 `NavigateToPose` 의 goal 을 바꾼다.
   - `DoWork` 는 `{duration}` 만큼 기다린다.
3. **작업이 끝날 때마다 트리를 초기화한다.** 서버가 `haltTree()` 로 모든 노드를 IDLE 로 돌린다. 다음 goal 은 새 값으로 루트부터 다시 시작한다.

bt_navigator 도 같은 방식이다. `navigate_to_pose` goal 이 오면 목표 위치를 blackboard 의 `{goal}` 에 쓰고, 같은 navigation BT 를 goal 마다 다시 돌린다 (1장, [MULTI_GOAL.md 1-1](../simple_line/MULTI_GOAL.md#1-1-재계획-주기0333hz와-지난-goal-반경07m은-어디에-있나)).

명령으로 바꿀 수 없는 것도 있다. **어떤 action 이 있는지(가지의 종류)와 각 가지의 노드 배치**는 XML 이 정한다. 새 action 을 더하려면 XML 에 가지를 더한다. 이미 있는 노드만 쓰면 다시 빌드할 필요는 없고 서버만 다시 띄운다. 다른 XML 을 쓰려면 `bt_xml` 파라미터로 준다.

### 4-4. TaskServer (`src/task_server.cpp`)

실행 노드(`src/task_server_main.cpp`)의 준비 과정은 3-2 에 있다.

1. 노드 종류를 등록한다.
2. blackboard 를 만든다.
3. bt_navigator 를 기다렸다가 트리를 만든다.

그다음 `TaskServer::step()` 을 100Hz 로 부른다. step 한 번이 하는 일은 이렇다 (발췌).

```cpp
void TaskServer::step()
{
  if (next_) {                           // 지난 step 에서 수락한 goal 을 시작한다
    if (active_) {                       // 선점: 지금 작업을 멈추고 새 작업으로
      finish(Outcome::Aborted, "preempted by " + next_->get_goal()->task.id);
    }
    start(next_);                        // blackboard 에 waypoints, task_id, action, duration
    next_.reset();
  }
  executor_.spin_some();                 // goal, cancel 요청 처리 (콜백은 next_ 에 goal 을 넣기만 한다)
  if (!active_) return;
  if (active_->is_canceling()) {
    finish(Outcome::Canceled, "canceled");
    return;
  }
  const auto status = tree_.tickRoot();
  bt_status_ = status;                   // feedback, result 의 bt_status
  if (status == BT::NodeStatus::RUNNING) {
    publishFeedback();                   // 실행 중인 노드나 도착한 점 수가 바뀌었을 때, 아니면 0.5초마다
  } else if (status == BT::NodeStatus::SUCCESS) {
    finish(Outcome::Succeeded, "done");
  } else {                               // "go_only failed at waypoint 2/3"
    finish(Outcome::Aborted, failureMessage(logger_.lastFailed(), waypointsReached(blackboard_), total_));
  }
}

void TaskServer::finish(Outcome outcome, const std::string & message)
{
  tree_.haltTree();                      // 실행 중인 노드를 멈추고(Nav2 goal 취소) 다음 작업을 위해 초기화
  ... active_->succeed(result) / active_->canceled(result) / active_->abort(result)
}
```

- **콜백은 누가 처리하나**: 액션 서버의 콜백은 서버만의 callback group 에 넣고, `step()` 에서 `spin_some()` 으로 처리한다 (Nav2 BT 노드가 액션 결과를 받는 것과 같은 방식). 콜백과 tick 이 같은 스레드에서 돌아서 잠금이 필요 없다.
- **수락한 goal 은 다음 step 에 시작한다**: 수락한 step 에서 바로 시작하면, 첫 feedback 이 수락 응답보다 먼저 클라이언트에 도착한다. 그러면 클라이언트는 아직 모르는 goal 의 feedback 이라며 버린다. 테스트에서 이 문제를 찾아서 고쳤다.
- **feedback 의 `running`**: `TransitionLogger` (`bt_common.cpp`) 가 상태 변화를 보면서, 지금 RUNNING 인 액션 노드와 마지막으로 실패한 노드를 기억한다. 실패한 노드 이름은 result 의 `message` 가 된다. 점을 다 들르지 못했으면 몇 번째 점으로 가다가 실패했는지도 붙인다.
- **feedback 의 `waypoints_reached`**: `ForEachWaypoint` 가 blackboard 의 `{waypoints_reached}` 에 쓴 값을 읽어 보낸다.
- **트리를 goal 마다 다시 만들지 않는다**: 트리는 한 번 만들고, 작업이 끝날 때마다 `haltTree()` 로 초기화해서 다시 쓴다 (4-3). Nav2 BT 노드는 만들 때 액션 서버를 찾느라 시간이 걸리기 때문이다.
- **`prepareBlackboard()`**: 트리를 만들기 전에 `waypoints` 를 타입 없는 항목으로 만들어 둔다 (3-3).
- **서버 종료 (Ctrl-C)**: 실행 중인 작업을 멈추고(Nav2 goal 취소), 클라이언트에 ABORTED(`"task_server shutting down"`)를 보낸 뒤 끝난다.

### 4-5. 명령 내리기: 입력 형식과 예시

작업은 세 가지 방법으로 보낼 수 있다.

| 방법 | 언제 | 아래 |
|---|---|---|
| `task_client.py` (대화형) | 터미널에서 여러 작업을 차례로, 선점, 취소 | 4-5-1 ~ 4-5-3 |
| `task_client.py <작업>` (한 번) | 스크립트에서 작업 하나를 보내고 결과를 종료 코드로 | 4-5-4 |
| `ros2 action send_goal` | 클라이언트 없이 메시지를 직접 | 4-5-5 |

#### 4-5-1. 한 줄 형식

```
<action>[:<duration>]  <점>  [<점> ...]

점 = x,y          방향은 자동으로 채운다
     x,y,yaw      yaw: 그 점에 도착했을 때 볼 방향 [도]
```

| 부분 | 쓰는 법 | 예 |
|---|---|---|
| `action` | `go`, `inspect`, `scan`, `look_around` 중 하나. 꼭 적는다 | `inspect` |
| `:duration` | `inspect` 의 작업 시간 [초]. 생략하면 3초. 다른 action 에서는 쓰이지 않는다 | `inspect:5` |
| 점 | `x,y` 또는 `x,y,yaw`. map 좌표 [m], yaw 는 **도(°)** | `1.5,-1`  `1.5,-1,90` |
| 띄어쓰기 | action 과 점, 점과 점 사이는 공백. **점 안에는 공백 없이** 쉼표만 | `go 0,-1 1,1` |
| 주석 | `#` 뒤는 무시 | `go 1,1  # 창고 앞` |

| action | 로봇이 하는 일 |
|---|---|
| `go` | 점을 차례로 들른다 |
| `inspect` | 점을 차례로 들른 뒤, 마지막 점에서 서서 `duration` 초 작업 |
| `scan` | 점을 차례로 들르는 동안 0.5m 마다 스캔 (마지막 점에 도착하면 멈춤) |
| `look_around` | 점을 차례로 들른 뒤, 마지막 점에서 제자리 한 바퀴 |

어느 action 이든 로봇은 **점마다 멈춰서 위치와 방향을 맞추고**, 도착을 알린 뒤 다음 점으로 간다. 출발할 때는 다음 점 쪽으로 먼저 다 돌고 나서 직선으로 간다.

**좌표와 방향** (빈 방 기준)

```
          y
     3 ───┼──────────────┐ 벽          yaw (도착했을 때 볼 방향)
          │              │                   90
          │      ·(0,0)  │                    ▲
          │              │            180 ◀───┼───▶ 0
          │ ▶ 출발       │                    ▼
    -3 ───┼──────────────┘                  -90
         -3              3  x
   출발 위치: (-2, -1), yaw 0°.  벽은 ±3m, 로봇 반지름을 생각해 ±2.5m 안을 쓴다
   (점은 벽, 물체에서 0.4m 넘게. 더 가까우면 controller 나 collision_monitor 가 벽 앞에서 세운다, 9장)
```

**yaw 를 생략하면** 이렇게 채운다.

- 중간 점: **다음 점 쪽**. 도착하면 다음 점을 향해 돌아 두고, 다음 출발 때 회전이 없다.
- 마지막 점: **직전 점에서 온 방향**. 따로 돌지 않고 선다.
- 작업의 첫 점은 직전 작업의 마지막 점(첫 작업이면 로봇의 현재 위치)에서 온 것으로 본다.

#### 4-5-2. 예시

| 입력 | 로봇이 하는 일 |
|---|---|
| `go 1,-1` | (1,-1) 로 가서 선다. 방향은 온 방향 |
| `go 1,-1,90` | (1,-1) 로 가서 +y 쪽(90°)을 보고 선다 |
| `go 0,-1 0,1 -2,1` | (0,-1) → (0,1) → (-2,1) 을 차례로. 점마다 서서 다음 점 쪽으로 돈 뒤 출발 |
| `go 0,-1 0,1 -2,-1,0` | 세 점을 돌고 마지막 점에서 +x 쪽(0°)을 보고 선다 |
| `inspect 1.5,-1,90` | (1.5,-1) 에서 90° 를 보고 서서 3초 점검 |
| `inspect:10 0,-1 1.5,-1,90` | (0,-1) 에 들렀다가 (1.5,-1) 에서 90° 를 보고 10초 점검 |
| `scan 1.5,1.5 -1,1.5,180` | 두 점을 들르는 동안 0.5m 마다 스캔 |
| `look_around -1,1.5` | (-1,1.5) 에 가서 제자리 한 바퀴 |

#### 4-5-3. 대화형 (`task_client.py`)

```bash
ros2 run bt_mission task_client.py
```

| 입력 | 뜻 |
|---|---|
| `<작업 한 줄>` | 작업을 더한다. 실행 중인 작업이 있으면 끝난 뒤에 보낸다 (차례대로) |
| `!<작업 한 줄>` | 지금 작업을 멈추고(선점) 바로 보낸다. 예) `!go -2,-1,0` |
| `cancel` | 지금 작업을 취소하고, 기다리는 작업도 비운다 |
| `help` | 도움말 |
| `quit`, Ctrl-D | 끝낸다. 남은 작업이 있으면 다 끝날 때까지 기다린다 |
| Ctrl-C | 지금 작업을 취소하고 끝낸다 |

```
task> go 0,-1 0,1.5 -2,1.5 -2,-1,0
  -> t1: go (0,-1) -> (0,1.5) -> (-2,1.5) -> (-2,-1)
  .. t1: RUNNING go_only (0.0 s)
  .. t1: arrived 1/4 (0,-1) (12.8 s)
  .. t1: arrived 2/4 (0,1.5) (27.7 s)
  .. t1: arrived 3/4 (-2,1.5) (40.5 s)
  .. t1: arrived 4/4 (-2,-1)
  <- t1 SUCCEEDED: done [BT SUCCESS]
```

| 출력 | 뜻 |
|---|---|
| `-> t1: ...` | 작업 t1 을 보냄 (id 는 클라이언트가 t1, t2, ... 로 붙인다) |
| `.. t2 queued (1 waiting)` | 앞 작업이 끝나기를 기다리는 중 |
| `.. t1: RUNNING go_only (0.0 s)` | 트리 상태와 지금 실행 중인 BT 노드 (feedback 의 `bt_status`, `running`) |
| `.. t1: arrived 2/4 (0,1.5) (27.7 s)` | 4개 중 2번째 점에 도착 |
| `<- t1 SUCCEEDED: done [BT SUCCESS]` | 결과: goal 상태, message, 트리 상태 (result 의 `bt_status`) |
| `<- t1 CANCELED: canceled [BT RUNNING]` | 끝나기 전에 취소됨. 트리는 RUNNING 이었다 |

자주 하는 실수 (실제 메시지):

| 입력 | 메시지 | 고칠 점 |
|---|---|---|
| `go 1, -1` | `'1,' is not x,y or x,y,yaw_deg` | 점 안에는 공백 없이: `go 1,-1` |
| `go 1,-1 2` | `'2' is not x,y or x,y,yaw_deg` | 점마다 x 와 y 를 다 적는다 |
| `move 1,1` | `unknown action 'move' (one of go, inspect, scan, look_around)` | action 이름 확인 |
| `go` | `need an action and at least one point` | 점을 하나 이상 |
| `inspect:abc 1,1` | `duration 'abc' is not a number (e.g. inspect:5)` | 작업 시간은 숫자 [초] |
| `go 1,-1,1.57` | (오류 없음) | yaw 는 라디안이 아니라 **도**다. 1.57° 로 해석된다. 90° 면 `go 1,-1,90` |

#### 4-5-4. 작업 하나만 보내기

```bash
# 끝날 때까지 기다렸다가 종료. 종료 코드 0 = SUCCEEDED, 1 = 그 외
ros2 run bt_mission task_client.py inspect 0,-1 1.5,-1,90 && echo ok

# 여러 작업을 파일에서 (한 줄에 하나, 차례대로 실행하고 모두 끝나면 종료)
ros2 run bt_mission task_client.py < tasks.txt
```

#### 4-5-5. `ros2 action send_goal` 로 직접

클라이언트 없이 goal 메시지를 직접 적는다. 필드는 4-1 의 `ExecuteTask` 와 `Task` 다.

```bash
ros2 action send_goal --feedback /execute_task bt_mission_msgs/action/ExecuteTask "
task:
  id: b1                                   # 작업 이름 (아무 문자열)
  action: inspect                          # go | inspect | scan | look_around
  duration: 2.0                            # inspect 의 작업 시간 [s]
  waypoints:                               # 들를 점, 차례대로
  - header: {frame_id: map}
    pose:
      position: {x: -2.0, y: 0.0}
      orientation: {z: 1.0, w: 0.0}        # yaw 180°
  - header: {frame_id: map}
    pose:
      position: {x: -2.0, y: -1.0}
      orientation: {z: 0.0, w: 1.0}        # yaw 0°
"
```

같은 내용을 한 줄로 (점 두 개, `go`):

```bash
ros2 action send_goal --feedback /execute_task bt_mission_msgs/action/ExecuteTask \
  "{task: {id: a1, action: go, waypoints: [{header: {frame_id: map}, pose: {position: {x: -1.0, y: -1.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: -1.0, y: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}]}}"
```

**방향(yaw)은 quaternion 으로 적는다.** 바닥에서 도는 로봇이면 `z = sin(yaw/2)`, `w = cos(yaw/2)` 이고 x, y 는 0 이다.

| yaw | `orientation` |
|---|---|
| 0° (+x) | `{z: 0.0, w: 1.0}` |
| 45° | `{z: 0.3827, w: 0.9239}` |
| 90° (+y) | `{z: 0.7071, w: 0.7071}` |
| 180° (-x) | `{z: 1.0, w: 0.0}` |
| -90° (-y) | `{z: -0.7071, w: 0.7071}` |

- `orientation` 을 안 적으면 `{w: 1.0}`, 즉 0° 다. `ros2 action send_goal` 은 yaw 를 채워 주지 않는다. 그래서 점마다 방향을 적지 않으면 로봇이 점마다 0° 쪽으로 돌았다가 다음 점으로 간다. 방향을 자동으로 채우려면 `task_client.py` 를 쓴다.
- `frame_id` 를 비우면 서버가 `map` 으로 채운다.
- 점이 없는 goal 은 거절된다 (`Goal was rejected.`).

출력 (실제, 줄임):

```
Goal accepted with ID: 2224ae7a6b0a4646abc9419ec63832e2

Feedback:
    task_id: a1
bt_status: RUNNING
running: go_only
waypoints_reached: 0
waypoints_total: 2
elapsed: 0.0006170350243337452
...
Feedback:
    task_id: a1
bt_status: RUNNING
running: go_only
waypoints_reached: 1
waypoints_total: 2
elapsed: 6.731001377105713
...
Result:
    success: true
message: done
bt_status: SUCCESS

Goal finished with status: SUCCEEDED
```

### 4-6. 클라이언트가 하는 일 (`scripts/task_client.py`)

- **한 줄 해석**: `parse_line()` 이 한 줄을 action 과 점으로 나누고, `fill_yaws()` 가 적지 않은 yaw 를 채운다.
- **차례대로 실행**: 서버는 새 goal 이 오면 선점하므로, 기다리는 작업은 클라이언트가 큐에 들고 있다가 앞 작업의 result 를 받으면 다음 goal 을 보낸다.
- **도착 출력**: feedback 의 `waypoints_reached` 가 늘면 그 점을 "arrived" 로 출력한다. 마지막 점에 도착하면서 끝나는 작업(`go`, `scan`)은 그 도착을 담은 feedback 없이 결과가 오므로, SUCCEEDED 를 받으면 남은 도착을 출력한다.
- **BT 상태**: 실행 중인 노드 앞에 feedback 의 `bt_status`(RUNNING)를, 결과 뒤에 result 의 `bt_status` 를 `[BT SUCCESS]` 처럼 붙인다.
- **Ctrl-C**: 지금 작업에 cancel 을 보내고, CANCELED 를 받은 뒤 끝낸다.
  - rclpy 기본 Ctrl-C 처리는 ROS 를 먼저 종료해서 cancel 을 보낼 수 없다.
  - 그래서 Ctrl-C 를 플래그로만 받고, 기다리는 루프에서 확인한다. 콜백 도중에 `KeyboardInterrupt` 가 나서 rclpy 내부가 꼬이는 일도 막는다.
- **TF 는 시작할 때 한 번만 읽는다**: 첫 작업의 yaw 를 채우려고 로봇 위치를 한 번 읽고 TF 구독을 끊는다. 계속 받으면 초당 수십 개의 TF 메시지가 feedback, result 처리와 경쟁한다.

### 4-7. 실행

```bash
# 터미널 1: 0장의 launch

# 터미널 2: 작업을 기다리는 액션 서버
ros2 run bt_mission task_server

# 터미널 3
ros2 run bt_mission task_client.py
```

클라이언트 출력 (가짜 로봇으로 실행한 실제 출력. 작업 4개를 한 번에 넣은 경우):

```
task>   -> t1: inspect (0,-1) -> (1.5,-1)
task>   .. t2 queued (1 waiting)
task>   .. t3 queued (2 waiting)
task>   .. t4 queued (3 waiting)
task> waiting for 4 task(s) to finish (Ctrl-C to cancel)
  .. t1: RUNNING go_inspect (0.0 s)
  .. t1: arrived 1/2 (0,-1) (10.8 s)
  .. t1: arrived 2/2 (1.5,-1) (21.8 s)
  .. t1: RUNNING work (21.8 s)
  <- t1 SUCCEEDED: done [BT SUCCESS]
  -> t2: scan (1.5,1.5) -> (-1,1.5)
  .. t2: RUNNING go_scan, scanning (0.0 s)
  .. t2: arrived 1/2 (1.5,1.5) (14.9 s)
  .. t2: arrived 2/2 (-1,1.5)
  <- t2 SUCCEEDED: done [BT SUCCESS]
  -> t3: look_around (-1,1.5)
  .. t3: RUNNING go_look (0.0 s)
  .. t3: arrived 1/1 (-1,1.5) (0.1 s)
  .. t3: RUNNING spin (0.1 s)
  <- t3 SUCCEEDED: done [BT SUCCESS]
  -> t4: go (-2,-1)
  .. t4: RUNNING go_only (0.0 s)
  .. t4: arrived 1/1 (-2,-1)
  <- t4 SUCCEEDED: done [BT SUCCESS]
```

선점 (`go 1.5,-1` 을 보내고 4초 뒤 `!go -2,-1`):

```
task>   -> t1: go (1.5,-1)
task>   .. t1: RUNNING go_only (0.0 s)
  -> t2: go (-2,-1)
task>   .. t2: RUNNING go_only (0.0 s)
  <- t1 ABORTED: preempted by t2 [BT RUNNING]
  .. t2: arrived 1/1 (-2,-1)
  <- t2 SUCCEEDED: done [BT SUCCESS]
```

취소 (`go 1.5,-1` 을 보내고 4초 뒤 `cancel`):

```
task>   -> t1: go (1.5,-1)
task>   .. t1: RUNNING go_only (0.0 s)
  .. canceling t1
task>   <- t1 CANCELED: canceled [BT RUNNING]
```

선점되거나 취소된 작업은 트리가 끝나기 전에 멈췄으므로 `[BT RUNNING]` 이다.

t3 는 t2 의 마지막 점과 같은 곳이라 바로 도착하고 회전한다.

### 4-8. 남은 것

- **실행은 한 번에 하나다.** 서버는 선점만 하고, 대기열은 보내는 쪽이 관리한다.
- **feedback 의 `running` 은 BT 노드 이름이다.** 사람이 읽을 단계 이름이 필요하면 노드 이름을 그렇게 짓거나, feedback 에 필드를 더한다.
- **Nav2 템플릿으로도 만들 수 있다.** Nav2 에는 이 구조를 위한 템플릿 `nav2_behavior_tree::BtActionServer` 가 있다 (bt_navigator 가 이것으로 만들어져 있다). 같은 일을 이 템플릿으로 한 버전과 비교는 5장에 있다.
- **점마다 서는 만큼 느려진다.** 7장의 작업 4개가 약 61초(경유점을 멈추지 않고 지나감)에서 78초(점마다 섬)가 됐다. 멈출 필요가 없는 점은 `NavigateThroughPoses` 로 지나가게 하는 가지를 `task.xml` 에 따로 두면 된다 (포트 타입은 3-3 을 본다).

---

## 5. Nav2 의 BtActionServer 로 만들기 (`task_server_lifecycle`)

4장의 `task_server` 는 액션 서버와 tick 루프를 직접 만들었다. Nav2 에는 같은 일을 하는 템플릿 `nav2_behavior_tree::BtActionServer` 가 있고, bt_navigator 도 `navigate_to_pose`, `navigate_through_poses` 를 이것으로 받는다. 같은 `task.xml` 과 ExecuteTask 액션으로 이 템플릿을 쓴 버전을 옆에 만들었다. 클라이언트(`task_client.py`, `ros2 action send_goal`)는 그대로 쓴다.

### 5-1. 실행

```bash
# 터미널 1: 0장의 launch
# 터미널 2: Nav2 가 뜬 뒤에. 서버와 이 서버만 맡는 lifecycle_manager 를 같이 띄운다
ros2 launch bt_mission task_server_lifecycle.launch.py
# 터미널 3
ros2 run bt_mission task_client.py
```

- lifecycle 노드라서 **configure → activate** 가 되어야 goal 을 받는다. launch 의 `lifecycle_manager_mission` 이 이 일을 한다.
- lifecycle_manager 없이 직접 할 때:
  ```bash
  ros2 run bt_mission task_server_lifecycle
  ros2 lifecycle set /task_server_lifecycle configure
  ros2 lifecycle set /task_server_lifecycle activate
  ```
- `task_server` 와 액션 이름(`/execute_task`)이 같으므로 둘 중 하나만 띄운다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `bt_xml` | `task.xml` | 실행할 트리 |
| `plugin_lib_names` | Nav2 플러그인 4개 + `bt_mission_plugin` | 트리에서 쓰는 BT 노드가 들어 있는 라이브러리 |
| `log_transitions` | true | 노드 상태 변화를 `[bt]` 로그로 출력 |
| `bt_loop_duration` | 10 | tick 주기 [ms] (BtActionServer 파라미터, bt_navigator 와 같은 이름) |
| `default_server_timeout` | 1000 | Nav2 BT 노드가 goal 응답 등을 기다리는 시간 [ms]. BtActionServer 기본은 20 인데 `task_server` 와 맞췄다 |
| `wait_for_service_timeout` | 5000 | 트리를 만들 때 Nav2 액션 서버를 찾는 시간 [ms]. BtActionServer 기본은 1000 |

### 5-2. 구조

```
task_server_lifecycle  (nav2_util::LifecycleNode, src/task_server_lifecycle.cpp)
 └─ BtActionServer<ExecuteTask>  (nav2_behavior_tree, Nav2 가 제공)
      ├─ SimpleActionServer   goal 수락, cancel, 실행 중에 온 goal 하나를 pending 으로 보관
      ├─ BehaviorTreeEngine   plugin_lib_names 의 .so 로 노드 등록, 작업 스레드에서 10ms 마다 tick
      ├─ blackboard           node, server_timeout, bt_loop_duration, wait_for_service_timeout
      └─ RosTopicLogger       BT 노드 상태 변화 → behavior_tree_log 토픽
    이 노드가 채우는 콜백: onGoalReceived, onLoop, onPreempt, onCompletion
```

| lifecycle 전이 | 하는 일 |
|---|---|
| configure | BtActionServer 생성 (액션 서버는 생기지만 아직 goal 을 거절). blackboard 에 `tf_buffer`, 타입 없는 `waypoints` 항목 (3-3) |
| activate | 트리 생성 (Nav2 BT 노드가 Nav2 액션 서버를 찾는다), goal 받기 시작, lifecycle_manager 와 bond 연결 |
| deactivate | 실행 중인 작업을 halt 하고 (Nav2 goal 취소) ABORTED("task server deactivated"). 이후 goal 은 거절 |
| cleanup | 트리, 액션 서버 정리 |

`task_server` 의 코드가 어디로 갔는지:

| `task_server` (4장) | `task_server_lifecycle` |
|---|---|
| 직접 만든 노드는 코드에서 등록 (`registerAllNodes`) | `plugin_lib_names` 로만 등록한다. 그래서 직접 만든 노드를 플러그인 `bt_mission_plugin` 으로 빌드한다 (`src/bt_plugin.cpp` 의 `BT_REGISTER_NODES`) |
| 수락 단계에서 점이 없는 goal 을 거절 | `onGoalReceived` 가 false 를 돌려주면 ABORTED |
| `start()`: blackboard 에 작업 쓰기 | `onGoalReceived` → `startTask()`. 쓰는 함수(`writeTask`)는 같다 |
| `step()`: tick, `publishFeedback()` | tick 은 BtActionServer, feedback 은 `onLoop` |
| `step()`: 새 goal 이면 지금 작업을 끝내고 새 작업 시작 | `onPreempt`: `haltTree()`, `acceptPendingGoal()`, `startTask()` |
| `step()`: cancel 이면 halt 후 CANCELED | BtActionServer 가 확인하고 halt. result 는 `onCompletion` 이 채운다 |
| `finish()`: halt 후 succeed / abort / canceled | BtActionServer 가 `haltAllActions` 후 `onCompletion` 이 채운 result 로 보낸다 |
| main: Ctrl-C 를 직접 받아 `stop()` | `nav2_util::LifecycleNode` 가 ROS 종료 직전에 deactivate → cleanup |
| main: bt_navigator 가 active 가 될 때까지 기다림 | 기다리지 않는다. activate 때 Nav2 액션 서버를 못 찾으면 activate 가 실패한다 |

### 5-3. 선점과 첫 feedback (`src/task_server_lifecycle.cpp`, 발췌)

BtActionServer 는 실행 중에 새 goal 이 오면 pending 으로 두고, tick 마다 `onPreempt` 를 부른다. 아무것도 하지 않으면 지금 작업이 끝난 뒤 이어서 실행된다 (대기열 1칸). bt_navigator 는 여기서 goal 위치만 바꾸고 트리를 계속 돌린다. 작업은 action 에 따라 가지가 달라지므로, 여기서는 트리를 처음부터 다시 돌린다.

```cpp
void TaskServerLifecycle::onPreempt(ExecuteTask::Goal::ConstSharedPtr goal)
{
  if (bt_server_->getTree().rootNode()->status() != BT::NodeStatus::RUNNING) {
    return;                              // 이번 tick 에 막 끝났으면 결과를 먼저 보낸다
  }
  ...
  bt_server_->haltTree();                // 지금 작업의 노드를 멈춘다 (Nav2 goal 취소)
  startTask(*bt_server_->acceptPendingGoal());   // 이전 goal 은 ABORTED (result 는 빈 값)
}
```

- **선점된 goal 의 result 는 빈 값이다.** `acceptPendingGoal()` 안에서 SimpleActionServer 가 빈 result 로 ABORTED 를 보낸다. `task_server` 처럼 `"preempted by t2"` 나 `bt_status` 를 실어 보낼 방법이 없다.
- **트리가 막 끝난 tick 에는 받지 않는다.** 그 tick 에 받으면, 끝난 작업의 SUCCESS 가 새 goal 의 결과로 보내진다.
- **첫 feedback 은 50ms 늦춘다.** BtActionServer 는 goal 을 수락하자마자 다른 스레드에서 tick 을 시작한다. 그래서 첫 feedback 이 수락 응답보다 먼저 도착하면, 클라이언트는 모르는 goal 의 것으로 버린다. gtest 를 10번 돌렸을 때 이렇게 버려진 feedback 이 8개였고, 50ms 늦춘 뒤에는 0개였다. `task_server` 는 수락한 goal 을 다음 step 에 시작해서 같은 문제를 피한다 (4-4).

### 5-4. 두 버전 비교

같은 날 같은 Nav2 서버에 같은 순서로 보낸 결과와, 같은 경우를 확인하는 gtest (7-1, 7-2) 결과다.
시간과 프로세스 수치는 점마다 멈추도록 바꾸기 전(`NavigateThroughPoses` 로 지나가던 때)에 쟀다. 두 버전의 차이를 보려는 것이라 그대로 두었다.

| | `task_server` (직접 구현) | `task_server_lifecycle` (BtActionServer) |
|---|---|---|
| 작업 4개 (7장) | 모두 SUCCEEDED, 61.1 s | 모두 SUCCEEDED, 60.6 s |
| 정상 / cancel / 이동 실패의 결과 | SUCCEEDED("done") / CANCELED("canceled") / ABORTED("go_look failed at waypoint 1/1") | 같음 |
| 선점된 작업 | ABORTED("preempted by t2"), bt_status RUNNING | ABORTED, message 와 bt_status 모두 빈 값 |
| 점이 없는 작업 | 수락 단계에서 REJECTED | 수락된 뒤 ABORTED, message 빈 값 |
| 작업 중 서버 Ctrl-C | ABORTED("task_server shutting down"). 신호 처리를 직접 짰다 | ABORTED("task server deactivated"). LifecycleNode 의 종료 처리 |
| 시작 | 실행하면 bt_navigator 가 active 가 될 때까지 기다린다 | configure → activate 가 필요하다 (lifecycle_manager, launch 부터 active 까지 3.7초). Nav2 가 없으면 activate 실패 |
| 서버가 죽었을 때 | 다른 노드는 알 수 없다 | lifecycle_manager 가 bond 로 4초 안에 알아챈다 (`SERVER task_server_lifecycle IS DOWN`) |
| tick 하는 곳 | 실행 노드의 메인 루프. 요청 처리와 tick 이 한 스레드 | SimpleActionServer 의 작업 스레드. 콜백도 이 스레드에서 불린다 |
| BT 상태 보기 | `[bt]` 로그 | `[bt]` 로그 + `behavior_tree_log` 토픽 (launch 에서 `mission/behavior_tree_log` 로 옮겼다. 안 옮기면 bt_navigator 의 것과 섞인다) |
| 직접 만든 BT 노드 | 코드에서 등록 | 플러그인 .so 로 빌드 |
| 테스트 | `step()` 을 직접 불러 한 단계씩 | 서버를 다른 스레드에서 spin. 가짜 Nav2 노드도 플러그인(`test_fake_nav2`)으로 |
| 코드 (주석 포함) | 342 줄 (헤더 83, cpp 181, main 78) | 356 줄 (헤더 75, cpp 215, main 24, 플러그인 12, launch 30) |
| 대기 중 프로세스 | 스레드 11, 메모리 49 MB | 스레드 12, 52 MB. lifecycle_manager 가 따로 스레드 12, 31 MB |

이 Humble 버전(1.1.18)의 BtActionServer 에는 Groot 모니터링이 없다. BT 상태는 `behavior_tree_log` 토픽으로만 나온다.

### 5-5. 어느 쪽을 쓸까

- **코드 양은 거의 같다.** BtActionServer 가 액션 서버, 스레드, 취소 처리를 맡아 주는 대신 lifecycle 콜백, 플러그인 라이브러리, launch 가 생긴다.
- **lifecycle 버전이 나은 경우**: Nav2 와 같이 켜고 끄고, 서버가 죽으면 알아야 할 때. Nav2 launch 를 직접 구성한다면 이 노드를 bt_navigator 와 같은 lifecycle_manager 의 `node_names` 뒤쪽에 넣어서, Nav2 가 active 가 된 다음에 activate 되게 할 수 있다.
- **`task_server` 가 나은 경우**: 선점 이유 같은 정보를 result 로 보내야 할 때, 흐름을 한 파일에서 처음부터 끝까지 보고 싶을 때, 테스트를 단순하게 하고 싶을 때.

---

## 6. 장애물: 멈췄다 가기 (stop & go)

길에 장애물이 있으면 로봇은 감속해서 그 앞에 서서 기다리고, 장애물을 치우면 같은 길로 다시 간다. **Nav2 goal 은 취소하지 않는다.** goal 은 그대로 실행 중(pending)이고, 로봇에 가는 속도만 0 이 된다.

이 일은 미션 BT 가 아니라 그 아래 두 곳에서 한다.

```
controller_server ─┐
                   ├─ /cmd_vel_nav ─▶ collision_monitor ─ /cmd_vel_safe ─▶ velocity_smoother ─ /cmd_vel ─▶ 로봇
behavior_server ───┘                       ▲                              가속 0.6, 비상 감속 1.0 m/s² 한계
  ▲                              /scan ────┘  로봇 둘레 원(0.3 m)이 지금 명령대로 1.2초 안에 닿으면 줄이거나 0 (비상)
  └ controller: local costmap 에서 경로 위 장애물까지 거리를 재고, S 커브로 감속해서 그 앞에 선다
```

- **평소: controller** (`SimpleLineController`). 경로를 따라가며 양옆 0.3 m 안에 레이저로 본 장애물(local costmap)이 있는 첫 지점까지 거리를 잰다. goal 앞에서 서는 것과 같은 S 커브(감속도 0.5 m/s², jerk 1 m/s³)로 감속해서, 로봇 중심에서 장애물까지 약 0.7 m 를 두고 선다. 로그: `obstacle on the path 0.xx m ahead, slowing down to stop`
- 장애물이 사라지고(또는 0.3 m 넘게 멀어지고) 1초가 지나면 다시 S 커브로 가속한다. 로그: `path clear, going again`
- **비상: collision_monitor** (`approach`). 로봇 둘레 원(반지름 0.3 m)을 지금 명령한 속도로 1.2초 앞까지 움직여 보고, 레이저 점에 닿으면 닿기까지 남은 시간에 비례해 속도를 줄인다. 원이 이미 닿아 있으면 0. velocity_smoother 가 1.0 m/s² 로 잇는다. controller 가 서기에는 너무 가까이 갑자기 나타났을 때(앞으로 끼어든 사람)만 동작한다. 로그: `Robot to approach for 1.200000 seconds away from collision`, 풀리면 `Robot to continue normal operation`
- **움직이는 방향만 본다.** 장애물 앞에 서 있어도 그쪽으로 가는 명령만 막고, 돌아서 나가는 회전이나 멀어지는 방향은 막지 않는다. 처음에는 로봇 앞 사각형(StopZone, `stop`)에 점이 있으면 회전까지 0 이 되어, 장애물 앞에 선 로봇에게 반대쪽 goal 을 줘도 돌지 못했다 ([AMR.md 5-2](../simple_line/AMR.md#5-2-비상-감속-움직이는-방향만-본다-collision_monitor-approach)).
- 그동안 Nav2 와 미션 BT 는 하던 일을 계속한다. `FollowPath`, `NavigateToPose`, 작업 모두 RUNNING 이다.
- 미션 BT(`task.xml`)에는 장애물 코드가 없다. RViz 의 Nav2 Goal, `ros2 action send_goal`, waypoint_follower 까지 controller 로 가는 모든 이동에 똑같이 적용된다. 비상 감속은 behavior_server 의 동작과 `cmd_vel_nav` 로 보낸 teleop 에도 적용된다.
- 처음에는 collision_monitor 에 SlowZone(앞 1 m 에서 0.3배)을 두고 감속까지 맡겼다. 속도가 구역 경계에서 한 번에 바뀌어 덜컹거리고(로봇에 간 명령의 jerk 12 m/s³), 장애물이 어디 있든 1 m 앞부터 기어가서 바꿨다. 비교와 가감속 알고리즘 정리는 [simple_line/AMR.md](../simple_line/AMR.md) 4, 5장에 있다.

![stop and go](bt_mission/doc/stop_and_go.png)

Gazebo 에서 (0,-1) 에 장애물(반지름 0.15 m)을 놓고 Nav2 goal (1,-1) 을 보냈다. 선 뒤 8초가 지나 치웠다 (7-3).

- **왼쪽**: 로봇은 0.25 m/s 로 가다가 필요한 거리에서만 S 커브로 감속해서 섰다 (로봇 중심에서 장애물까지 0.79 m 에서 시작해 0.88초, 10.5 cm 만에 섰다). 치우자 1.6초 뒤 다시 S 커브로 가속해서 goal 까지 갔다.
- **오른쪽**: 로봇 앞면에서 장애물 표면까지 0.61 m 를 두고 섰다. collision_monitor 의 원(주황 점선)이 지금 명령으로는 장애물에 닿지 않아 비상 감속은 동작하지 않았다. 경로는 처음 직선 그대로다.

### 6-1. 직접 해 보기 (Gazebo)

```bash
# 터미널 1: Gazebo + Nav2 + RViz. collision_monitor 도 같이 뜬다
ros2 launch simple_line_bringup simple_line.launch.py
# 터미널 2
ros2 run bt_mission task_server
# 터미널 3: 장애물 도구. RViz 에서 찍은 곳에 장애물을 놓고, 놓은 장애물을 다시 찍으면 치운다
ros2 run simple_line_bringup obstacle.py click
# 터미널 4
ros2 run bt_mission task_client.py
task> go 1,-1,0
```

1. 작업을 보내 로봇이 출발하면, RViz 위쪽 툴바의 **Publish Point** 를 누르고 로봇 앞 길 위(예: (0,-1) 근처)를 찍는다. Gazebo 에 빨간 원기둥이 생긴다. Publish Point 는 한 번 찍으면 풀리므로, 찍을 때마다 다시 누른다.
2. 원기둥이 레이저에 보이면(local costmap 에 표시되면) 로봇이 감속해서 그 앞 약 0.7 m 에 선다. 터미널 1 에 `obstacle on the path ... slowing down to stop` 이 나온다.
3. 원하는 만큼 기다린 뒤, Publish Point 로 그 원기둥을 다시 찍는다. 원기둥이 사라지고 `path clear, going again` 이 나온 뒤 로봇이 다시 간다.
4. 작업은 끝까지 가서 `SUCCEEDED` 로 끝난다. 30초 넘게 막혀 있으면 Nav2 가 재시도를 시작한다 (6-3).
5. 비상 감속을 보려면 달리는 로봇 바로 앞(0.4~0.5 m)을 찍는다. `Robot to approach ...` 가 나오고 1.0 m/s² 로 줄여 선다.
6. 그 상태에서 `task> go -2,-1,180` 처럼 반대쪽 goal 을 보내면 로봇이 그 자리에서 돌아서 간다. 장애물이 로봇 중심에서 0.3 m 안(RViz 의 빨간 원 안)이면 돌다가 닿을 수 있어서 돌지 않는다. 치우거나 조이스틱으로 빼낸다.

장애물은 다른 방법으로도 놓고 치울 수 있다.

| 방법 | 놓기 | 치우기 |
|---|---|---|
| 명령줄 | `ros2 run simple_line_bringup obstacle.py put 0,-1` (`--radius 0.2` 로 크기) | `ros2 run simple_line_bringup obstacle.py remove` (이 도구로 놓은 것 모두) |
| Gazebo 창 | 위쪽 툴바의 원기둥 / 상자를 누르고 바닥을 찍는다 | 물체를 고르고 Delete 키. 또는 이동 도구(T)로 끌어서 길 밖으로 |
| RViz (위와 같음) | `obstacle.py click` + Publish Point | 같은 물체를 다시 찍는다 |

- `obstacle.py` 는 gazebo_ros 의 `/spawn_entity`, `/delete_entity`, `/get_model_list` 서비스를 쓴다. 장애물 이름은 `obstacle_<놓은 시각>` 이다.
- 지운 모델과 같은 이름으로 다시 놓으면 gzserver 가 죽는 경우가 있었다 (Gazebo 11, 이 저장소의 확인 환경). 그래서 `obstacle.py` 는 이름을 다시 쓰지 않는다.
- 레이저 높이(약 0.17 m)에 걸리는 물체만 보인다. Gazebo 창에서 놓는 기본 상자(높이 1 m)와 원기둥은 보인다.
- `ros2 run` 은 뜨는 데 몇 초 걸린다 (이 환경에서 길게는 10초 넘게). 달리는 로봇 바로 앞에 놓으려면 click 모드를 미리 띄워 둔다.
- 미션 없이 RViz 의 **Nav2 Goal** 로 goal 을 줘도 똑같이 멈췄다 간다.

### 6-2. 설정 (`simple_line_bringup`)

| 파일 | 하는 일 |
|---|---|
| `launch/navigation.launch.py` | collision_monitor 와 velocity_smoother 를 띄우고 속도 토픽을 위 그림처럼 잇는다. controller_server 와 behavior_server 의 출력은 `/cmd_vel_nav`, 로봇은 표준 `/cmd_vel` 을 듣는다 |
| `launch/rviz.launch.py` | Nav2 기본 RViz 설정에 collision_monitor 의 원(빨강, `/polygon_footprint`) 표시를 더한다 |
| `params/simple_line_params.yaml` | `controller_server` 의 `FollowPath`(장애물 감속)와 `progress_checker`, `collision_monitor`, `velocity_smoother` |

```yaml
controller_server:
  ros__parameters:
    progress_checker:
      movement_time_allowance: 30.0        # 막혀서 서 있을 수 있는 시간 (Humble 기본 10) [s]
    FollowPath:
      plugin: "simple_line_controller::SimpleLineController"
      max_linear_decel: 0.5                # 감속도 [m/s^2]
      max_linear_jerk: 1.0                 # S 커브 [m/s^3]
      obstacle_check: true
      obstacle_check_width: 0.3            # 경로 양옆으로 이만큼 안의 장애물을 본다 [m]
      obstacle_stop_distance: 0.4          # 막힌 곳 이만큼 앞에 선다 [m]
      obstacle_resume_delay: 1.0           # 서서 기다리다가 길이 이만큼 비어 있으면 출발 [s]

collision_monitor:
  ros__parameters:
    cmd_vel_in_topic: "cmd_vel_nav"        # controller, behavior_server 가 내는 속도
    cmd_vel_out_topic: "cmd_vel_safe"      # velocity_smoother 로
    polygons: ["Footprint"]
    Footprint:
      type: "circle"
      radius: 0.3                          # 제자리에서 돌 때 쓸고 가는 원(0.24 m) + 여유
      action_type: "approach"              # 지금 명령대로 움직여 보고 닿을 때만 줄인다
      time_before_collision: 1.2           # [s]
      simulation_time_step: 0.05           # [s]
      max_points: 3                        # 원 안의 점이 이보다 많으면 닿은 것
    observation_sources: ["scan"]
    scan: {type: "scan", topic: "scan"}

velocity_smoother:
  ros__parameters:
    max_accel: [0.6, 0.0, 3.2]             # controller 보다 조금 크게 (평소에는 손대지 않는다)
    max_decel: [-1.0, 0.0, -3.2]           # 비상 정지 감속도
```

| 값 | 여기서 | 바꾸면 |
|---|---|---|
| `obstacle_check_width` | 0.3 m | 로봇 반폭 + 여유. 좁은 통로에서 옆 선반에 서 버리면 줄이고, 옆으로 튀어나온 물체를 놓치면 늘린다 |
| `obstacle_stop_distance` | 0.4 m | 서는 자리 (로봇 중심 → 장애물 약 width + 이 값). 원 반지름 + (속도 x 1.2초) 보다 멀어야 평소에 비상 감속이 걸리지 않는다 |
| `obstacle_resume_delay` | 1.0 s | 길이 빈 뒤 출발까지. 레이저 잡음에 꿈틀거리면 늘린다 |
| `Footprint.radius` | 0.3 m | 로봇이 제자리에서 돌 때 쓸고 가는 반지름 + 여유. waffle 은 회전 중심이 앞쪽이라 뒤 모서리가 0.24 m. 길쭉한 로봇이면 `type: "polygon"` 으로 실제 모양 ([AMR.md 5-2](../simple_line/AMR.md#5-2-비상-감속-움직이는-방향만-본다-collision_monitor-approach)) |
| `time_before_collision` | 1.2 s | 이만큼 앞까지 움직여 본다. 속도 x 이 값이 서는 데 필요한 거리보다 길게 |
| `max_points` | 3 | 레이저 잡음에 줄었다 가면 늘리고, 가는 물체를 놓치면 줄인다 |
| velocity_smoother `max_decel` | -1.0 m/s² | 비상 정지 감속도. 짐이 미끄러지지 않는 가장 큰 값 |
| `progress_checker.movement_time_allowance` | 30 s | 서 있는 동안 Nav2 가 기다려 주는 시간 (6-3) |
| `source_timeout` | 5 s | scan 이 이보다 오래되면 버린다. Humble 은 그때 막힌 곳이 없는 것으로 본다 (9장) |

- `--symlink-install` 로 빌드했다면 yaml 을 고친 뒤 launch 만 다시 하면 된다.
- 이 Humble 버전(1.1.18)의 collision_monitor 는 멈춤 상태를 토픽으로 내지 않는다 (로그만 남긴다). controller 의 감속도 로그뿐이다. 그래서 작업 feedback 에는 "장애물 때문에 기다리는 중" 이 따로 없다.

### 6-3. 오래 막혀 있으면 (progress checker)

장애물 앞에 서 있는 시간도 controller_server 의 progress checker 는 "진행이 없다" 로 본다. `movement_time_allowance`(30초) 동안 0.5 m 를 못 가면 `Failed to make progress` 로 FollowPath 가 실패하고, bt_navigator 의 navigation BT 가 재시도를 시작한다.

navigation BT 는 Nav2 기본 트리에서 제자리 회전, 후진 recovery 를 뺀 `simple_line_bringup/behavior_trees/navigate_to_pose_amr.xml` 이다. 실패하면 costmap 을 비우고 5초 기다린 뒤 다시 한다 ([AMR.md 6장](../simple_line/AMR.md#6-막혔을-때-회전-후진-없이-기다리기)). Gazebo 창고 월드에서 장애물을 치우지 않고 두었을 때 실제로 이렇게 됐다.

| 감속 시작 뒤 | 일어난 일 |
|---|---|
| 0 s | controller 가 감속해서 장애물 앞(로봇 중심 → 장애물 0.71 m)에 섬 |
| 31 s | `Failed to make progress`. FollowPath 를 다시 보낸다 |
| 61 s | 다시 실패. costmap 을 비우고 5초 대기, 경로부터 다시 |
| 그 뒤 | 약 30초마다 실패, 두 번에 한 번 5초 대기 |
| 457 s | 재시도(6번)를 다 써서 NavigateToPose 실패. 작업은 `go_only failed at waypoint 1/1` 로 ABORTED |

- 457초(7.6분) 동안 로봇은 한 번도 움직이지 않았다 (controller 명령 0, 속도 최대 0.0003 m/s). costmap 을 비운 직후에도 출발하지 않았다 (레이저가 0.2초 안에 다시 표시하고, 출발에는 1초 동안 길이 비어 있어야 한다).

- 중간에 치우면 바로 다시 간다. 재시도 사이 5초 대기 중이면 대기가 끝난 뒤 간다.
- 기다려 줄 시간은 `movement_time_allowance` 와 BT 의 `number_of_retries` 로 정한다.

### 6-4. 왜 goal 을 취소하지 않고 속도를 막나

**Nav2 에는 goal 을 잠시 멈추는(pause) 기능이 없다.** `NavigateToPose` 는 보내거나 취소하는 것만 된다. 그래서 "goal 은 그대로 두고 기다리기" 는 goal 아래에서 속도를 0 으로 하는 것으로 한다.

**AMR 들도 장애물에는 보통 이렇게 한다.**

- 주행 소프트웨어가 경로 위 장애물을 보고 미리 감속해서 그 앞에 선다. 비면 다시 간다. 주행 명령은 취소되지 않는다. 이 저장소의 controller 가 하는 일이다.
- 그 아래에 안전 레이저 스캐너가 있다. 보호 구역(protective field)에 무엇이 들어오면 모터를 세우고, 구역이 비면 다시 간다 (자동 재시작으로 설정했을 때). 구역은 움직임에 따라 바꾼다 (전진 속도 구간마다 다른 길이, 제자리 회전 때는 로봇을 둘러싼 작은 구역, 후진 때는 뒤쪽). 그 앞에 경고 구역(warning field)을 두어 감속시키기도 한다. collision_monitor 의 approach 는 "지금 움직이는 방향으로 부딪히는가" 를 보는 점에서 이 구역 전환과 같은 일을 소프트웨어로 한다.
- 정해 둔 시간보다 오래 막히면 경로를 다시 짜서 돌아가거나, 관제에 "길이 막힘" 을 알린다.
- 명령을 취소하고 다시 주는 방식은 관제(fleet) 쪽에서 쓴다. 교통 정리(다른 로봇에게 길 양보), 작업을 다른 로봇에 넘길 때, 사람이 일시정지를 누를 때다.

**이전 구현과 비교**: 이 저장소는 한동안 미션 BT 의 `StopAndGo` 노드가 레이저를 직접 보고, 막히면 Nav2 goal 을 취소했다가 길이 비면 같은 점으로 다시 보냈다.

| | controller 감속 + collision_monitor (지금) | 미션 BT 에서 취소 후 다시 보내기 (이전) |
|---|---|---|
| Nav2 goal | 그대로 (pending) | 취소 → 같은 점으로 새 goal |
| 멈추는 곳 | controller 가 내는 속도 (비상이면 로봇에 가는 속도) | Nav2 위 (미션 BT) |
| 적용 범위 | controller 로 가는 모든 이동. 비상 감속은 recovery 동작, `cmd_vel_nav` 로 보낸 teleop 까지 | 미션 BT 가 보낸 `NavigateToPose` 만 |
| 서는 자리 | 경로 위 장애물 0.4 m 앞 (경로를 따라 잰다). 비상: 로봇 둘레 원을 움직이는 방향으로 | goal 까지의 직선 위 |
| 감속 | S 커브 (jerk 1 m/s³) | Nav2 취소 시 0 → velocity_smoother 로 |
| 미션이 아는가 | 모른다 (작업은 RUNNING) | feedback `blocked` 로 알렸다 |
| 오래 막히면 | progress checker 가 30초마다 재시도, 약 7.6분 뒤 실패 (회전, 후진 없음) | `max_wait` 로 정했다 |
| 코드 | controller 에 약 100줄 + 설정 | BT 노드 2개, 약 350줄 |

**회피(돌아가기)를 나중에 더하려면**: 장애물을 보는 planner 와 controller 가 필요하다. Nav2 기본 costmap 은 이미 `/scan` 으로 장애물을 표시하므로, planner 를 NavFn 이나 Smac 으로, controller 를 Regulated Pure Pursuit 나 DWB 처럼 costmap 을 보는 것으로 바꾸면 된다. 그때도 collision_monitor 는 마지막 안전장치로 둔다.

---

## 7. 확인한 것

7-1, 7-2 는 같은 환경에서 확인했다. 7-3 은 Gazebo 에서 확인했다.

- **서버**: 실제 Nav2 Humble 서버(map_server, planner_server, controller_server, behavior_server, bt_navigator, velocity_smoother, waypoint_follower). Nav2 기본 BT, 빈 방 맵.
- **로봇**: Gazebo 대신 `cmd_vel` 을 적분하는 가짜 로봇. AMCL 은 쓰지 않았다.
- **출발점**: 모두 (-2.0, -1.0, 0°).
- **작업 4개**: 여러 곳에서 같은 작업 4개를 한 번에 넣었다. 정지 작업, 주행 중 작업, 회전, 복귀가 다 들어 있다.

  ```
  inspect 0,-1 1.5,-1,90
  scan 1.5,1.5 -1,1.5,180
  look_around -1,1.5,180
  go -2,-1,0
  ```

### 7-1. 액션으로 맡긴 작업 (`task_server`, `task_client.py`)

**단위 테스트** (`test/test_task_server.cpp`, gtest 7개 통과. 30번 반복해서 210/210): 실제 `task.xml` 과 `TaskServer` 에 액션 클라이언트로 goal 을 보낸다. Nav2 노드만 가짜다.

| 테스트 | 확인 내용 |
|---|---|
| `RunsATaskAndReportsFeedback` | 점 2개 inspect → SUCCEEDED("done", bt_status SUCCESS). feedback 이 (`go_inspect`, 도착 0) → (`go_inspect`, 1) → (`work`, 2) 로 바뀌고 bt_status 는 늘 RUNNING. 점 사이에도 `running` 이 비지 않음. 점이 차례로 `NavigateToPose` 의 goal 로 감 |
| `RunsTasksOneAfterAnother` | 같은 서버로 작업 두 개 (scan, look_around) 를 차례로 → 둘 다 SUCCEEDED (트리가 작업마다 초기화됨) |
| `NewGoalPreemptsTheRunningOne` | 작업 a 실행 중에 b 를 보냄 → a 는 ABORTED("preempted by b", bt_status RUNNING), a 의 이동 노드 halt, b 는 SUCCEEDED |
| `CancelHaltsTheTask` | scan 실행 중 cancel → CANCELED(bt_status RUNNING), 이동과 스캔 노드 모두 halt |
| `FailedNavigationAbortsWithTheNodeName` | 이동 실패 → ABORTED("go_look failed at waypoint 1/1", bt_status FAILURE), 회전은 하지 않음 |
| `FailureStopsAtThatWaypoint` | 점 3개 중 둘째 점에서 실패 → ABORTED("go_only failed at waypoint 2/3"), 셋째 점으로는 가지 않음 |
| `RejectsATaskWithoutWaypoints` | 점이 없는 goal → 거절 |

이 테스트는 30번 중 한 번꼴로 멈춰 있었다. 클라이언트가 서버를 찾은 직후에 보낸 goal 의 응답이 사라지는 경우인데 (서버가 아직 클라이언트의 응답 구독을 찾지 못한 때), 테스트가 응답을 시간 제한 없이 기다리고 있었다. 지금은 클라이언트를 만든 뒤 잠깐 기다리고, 응답이 없으면 멈추지 않고 실패로 끝낸다.

**Nav2 서버로 실행**: `task_server` 를 띄우고 `task_client.py` 와 `ros2 action` 명령으로 보냈다.

| 항목 | 결과 |
|---|---|
| 작업 4개를 한 번에 입력 (클라이언트가 차례로 보냄) | 모두 SUCCEEDED, 78.5 s (점마다 멈추기 전 약 61초) |
| 점마다 도착 | 6개 점 모두 도착 보고, 도착할 때 로봇과 점의 거리 3.5~4.9 cm |
| t2 feedback | `go_scan, scanning` (이동과 스캔이 동시에 RUNNING), 스캔 10개 |
| 선점 (`!go`) | t1 ABORTED("preempted by t2"), 이어서 t2 SUCCEEDED |
| `cancel` 명령 | t1 CANCELED, bt_navigator 의 goal 취소, 로봇 정지 |
| 클라이언트 Ctrl-C | t1 CANCELED, 로봇 정지 |
| `ros2 action send_goal --feedback` (4-5-5 의 두 명령) | 도착할 때마다 `waypoints_reached` 증가, SUCCEEDED. 점이 없는 goal 은 `Goal was rejected.` |
| 작업 중 `task_server` Ctrl-C | 클라이언트에 ABORTED("task_server shutting down"), Nav2 goal 취소, 로봇 정지 |

점 4개짜리 작업(`go 0,-1 0,1.5 -2,1.5 -2,-1,0`, 사각형)의 구간별 결과. 출발할 때 남은 방향 오차는 다음 점을 향해 다 돈 뒤 출발했는지를 보여 준다.

| 구간 | 출발할 때 남은 방향 오차 | 직선에서 벗어난 최대 거리 | 정지 위치와 점의 거리 |
|---|---|---|---|
| (-2,-1) → (0,-1) | 0.0° | 0.0 cm | 3.5 cm |
| (0,-1) → (0,1.5) | 1.7° | 1.0 cm | 3.4 cm |
| (0,1.5) → (-2,1.5) | 0.6° | 0.2 cm | 3.1 cm |
| (-2,1.5) → (-2,-1) | 0.3° | 0.2 cm | 3.0 cm |

(0,-1) 처럼 yaw 를 생략한 점에서는 도착할 때 다음 점 쪽으로 돌아 둔다. 그래서 다음 구간은 출발할 때 더 돌지 않는다.

### 7-2. BtActionServer 버전 (`task_server_lifecycle`)

**단위 테스트** (`test/test_task_server_lifecycle.cpp`, gtest 8개 통과. 10번 반복해서 80/80): 7-1 와 같은 경우를 lifecycle 노드로 확인한다. 서버 노드는 다른 스레드에서 spin 하고, Nav2 노드는 가짜 플러그인(`test_fake_nav2`)으로 바꿨다. 직접 만든 노드는 `bt_mission_plugin` 을 그대로 쓴다.

| 테스트 | 확인 내용 |
|---|---|
| `RunsATaskAndReportsFeedback` | 점 2개 inspect → SUCCEEDED("done"). feedback 이 (`go_inspect`, 0) → (`go_inspect`, 1) → (`work`, 2) |
| `RunsTasksOneAfterAnother` | 작업 두 개 (scan, look_around) 를 차례로 → 둘 다 SUCCEEDED |
| `NewGoalPreemptsTheRunningOne` | a 는 ABORTED (message 빈 값), a 의 이동 노드 halt, b 는 SUCCEEDED |
| `CancelHaltsTheTask` | feedback `go_scan, scanning` → cancel → CANCELED("canceled"), 이동 노드 halt |
| `FailedNavigationAbortsWithTheNodeName` | ABORTED("go_look failed at waypoint 1/1"), 회전은 하지 않음 |
| `FailureStopsAtThatWaypoint` | ABORTED("go_only failed at waypoint 2/3"), 셋째 점으로는 가지 않음 |
| `TaskWithoutWaypointsIsAcceptedThenAborted` | 점이 없는 goal → 수락된 뒤 ABORTED (`task_server` 는 거절) |
| `DeactivateStopsTheRunningTask` | 작업 중 deactivate → ABORTED("task server deactivated"), 이동 노드 halt. inactive 에서는 goal 거절 |

**Nav2 서버로 실행**: launch 로 lifecycle_manager 와 같이 띄우고 7-1 와 같은 순서로 보냈다. 아래 표는 점마다 멈추도록 바꾸기 전에 쟀고, 바꾼 뒤에는 점 2개 inspect (`inspect:1 0,-1 0,1,90`) 를 보내 arrived 1/2 → 2/2 → work → SUCCEEDED 를 확인했다.

| 항목 | 결과 |
|---|---|
| 작업 4개 | 모두 SUCCEEDED, 60.6 s (같은 날 `task_server` 61.1 s) |
| t2 feedback | `go_scan, scanning` |
| 선점 (`!go`) | t1 ABORTED (message 빈 값), 이어서 t2 SUCCEEDED |
| `cancel` 명령 | t1 CANCELED("canceled"), bt_navigator 의 goal 취소, 로봇 정지 |
| 클라이언트 Ctrl-C | t1 CANCELED, 로봇 정지 |
| `ros2 action send_goal --feedback` | feedback `go_inspect` → `work`, SUCCEEDED |
| 작업 중 서버 Ctrl-C | 클라이언트에 ABORTED("task server deactivated"), Nav2 goal 취소, 로봇 정지. 4초 뒤 lifecycle_manager 가 서버가 죽은 것을 알림 |

### 7-3. 장애물 (controller 감속, collision_monitor, Gazebo)

7-1, 7-2 와 달리 **Gazebo** 에서 확인했다. `simple_line.launch.py` 를 그대로 띄웠다 (Gazebo 11, TurtleBot3 waffle 과 그 레이저, AMCL, Nav2 Humble 1.1.18, 화면 없이). 장애물은 반지름 0.15 m, 높이 0.5 m 원기둥이다.

| 경우 | 결과 |
|---|---|
| Nav2 goal (1,-1), 빈 방. 출발 전에 (0,-1) 에 장애물, 선 뒤 8초가 지나면 치움 (6장 그림) | SUCCEEDED. 로봇 중심 → 장애물 0.80 m 에서 S 커브 감속 시작, 0.82초 뒤 0.70 m (앞면 0.63 m) 에 섬. 비상 정지 동작 없음. 기다리는 동안 움직임 없음. 치우자 1.6초 뒤 다시 가속 |
| 같은 goal, 달리는 로봇 0.4 m 앞에 갑자기 장애물, 선 뒤 5초가 지나면 치움 | SUCCEEDED. collision_monitor approach 로 0.95초 / 5.0 cm 만에 0.33 m 앞에 섬. 치우자 1.9초 뒤 S 커브로 출발 |
| 장애물 앞에 선 로봇에게 반대쪽 goal (-2,-1, 180°) | 0.70 m 앞에 섰을 때 0.5초, 0.33 m 앞에 섰을 때 0.3초 뒤 돌기 시작, 둘 다 SUCCEEDED. 0.20 m 앞(원 안, 돌면 닿음)이면 돌지 않음. 처음(StopZone)에는 0.33 m 앞에서 60초 동안 돌지 못했다 |
| 창고 월드, 장애물을 치우지 않음 | 457초 동안 움직이지 않고 30초마다 재시도, 그 뒤 ABORTED (6-3) |
| 작업 4개 (7장 처음의 목록), 빈 방, 장애물 없음 | 모두 SUCCEEDED, 74 s |
| 창고 월드 경로 9점, AMCL | SUCCEEDED. 선반, 벽 옆에서 controller 도 collision_monitor 도 잘못 서지 않음 |
| `obstacle.py click` 에 `/clicked_point` 를 세 번 보냄 (두 곳, 그리고 첫 장애물 근처) | 두 개를 놓고, 세 번째에 첫 장애물을 치웠다 |
| `obstacle.py remove` | 남은 `obstacle_*` 를 모두 치웠다. 남은 것이 없으면 `no obstacles to remove` |

- 처음 방식(collision_monitor 의 SlowZone 0.3배 + StopZone)에서는 0.97 m 앞에서 0.075 m/s 로 줄여 6초 기어간 뒤 섰고, 로봇에 간 명령의 jerk 가 12 m/s³ 였다 ([AMR.md 5장](../simple_line/AMR.md#5-장애물-앞에서-감속하고-서기)).
- 확인하면서 고친 것: 기다리는 동안 출발 전 회전 명령이 쌓였다가 풀릴 때 0.42 rad/s 로 튀던 것, 섰다가 5 cm 움찔 다시 가던 것, 감속 중에 0.4초 풀렸다가 다시 감속하던 것, 장애물을 등지고 돌아선 뒤에도 등 뒤의 그것을 "앞에 있다" 고 보던 것 (AMR.md 5-1).
- 창고 월드에서 지도 만들기, AMCL, 가감속을 확인한 결과는 [AMR.md 8장](../simple_line/AMR.md#8-확인한-것) 에 있다.
- RViz 는 화면 없이(Xvfb) 띄워서, 고친 설정이 열리고 collision_monitor 의 구역 표시(`Footprint`)가 목록에 들어가는 것까지 확인했다. 원(`/polygon_footprint`)은 주행 중 20 Hz 로 나왔다. Humble 의 collision_monitor 는 속도 명령이 들어올 때만 내므로, 로봇이 서 있으면 RViz 에 새로 그려지지 않는다.
- Gazebo 창(GUI)에서 물체를 놓고 지우는 방법은 확인하지 못했다.

---

## 8. 어떤 방법을 쓸까

| 상황 | 방법 |
|---|---|
| 경유점을 멈추지 않고 지나 최종 goal 에서만 서면 된다 | `NavigateThroughPoses` ([MULTI_GOAL.md](../simple_line/MULTI_GOAL.md) 의 `send_waypoints.py`) |
| 지점마다 멈춰서 같은 작업을 하고, 순서가 고정이다 | `waypoint_follower` ([MULTI_GOAL.md 5장](../simple_line/MULTI_GOAL.md#5-멈춰-가며-경유하기-waypoint_follower)). 작업은 task executor 플러그인으로 |
| 어디로 가서 무엇을 할지는 실행 중에 사람이나 서버가 정한다 | **`task_server` + ExecuteTask 액션 (4장)**. 점마다 멈춰서 도착을 알리고, 결과, 취소, 선점이 표준으로 있다 |
| 위와 같고, 서버를 Nav2 처럼 lifecycle_manager 로 관리한다 | `task_server_lifecycle` (5장). 비교와 고르는 기준은 5-4, 5-5 |
| 길에 장애물이 있으면 감속해서 멈췄다가, 치우면 같은 길로 간다 | controller 의 장애물 감속 + collision_monitor 비상 정지 (6장). goal 은 그대로, 속도만 0 |
| 장애물을 돌아서 간다 | 지금은 없다. costmap 을 보는 planner / controller 로 바꾼다 (6-4 끝) |

---

## 9. 주의할 점 (Humble)

- **도착 직후 회전이 조금 남을 수 있다.** 도착은 로봇이 멈춘 뒤 판정하지만(`StoppedGoalChecker`, 0.01 m/s, 0.05 rad/s 아래), goal 방향을 맞추다 회전 방향이 바뀌는 순간에도 멈춘 것으로 보일 수 있다. 그때 남은 회전 명령(0.2 rad/s 정도)은 velocity_smoother 가 0.1초 안에 0 으로 줄인다. 작업이 완전한 정지를 요구하면, 작업 노드 앞에 "속도가 0 인지" 확인하는 조건 노드를 두면 된다.
- **점 목록 포트 타입, tick 주기**: 3-3 을 본다.
- **점에 얼마나 정확히 서나**: goal checker 의 `xy_goal_tolerance` / `yaw_goal_tolerance` 가 정한다. `simple_line_params.yaml` 에서 0.05m / 0.05rad 로 줄였다 (Humble 기본 0.25 / 0.25). 실측으로 점에서 3~5cm 안에 섰다.
- **`ros2 action send_goal` 로 보낼 때 방향**: `orientation` 을 안 적으면 0° 다. 점마다 0° 로 돌았다가 다음 점으로 가므로, 방향을 적거나 `task_client.py` 를 쓴다 (4-5-5).
- **`task_server` 와 `task_server_lifecycle` 은 같이 띄우지 않는다.** 둘 다 `/execute_task` 액션을 연다.
- **벽도 장애물이다.** 경로나 goal 이 벽, 선반에서 0.3 m(`obstacle_check_width`) 안을 지나면 controller 가 그 0.4 m 앞에서 서서 기다린다. planner 는 0.22 m(로봇 반지름)까지만 거절하므로 그 사이의 점은 경로는 나오는데 도착하지 못한다. collision_monitor 의 원도 0.3 m 라서, 점은 벽, 물체에서 0.4 m 넘게 떨어뜨린다.
- **회전은 정말 닿을 때만 막는다.** collision_monitor 는 지금 명령대로 움직여 보고 판단하므로 장애물 앞에서도 돌아서 나갈 수 있다. 다만 로봇 중심에서 0.3 m(원) 안에 무엇이 있으면 돌다가 뒤 모서리가 닿을 수 있어서 회전도 0 이다. 이 controller 는 후진하지 않으므로, 그때는 사람이 치우거나 조이스틱으로 빼낸다.
- **레이저가 끊기면 멈추지 않는다 (Humble).** scan 이 `source_timeout`(5초)보다 오래되면 collision_monitor 는 그 데이터를 버리고 막힌 곳이 없는 것으로 본다. 레이저가 죽었는지는 따로 감시한다.
- **레이저 높이의 물체만 본다.** 2D 레이저 평면보다 낮거나 높은 물체(낮은 상자, 테이블 상판)는 보지 못한다. 필요하면 local costmap 과 collision_monitor 의 `observation_sources` 에 depth 카메라 점구름(`pointcloud`)을 더한다.
- **오래 막히면 Nav2 가 재시도한다.** 30초마다 재시도하고 약 7.6분 뒤 실패한다. 회전, 후진 recovery 는 뺐다 (6-3).

---

## 10. 파일

```
bt_navigator/
├── bt_mission_msgs/                         4장
│   ├── msg/Task.msg                         작업 하나 (id, waypoints, action, duration)
│   └── action/ExecuteTask.action            goal: Task, result: success/message/bt_status,
│                                            feedback: bt_status/running/도착한 점 수/elapsed
└── bt_mission/
    ├── behavior_trees/
    │   └── task.xml                         작업 하나를 실행, 점마다 NavigateToPose (4장)
    ├── include/bt_mission/
    │   ├── mission_nodes.hpp                직접 만든 BT 노드 선언 (포트 포함)
    │   ├── bt_common.hpp                    노드 등록, blackboard, bt_navigator 대기, 상태 로거
    │   ├── task_server.hpp                  ExecuteTask 액션 서버 + BT 실행 (4장)
    │   └── task_server_lifecycle.hpp        같은 일을 BtActionServer + lifecycle 노드로 (5장)
    ├── src/
    │   ├── mission_nodes.cpp                DoWork, ScanWhileMoving, ForEachWaypoint
    │   ├── bt_common.cpp                    서버들이 같이 쓰는 준비 코드 (노드 등록, 작업 → blackboard)
    │   ├── task_server.cpp                  goal 수락, 선점, 취소, feedback, result
    │   ├── task_server_main.cpp             task_server 실행 노드 (3-2)
    │   ├── task_server_lifecycle.cpp        BtActionServer 콜백 (5장)
    │   ├── task_server_lifecycle_main.cpp   task_server_lifecycle 실행 노드
    │   └── bt_plugin.cpp                    직접 만든 BT 노드를 플러그인 bt_mission_plugin 으로 (5장)
    ├── launch/task_server_lifecycle.launch.py   task_server_lifecycle + lifecycle_manager
    ├── scripts/task_client.py               액션 클라이언트: 한 줄 형식 해석, 대기열, 선점(!), cancel, 도착 출력
    ├── test/
    │   ├── test_task_server.cpp             gtest: ExecuteTask 액션 → task.xml (선점, 취소, feedback, BT 상태)
    │   ├── test_task_server_lifecycle.cpp   gtest: 같은 경우를 task_server_lifecycle 로
    │   └── test_fakes.hpp / .cpp            테스트가 같이 쓰는 가짜 Nav2 노드 (플러그인 test_fake_nav2)
    ├── tools/draw_bt.py                     XML → 그림 (graphviz)
    └── doc/
        ├── task.png                         BT 그림 (draw_bt.py 로 생성)
        ├── stop_and_go.png                  Gazebo 에서 장애물을 놓고 치운 기록 (6장)
        └── layers.dot / layers.png          계층 구조 그림
```

장애물 처리(controller 의 장애물 감속은 `simple_line/simple_line_controller`, collision_monitor 설정과 장애물 도구 `obstacle.py` 는 `simple_line/simple_line_bringup`)는 6-2 에 있다.
