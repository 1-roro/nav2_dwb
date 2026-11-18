# 설비 앞 15 cm 정밀 도킹: ArUco marker 하나, 4바퀴 메카넘

설비(기계)의 도킹 면에 붙인 **ArUco marker (60 mm 하나, 또는 둘)** 를 **RealSense D455** 의 컬러 카메라로 보고, 메카넘 로봇의 앞면을 설비에서 **15 cm** 떨어진 자리에 mm 단위로 세운다. 그 자리에서 설비 작업을 한 뒤 undock 한다. 작업은 `task_client.py` 로 보낸다: `dock:dock_a <staging 자리>`, `undock`, `undock_precise:dock_a`. `dock_a` 는 marker 하나(0번), `dock_b` 는 marker 둘(2번, 3번)을 붙인 설비다.

이 문서 하나에 원리(ArUco marker, 자세 추정), 도킹 메커니즘, 구현 구조, 실제 로봇으로 옮기는 순서, 실험 결과, 시험 시나리오를 모두 정리했다. 처음 공부한다면 **[DOCKING_STUDY.md](DOCKING_STUDY.md)** 부터 읽는다: 좌표계, 카메라, 공분산, 최적화, 제어 같은 기초 개념부터 시작해, 영상 한 장이 속도 명령이 되기까지의 계산과 판단을 실제 코드로 계산한 숫자 예제로 한 단계씩 푼 공부 노트다. 메카넘 바퀴가 어떻게 옆으로 가는지(구조, 힘, 바퀴 속도의 식, 실제의 한계)는 **[MECANUM_STUDY.md](MECANUM_STUDY.md)**. 코드들이 왜 있고 어떻게 이어지는지, 좌표계·odometry·PnP·IPPE·칼만 필터를 그림으로 먼저 보려면 **[DOCKING_BIG_PICTURE.md](DOCKING_BIG_PICTURE.md)**. 미션 BT 의 다른 부분(작업 액션, 장애물 대기)은 [README.md](README.md).

![mecanum robot docked](../simple_line/doc/mecanum_robot.png)

Gazebo 결과 (참값. 카메라는 D455 의 화각, 848 x 480):

| 조건 | marker | 앞뒤 [mm] | 옆 [mm] | 방향 [°] |
|---|---|---|---|---|
| 실제와의 차이 없음, 도면 값 목표 | 하나 (3회) | -0.7 ~ +0.9 | -0.3 ~ -0.4 | 0.01 ~ 0.02 |
| | 둘 (3회) | +0.5 ~ +0.9 | -0.4 | 0.00 |
| 차이 넣음 (메카넘 미끄러짐 + 카메라 TF yaw 1.5°, 옆 1 cm + marker 크기 2 %), 도면 값 목표 | 하나 (2회) | -2.3 ~ -2.9 | **-5.3 ~ -6.8** | **+1.5 ~ +1.6** |
| | 둘 (2회) | -0.2 ~ +0.6 | **-6.9 ~ -7.1** | **+1.5** |
| 같은 차이, **가르친 자세** 목표 | 하나 (2회) | +0.5 ~ +1.4 | -0.5 ~ +0.1 | 0.02 ~ 0.05 |
| | 둘 (3회) | -0.2 ~ +1.4 | -0.3 ~ -0.6 | 0.00 ~ 0.04 |

![results](docs/aruco_results.png)

어디부터 읽나:

| 알고 싶은 것 | 장 |
|---|---|
| 큰 그림과 "왜" 를 그림으로 (코드 지도, 좌표계, odom, PnP·IPPE, 칼만 필터) | [DOCKING_BIG_PICTURE.md](DOCKING_BIG_PICTURE.md) |
| 기초부터 차근차근 (공부 노트) | [DOCKING_STUDY.md](DOCKING_STUDY.md) |
| 메카넘 바퀴는 어떻게 옆으로 가나 (공부 노트) | [MECANUM_STUDY.md](MECANUM_STUDY.md) |
| 전체가 어떻게 생겼나, 무엇이 무엇과 이어지나 | 1 |
| ArUco marker 가 무엇이고 어떻게 자세가 나오나 | 2 |
| marker 를 본 것이 어떻게 "15 cm 앞에 서기" 가 되나 | 3 |
| 코드가 어떻게 나뉘어 있나 | 4, 17 |
| 시뮬레이션으로 돌려 보기 | 5 |
| **실제 로봇으로 옮기는 순서** | 6 (설정 파일은 7) |
| 카메라(D455)의 화각, marker 를 얼마나 크게 어디에 붙이나, **marker 를 둘로 늘리면** | 8 |
| 알고리즘의 자세한 설명과 근거 | 9 ~ 12 |
| 얼마나 정확한가, 실제 로봇에서 어떻게 확인하나 | 13, 14 |
| 여러 기능을 잇고 일부러 문제를 넣어 시험하기 (시나리오) | 16 |
| 안 될 때 | 15 |

목차

- [1. 프로젝트 한눈에](#1-프로젝트-한눈에)
  - [1-1. 무엇을 하나](#1-1-무엇을-하나)
  - [1-2. 구성과 데이터 흐름](#1-2-구성과-데이터-흐름)
  - [1-3. 좌표계와 자리](#1-3-좌표계와-자리)
  - [1-4. 핵심 다섯 가지](#1-4-핵심-다섯-가지)
- [2. ArUco marker 의 원리](#2-aruco-marker-의-원리)
  - [2-1. marker 의 구조](#2-1-marker-의-구조)
  - [2-2. 영상에서 찾는 과정](#2-2-영상에서-찾는-과정)
  - [2-3. 모서리 넷에서 자세로](#2-3-모서리-넷에서-자세로)
  - [2-4. marker 하나의 한계와 이 프로젝트의 대응](#2-4-marker-하나의-한계와-이-프로젝트의-대응)
- [3. 도킹 메커니즘: 본 것에서 움직임까지](#3-도킹-메커니즘-본-것에서-움직임까지)
  - [3-1. 한 번의 도킹](#3-1-한-번의-도킹)
  - [3-2. 측정에서 명령까지](#3-2-측정에서-명령까지)
  - [3-3. undock](#3-3-undock)
- [4. 구현: 코드 구조](#4-구현-코드-구조)
  - [4-1. 실행 단위](#4-1-실행-단위)
  - [4-2. 로봇이 줘야 하는 것, 이 프로젝트가 내는 것](#4-2-로봇이-줘야-하는-것-이-프로젝트가-내는-것)
  - [4-3. 도킹 BT 노드가 만들어진 방식](#4-3-도킹-bt-노드가-만들어진-방식)
  - [4-4. 계산과 ROS 를 나눴다](#4-4-계산과-ros-를-나눴다)
  - [4-5. 빌드와 테스트](#4-5-빌드와-테스트)
- [5. 해 보기 (시뮬레이션)](#5-해-보기-시뮬레이션)
- [6. 실제 로봇으로 옮기기](#6-실제-로봇으로-옮기기)
  - [6-1. 로봇이 갖춰야 하는 것](#6-1-로봇이-갖춰야-하는-것)
  - [6-2. 옮기는 순서](#6-2-옮기는-순서)
  - [6-3. 안전 영역 (collision_monitor)](#6-3-안전-영역-collision_monitor)
  - [6-4. 옮긴 뒤 무엇을 어디서 바꾸나](#6-4-옮긴-뒤-무엇을-어디서-바꾸나)
- [7. 설정 파일 하나: docking.yaml](#7-설정-파일-하나-dockingyaml)
- [8. 카메라(D455)와 marker: 크기, 배치, 15 cm](#8-카메라d455와-marker-크기-배치-15-cm)
  - [8-1. 카메라: RealSense D455 의 컬러](#8-1-카메라-realsense-d455-의-컬러)
  - [8-2. marker 하나 (dock_a)](#8-2-marker-하나-dock_a)
  - [8-3. marker 둘 (dock_b): 크기, 배치, 달라지는 것](#8-3-marker-둘-dock_b-크기-배치-달라지는-것)
  - [8-4. 인쇄와 부착](#8-4-인쇄와-부착)
  - [8-5. 15 cm 때문에 바뀐 것](#8-5-15-cm-때문에-바뀐-것)
- [9. 자세 추정 (aruco_detector.py)](#9-자세-추정-aruco_detectorpy)
  - [9-1. 왜 그냥 PnP 가 아닌가](#9-1-왜-그냥-pnp-가-아닌가)
  - [9-2. 바닥 사전분포를 둔 6자유도 MAP](#9-2-바닥-사전분포를-둔-6자유도-map)
  - [9-3. 깊이로 방향 돕기 (선택, `use_depth:=True`)](#9-3-깊이로-방향-돕기-선택-use_depthtrue)
- [10. 도킹 순서와 제어](#10-도킹-순서와-제어)
  - [10-1. dock 의 자리: Kalman 필터 (`DockFilter`)](#10-1-dock-의-자리-kalman-필터-dockfilter)
  - [10-2. 제어: dock 좌표계의 오차와 깔때기 (`mecanumStep`)](#10-2-제어-dock-좌표계의-오차와-깔때기-mecanumstep)
  - [10-3. 서서 재고 고치기 (`ArucoRefine`)](#10-3-서서-재고-고치기-arucorefine)
  - [10-4. 안 보일 때, 놓쳤을 때: 뒤로 가지 않고 그 자리에서](#10-4-안-보일-때-놓쳤을-때-뒤로-가지-않고-그-자리에서)
- [11. undock 두 가지](#11-undock-두-가지)
- [12. 가르친 자세: 고정 오차 지우기](#12-가르친-자세-고정-오차-지우기)
- [13. 시뮬레이션 실험](#13-시뮬레이션-실험)
- [14. 실제 로봇에서의 정밀도](#14-실제-로봇에서의-정밀도)
  - [14-1. 오차 예산](#14-1-오차-예산)
  - [14-2. 카메라와 TF 점검](#14-2-카메라와-tf-점검)
  - [14-3. 배율과 데드밴드 재기 (`dock_calibrate.py`)](#14-3-배율과-데드밴드-재기-dock_calibratepy)
  - [14-4. 확인 순서](#14-4-확인-순서)
  - [14-5. 더 높이려면](#14-5-더-높이려면)
- [15. find_marker 가 실패할 때](#15-find_marker-가-실패할-때)
- [16. 시나리오로 시험하기](#16-시나리오로-시험하기)
  - [16-1. 준비](#16-1-준비)
  - [16-2. S1: 점검 순회 → 도킹 → 작업 → 정밀 undock → 복귀](#16-2-s1-점검-순회--도킹--작업--정밀-undock--복귀)
  - [16-3. S2: 설비 두 대를 차례로](#16-3-s2-설비-두-대를-차례로)
  - [16-4. S3: staging 에서 marker 가 화면 밖](#16-4-s3-staging-에서-marker-가-화면-밖)
  - [16-5. S4: 들어가는 도중 marker 가 가려짐](#16-5-s4-들어가는-도중-marker-가-가려짐)
  - [16-6. S5: 도킹 중 취소와 선점](#16-6-s5-도킹-중-취소와-선점)
  - [16-7. S6: 붙어 있는 동안 뒤가 막힘](#16-7-s6-붙어-있는-동안-뒤가-막힘)
  - [16-8. S7: staging 으로 가는 길의 장애물](#16-8-s7-staging-으로-가는-길의-장애물)
  - [16-9. S8: 실제 로봇과의 차이, 가르치기, 되풀이](#16-9-s8-실제-로봇과의-차이-가르치기-되풀이)
  - [16-10. S9: 틀린 입력](#16-10-s9-틀린-입력)
  - [16-11. 실제 로봇에서](#16-11-실제-로봇에서)
  - [16-12. 시나리오가 찾아낸 것](#16-12-시나리오가-찾아낸-것)
- [17. 파일](#17-파일)
- [참고 자료](#참고-자료)

---

## 1. 프로젝트 한눈에

### 1-1. 무엇을 하나

1. Nav2 로 설비 앞 **staging 자리**(설비 면에서 0.6 ~ 0.9 m, marker 가 보이는 곳)까지 간다. 여기까지는 지도와 AMCL 의 정밀도(수 cm, 수 °)다.
2. 거기서부터는 지도를 쓰지 않는다. **카메라로 설비에 붙은 marker 를 보고**, 설비를 기준으로 한 자리에 mm 단위로 선다 (로봇 앞면이 설비 면에서 15 cm).
3. 작업이 끝나면 물러난다 (undock). 곧게 뒤로 가거나, marker 를 보며 정해진 자리로 나온다.

지도 기반 위치 추정으로는 mm 를 맞출 수 없다. 그래서 마지막 구간은 "도킹 대상에 붙은 표식을 직접 보고 그것에 대한 상대 자세를 0 으로 만드는" 시각 서보로 한다. 표식으로 ArUco marker 를 쓴다.

### 1-2. 구성과 데이터 흐름

```
 task_client.py ──execute_task 액션 (dock:dock_a 3.85,1.0,0)──▶ task_server ─── 미션 BT (task.xml) 를 tick
                                                                   │
        ① staging 까지                                             │ NavigateToPose 액션
        Nav2 (지도, AMCL, planner, controller) ◀───────────────────┤
                                                                   │
        ② marker 보기                                              │
 RealSense ──color/image_raw, camera_info──▶ aruco_detector.py ──/dock/aruco──▶ 도킹 BT 노드
           └─(선택) aligned_depth ─────────▶   영상 → "로봇에서 본 dock 의 자세" + 공분산       DetectArucoDock
                                              ▲ TF base_footprint ← camera                    ArucoServo (approach, insert, retreat)
                                                                                              ArucoRefine
        ③ 로봇 자리 (짧은 시간에는 정확하다)                                                    BackOff
 바퀴 odom ──TF odom ← base_footprint────────────────────────────────────────▶    │
 레이저 ──/scan (제자리 회전 전 주변, 물러날 때 뒤)────────────────────────────▶    │
                                                                                  ▼ cmd_vel_nav (vx, vy, wz)
        ④ 움직이기              collision_monitor ──▶ velocity_smoother ──cmd_vel──▶ 메카넘 구동
                                (마지막 구간에서는 MuteCollisionMonitor 가 잠깐 끈다: 설비는 붙어야 하는 물체다)

 config/docking.yaml ──▶ aruco_detector.py (marker 번호, 크기, 높이)
                    └──▶ 도킹 BT 노드 (로봇 치수, 토픽, 속도, 허용오차, 단계, 안전)
```

- **detector 는 보기만 하고, BT 노드는 움직이기만 한다.** 둘 사이는 토픽 하나(`/dock/aruco`)다. 그래서 카메라나 검출 방법을 바꿔도 도킹 쪽은 그대로다.
- 시뮬레이션과 실제 로봇이 다른 것은 맨 왼쪽(카메라, odom, 레이저, 구동)과 `docking.yaml` 뿐이다.

### 1-3. 좌표계와 자리

![docking overview](docs/docking_overview.png)

위에서 본 그림이다. staging 이 옆으로 0.25 m, 방향이 15° 틀어진 경우(`dock:dock_a 3.85,1.25,-15`)이고, 길은 실제 제어 식(`aruco_dock_control.hpp` 의 `mecanumStep`)으로 계산했다. 설 곳은 D 에서 x = `gap` + 로봇 앞면까지 = 0.15 + 0.16 = **0.31 m**, y = 0, 설비를 보는 방향(180°)이다.

| 좌표계 | 무엇 | 누가 정하나 |
|---|---|---|
| D (dock) | 원점 = 바닥에서 도킹 면의 가운데, x = 로봇 쪽, z = 위 | `docking.yaml` 의 `docks` (marker 가 D 의 어디에 있는지) |
| camera optical | 카메라 렌즈. x 오른쪽, y 아래, z 앞 | 카메라 드라이버의 frame, TF |
| base_footprint | 로봇. 바닥, 회전 중심 | 로봇의 URDF / TF |
| odom | 바퀴 odom 의 기준. 몇 초 동안은 mm 단위로 믿을 수 있고, 오래되면 흐른다 | 로봇 드라이버 |
| map | 지도. staging 까지 갈 때만 쓴다 | AMCL |

도킹은 **odom 과 D** 만 쓴다. dock 의 자리를 odom 에 잡아 두고(필터), 로봇은 odom 으로 움직이고, 영상이 올 때마다 dock 의 자리를 고친다 (3-2).

### 1-4. 핵심 다섯 가지

| 문제 | 한 것 | 장 |
|---|---|---|
| marker 하나로는 방향(yaw)이 약하고, 자세의 해가 둘이다 | "설비는 바닥에 서 있다" 를 사전분포로 둔 자세 추정(MAP) + 필터가 후보를 고른다 | 2-4, 9 |
| 영상은 늦게 오고(50 ~ 150 ms), 멀리서 본 것은 부정확하다 | 영상을 찍은 시각의 odom 으로 옮기고, 공분산으로 무게를 준 Kalman 필터 | 10-1 |
| 메카넘은 미끄러지고 느린 명령에 반응하지 않는다 | 끝까지 영상으로 닫힌 제어, 깔때기, 축마다 최소 속도와 배율, 서서 재고 고치기 | 10-2, 10-3 |
| 15 cm 는 가깝다: marker 가 잘리거나 부딪힐 수 있다 | marker 크기 60 mm, 방금 본 marker 만 믿고 전진, 최소 거리에서 정지 | 8 |
| 카메라 TF, marker 치수는 아무리 재도 조금 틀린다 (고정 오차) | 로봇을 한 번 정확히 놓고 그때 본 것을 목표로 삼는다 (가르친 자세) | 12 |

## 2. ArUco marker 의 원리

![aruco principle](docs/aruco_principle.png)

① marker 의 구조 (2-1), ② 보이는 모습과 자세 (2-3), ③ ~ ⑦ 영상에서 찾는 과정 (2-2). 영상은 `test_aruco_detector.py` 가 쓰는 합성 영상이고, ④ ~ ⑥ 은 OpenCV 가 안에서 하는 일을 같은 함수(`adaptiveThreshold`, `findContours`, `approxPolyDP`, `warpPerspective`)로 따라 한 것이다.

### 2-1. marker 의 구조

ArUco marker 는 **검은 테두리 안에 흑백 칸(비트)이 격자로 들어 있는 정사각형**이다. 이 프로젝트의 marker 는 `DICT_4X4_50` 의 0번이다: 안쪽 4 x 4 = 16 칸, 테두리까지 6 x 6 칸. 한 변 60 mm 이므로 한 칸이 10 mm 다.

```
  ████████████      검정 = 0, 흰색 = 1
  ██  ██    ██      안쪽 4 x 4 (16 비트):   1 0 1 1
  ████  ██  ██                             0 1 0 1
  ██████    ██                             0 0 1 1
  ██████  ████                             0 0 1 0    → 0번
  ████████████      바깥 한 줄은 테두리 (항상 검정). 그 둘레에 흰 여백 20 mm
```

- **dictionary**: 쓸 수 있는 비트 무늬의 목록. `DICT_4X4_50` 은 16 비트 무늬 50개(0 ~ 49번)다. 2^16 가지 중에서 **서로 많이 다른 것만** 골랐다: 어느 두 marker 도, 어느 방향으로 돌려도 4 비트 이상 다르다 (OpenCV 4.5.4 로 세어 보았다). 그래서 한두 칸을 잘못 읽어도 다른 번호가 되지 않는다.
- **방향을 스스로 안다**: 무늬가 90° 돌린 자기 자신과도 4 비트 이상 다르다 (0번은 10 ~ 12 비트). 그래서 영상에서 어느 쪽이 "위" 인지 알 수 있고, 네 모서리에 순서(왼위 → 오른위 → 오른아래 → 왼아래)를 붙일 수 있다. 이것이 자세를 구하는 데 꼭 필요하다.
- **틀리게 읽으면 "다른 번호" 가 아니라 "못 찾음" 이 된다**: OpenCV 의 기본 설정에서 이 dictionary 는 한 비트만 달라도 고쳐 읽지 않고 버린다 (`maxCorrectionBits` 1 x `errorCorrectionRate` 0.6 = 0 비트. 4.5.4 에서 칸 하나를 뒤집어 확인했다). 도킹에는 이쪽이 안전하다: 엉뚱한 설비로 가지 않고, 다음 영상을 기다린다.
- **흰 여백이 꼭 있어야 한다**: 테두리가 검정이므로 그 바깥이 밝아야 사각형으로 잡힌다. 어두운 면에 여백 없이 붙이면 찾지 못한다 (확인했다). 그래서 인쇄한 종이의 흰 여백을 자르지 않는다.
- **칸이 적을수록 멀리서 읽힌다**: 같은 크기면 4x4 가 6x6 보다 칸이 크다. marker 를 몇 개만 쓰므로 가장 작은 dictionary 를 골랐다. 여러 설비에 붙일 때는 1번, 2번 ... 을 쓴다 (`docking.yaml` 의 `docks`).
- QR 코드와 다른 점: QR 은 글자를 담으려고 칸이 수백 개라 가까이서 선명해야 읽힌다. ArUco 는 번호 하나만 담고 **네 모서리를 정확히 찾는 것**이 목적이라 멀리서, 흐려도, 비스듬해도 잡힌다.

### 2-2. 영상에서 찾는 과정

OpenCV `cv2.aruco.detectMarkers` 가 하는 일이다. 번호는 위 그림과 같고, 괄호 안은 기본값이다.

```
③ 영상           흑백으로 바꾼다
④ 이진화         adaptive threshold: 주변(창 3, 13, 23 px 세 가지)의 평균보다 어두운 화소를 검정으로.
                 한 값으로 자르지 않으므로 조명이 고르지 않아도, 그림자가 져도 경계가 남는다
⑤ 사각형 후보     윤곽선 → 다각형 근사 → 꼭짓점이 4개인 볼록 다각형만. 둘레가 영상 긴 변의 3 % ~ 400 % 인 것
⑥ 펴서 읽기       네 꼭짓점으로 원근을 없애 정사각형으로 편다 (한 칸에 4 px). 칸마다 흑백을 판정한다 (Otsu, 칸 가장자리 13 % 는 무시)
                 테두리가 검정이 아니면 버린다. 16 비트를 dictionary 와 네 방향으로 비교 → 번호와 방향. 맞는 것이 없으면 버린다
⑦ 모서리 다듬기   네 모서리를 화소 아래 단위로 (CORNER_REFINE_SUBPIX: 모서리 둘레의 밝기 경사로 0.1 ~ 0.3 px)
결과: marker 마다 번호 + 네 모서리의 영상 좌표 (u, v). 순서는 marker 기준 왼위 → 오른위 → 오른아래 → 왼아래
```

- 그림의 ⑤ 에서 후보는 7개다: marker, 그 옆의 검은 사각형, 흰 판, marker 안의 흰 칸들. ⑥ 을 지나면 marker 하나만 남는다.
- ⑥ 덕분에 **잘못 찾는 일이 거의 없다**: 우연히 생긴 사각형(명판, 창문, 상자)은 dictionary 에 없는 무늬라 버려진다.
- 정밀도를 정하는 것은 ⑦ 이다. 이 프로젝트의 모든 mm 는 "네 모서리를 0.1 ~ 0.3 px 로 안다" 에서 나온다 (붙은 자리에서 1 px = 0.25 mm).
- 이 프로젝트에서 바꾼 설정: 모서리 다듬기를 켰다 (기본은 끔). `minDistanceToBorder` 를 0 으로 했다 (기본 3 이면 가까이서 크게 보이는 marker 를 놓쳤다). 대신 모서리가 화면 끝 6 px 안에 걸린 marker, 한 변 16 px 보다 작은 marker 는 직접 버린다 (`aruco_detector.py` 의 `usable`).

### 2-3. 모서리 넷에서 자세로

marker 의 **실제 크기 s 를 알면**, marker 좌표계에서 네 모서리의 자리를 안다: (∓s/2, ±s/2, 0). 카메라는 3차원 점 X 를 영상의 (u, v) 로 보낸다 (핀홀 모델):

```
        ⎡u⎤       ⎡fx  0  cx⎤
    λ · ⎢v⎥  =    ⎢ 0 fy  cy⎥ · ( R · X + t )         K: 카메라 내부값 (camera_info 의 K, 왜곡 D)
        ⎣1⎦       ⎣ 0  0   1⎦                          R, t: 카메라에서 본 marker 의 방향과 위치 (모르는 것 6개)
```

모서리 4개가 식 8개를 주므로 모르는 것 6개(위치 3, 방향 3)를 풀 수 있다. 이것이 **PnP** (Perspective-n-Point) 다. 직관으로는:

| 영상에서 보이는 것 | 알게 되는 것 |
|---|---|
| marker 가 **얼마나 크게** 보이나 | 거리 (크기 s 를 알기 때문. 60 mm 가 240 px 이면 0.16 m) |
| marker 가 **화면 어디에** 있나 | 방향각(옆, 위아래 위치) |
| 사각형이 **얼마나 돌아가** 보이나 | 시선 둘레의 회전 (roll) |
| 정사각형이 **사다리꼴로 찌그러진** 정도 (원근) | marker 면이 기운 각 (yaw, pitch) |

- 크기 s 가 1 % 틀리면 거리가 1 % 틀린다. 그래서 인쇄한 marker 를 재서 `size` 에 넣는다.
- 카메라 내부값 K 는 RealSense 가 공장 보정값을 `camera_info` 로 준다. 코드는 화각을 가정하지 않고 이 값을 쓴다.

### 2-4. marker 하나의 한계와 이 프로젝트의 대응

위 표의 마지막 줄이 약점이다. **위치는 화소가 직접 말해 주지만, 면의 기울기는 "찌그러진 정도" 로만 보인다.**

1. **해가 둘**: 정사각형 하나의 PnP 는 수학적으로 해가 두 개다 (IPPE). 면이 왼쪽으로 기운 것과 오른쪽으로 기운 것이 영상에서 거의 같게 보인다. 정면에 가깝거나 멀수록 구분이 안 된다.
2. **방향이 흔들린다**: 60 mm marker 를 0.6 m 에서 보면 64 px 이다 (D455, 1280 x 720). 1° 기울 때 좌우 변의 길이 차이는 0.1 px 남짓이라 모서리 잡음과 같은 크기다. 그래서 한 장으로 잰 방향이 0.6 m 에서 1° 안팎, 1 m 밖에서는 몇 ° 씩 틀린다 (9-2 의 표).
3. **지렛대**: 방향이 1° 틀리면 0.3 m 떨어진 로봇 자리는 옆으로 5 mm 틀린다.

대응:

| 약점 | 대응 | 장 |
|---|---|---|
| 해가 둘, 기울기와 방향이 섞인다 | 설비는 바닥에 서 있으므로 "기울기 0, 높이 0 근처" 라는 사전분포를 넣어 푼다. 두 해가 남으면 필터가 지금까지의 추정에 가까운 쪽을 고른다 | 9-2, 10-1 |
| 멀리서 본 방향이 부정확하다 | marker 를 둘로 늘리면 방향이 두 marker 의 위치 차이로 정해진다 (오차가 약 1/7) | 8-3 |
| | 측정마다 공분산을 계산해 필터가 가까이서 본 것을 크게 반영한다. 다가갈수록 정확해진다. 마지막 판단은 0.16 m 에서 서서 여러 장으로 한다 (옆 0.1 mm, 방향 0.02°) | 10-1, 10-3 |
| 그래도 접근 초반의 방향이 필요하다 | (선택) 깊이 영상에서 설비 면의 평면을 맞춰 방향을 따로 잰다 | 9-3 |
| 지렛대 | marker 를 도킹 면에 바로 붙였다 (지렛대 = 로봇까지의 0.31 m 뿐) | 8 |

## 3. 도킹 메커니즘: 본 것에서 움직임까지

### 3-1. 한 번의 도킹

| | 단계 (BT 노드, stage) | 하는 일 | 끝나는 조건 | Gazebo 에서 |
|---|---|---|---|---|
| 0 | `NavigateToPose` | 지도로 staging 까지 | Nav2 goal checker (5 cm, 3°) | 거리에 따라 |
| 1 | `DetectArucoDock` (find_marker) | 서서 marker 를 8장 보고 dock 의 자리(odom)와 설 곳을 잡는다. 안 보이면 제자리에서 돌며 찾는다 | 8장 | 1.1 ~ 1.3 s |
| 2 | `ArucoServo` (approach) | 설 곳 0.25 m 앞까지. 옆과 방향을 먼저 맞춘다. collision_monitor 켠 채 | 10 mm, 5 mm, 1° 안 | 3.7 ~ 6.7 s |
| 3 | `ArucoServo` (insert) | collision_monitor 를 끄고 마지막 0.25 m 를 0.05 m/s 로 | 3 mm, 2 mm, 0.3° 안 | 7.6 s |
| 4 | `ArucoRefine` (refine) | 서서 1 s 재고, 남은 오차를 고치고, 다시 잰다 | 1.5 mm, 1.5 mm, 0.2° 안 | 3.4 s (고치기 1번) |
| | | | staging 도착 뒤 합계 | 약 16 ~ 19 s |

2 ~ 4 가 실패하면(marker 를 2 초 놓침 등) 뒤로 물러나지 않고 그 자리에 서서 다시 보고(`look_again`) 이어서 한다. 3번까지 (10-4).

### 3-2. 측정에서 명령까지

```
① 검출 (영상마다)        모서리 u₁..u₄ ──PnP + 바닥 사전분포──▶ q = base 에서 본 dock (x, y, yaw), 공분산 Σ
② odom 으로 옮기기      z = T_odom←base(영상을 찍은 시각) ∘ q            (움직이며 찍어도, 늦게 와도 맞다)
③ 필터                 dock 의 odom 자세 d ← Kalman(d, z, Σ)            (설비는 움직이지 않는다)
④ 오차 (20 Hz)         로봇의 D 좌표 r = d⁻¹ ∘ T_odom←base(지금)
                       e = r − 설 곳  →  along (앞뒤), lat (옆), yaw
⑤ 명령                 (vx, vy, wz) = mecanumStep(e)  ──cmd_vel_nav──▶ 로봇
```

- **dock 을 odom 에 고정해 두고, 로봇은 odom 으로 움직인다.** 영상은 15 ~ 30 Hz 이고 (Gazebo 15, RealSense 30) 50 ~ 150 ms 늦게 오지만 odom 은 빠르고 짧은 시간에는 정확하다. 그래서 제어(④, ⑤)는 odom 으로 20 Hz 로 매끄럽게 하고, 영상(①~③)은 "dock 이 odom 의 어디에 있나" 만 계속 고친다. odom 이 흐르면 다음 영상이 그만큼 dock 의 자리를 옮겨 준다.
- 이것은 위치 기반 시각 서보(PBVS)다: 영상에서 3차원 자세를 먼저 구하고, 그 자세의 오차를 0 으로 만든다. 설 곳을 "가르친 자세" 로 주면 (12장) 수렴점에서는 영상 자체를 맞추는 것과 같아져서 카메라 보정 오차에 둔감해진다.
- ⑤는 메카넘이라 세 축을 동시에 낸다. 다만 "깔때기": 옆과 방향이 남은 거리에 비해 틀려 있으면 앞으로 가지 않고 그것부터 맞춘다. 그리고 marker 를 방금 보지 못했으면 설비 쪽으로는 가지 않는다 (10-2, 8장).

### 3-3. undock

| | 방법 |
|---|---|
| `undock` | odom 으로 곧게 뒤로 0.4 m. 레이저로 뒤를 본다 |
| `undock_precise` | 같은 ①~⑤ 로, 목표만 "붙은 자리 + 0.40 m" 로 바꿔서 물러난다. 옆과 방향이 설비 기준으로 맞은 채 나온다 (11장) |

## 4. 구현: 코드 구조

### 4-1. 실행 단위

| 실행 단위 | 하는 일 | 실제 로봇에서 |
|---|---|---|
| Gazebo + 로봇 모형 (`simple_line.launch.py` 의 "로봇" 묶음) | 카메라, 레이저, odom 을 내고 cmd_vel 로 움직인다 | **로봇 드라이버 + RealSense 드라이버로 바꾼다** |
| Nav2 (`simple_line_bringup/launch/navigation.launch.py`) | 지도, AMCL, planner, controller, collision_monitor, velocity_smoother | 그대로, 또는 로봇이 이미 쓰는 Nav2 |
| `aruco_detector.py` (`aruco_detector.launch.py`) | 영상 → `/dock/aruco` | 그대로 (`camera`, `config_file` 만 준다) |
| `task_server` | 작업 액션 서버. `task.xml` 의 BT 를 tick 한다 (100 Hz) | 그대로 (`docking_config` 만 준다) |
| `task_client.py` | 작업을 한 줄로 보낸다 | 그대로, 또는 상위 시스템이 `execute_task` 액션을 직접 부른다 |

### 4-2. 로봇이 줘야 하는 것, 이 프로젝트가 내는 것

| 방향 | 이름 (기본값, `docking.yaml` 의 `ros` 에서 바꾼다) | 형식 | 쓰는 곳 |
|---|---|---|---|
| 로봇 → | `<camera>/color/image_raw`, `camera_info` | Image, CameraInfo | detector |
| 로봇 → | TF `base_footprint` ← 카메라 optical frame | TF (고정) | detector |
| 로봇 → | TF `odom` ← `base_footprint` | TF | 도킹 BT 노드 |
| 로봇 → | `scan` | LaserScan | 제자리 회전 전 주변 확인, 물러날 때 뒤 확인 |
| 로봇 → | `imu`, `odom` 토픽 | Imu, Odometry | `dock_calibrate.py` 만 |
| → 로봇 | `cmd_vel_nav` (collision_monitor, velocity_smoother 를 거쳐 `cmd_vel`) | Twist (linear.x, **linear.y**, angular.z) | 도킹 BT 노드 |
| 내부 | `dock/aruco` | bt_mission_msgs/ArucoDetection | detector → BT 노드 |
| 내부 | `dock/aruco_debug`, `dock/aruco_estimate`, `dock/final_error` | Image, PoseStamped, Vector3Stamped | 확인, 기록 |
| 상위 → | `execute_task` | bt_mission_msgs/action/ExecuteTask | 작업 받기 |
| → Nav2 | `navigate_to_pose` | nav2_msgs/action/NavigateToPose | staging 까지 |
| → Nav2 | `collision_monitor` 의 파라미터 `Footprint.enabled` | 파라미터 서비스 | 마지막 구간에서 끄고 켜기 |

### 4-3. 도킹 BT 노드가 만들어진 방식

- 모두 BehaviorTree.CPP 의 `StatefulActionNode` 다: `onStart` (준비, 설정 읽기) → `onRunning` (tick 마다, 끝날 때까지 RUNNING) → `onHalted` (작업 취소, 선점: **속도 0 을 내고** 구독을 닫는다).
- `task_server` 는 BT 노드의 ROS 콜백을 돌려 주지 않는다. 그래서 노드마다 자기 callback group 과 executor 를 두고 tick 할 때 `spin_some` 한다 (`DockRos`). Nav2 의 BT 노드와 같은 방식이다.
- 노드끼리는 blackboard 의 `{aruco_dock}` (`ArucoDockState`: dock 이름, 필터, 설 곳) 로 이어진다. `DetectArucoDock` 가 만들고 `ArucoServo`, `ArucoRefine` 이 이어 쓴다.
- 값은 XML 포트가 아니라 `docking.yaml` 에서 읽는다 (`loadDockingConfig`, 시작할 때마다). XML 에는 `stage="insert"` 같은 이름만 있다 (7장).
- 제어 주기는 노드 안에서 20 Hz 로 맞춘다 (tick 은 100 Hz).

### 4-4. 계산과 ROS 를 나눴다

| 파일 | 내용 | 시험 |
|---|---|---|
| `aruco_dock_control.hpp` | 2D 자세 연산, Kalman 필터, 후보 고르기, 여러 장 합치기, 깔때기, `mecanumStep` | `test_aruco_dock_control.cpp` (미끄러짐과 늦음을 넣은 폐루프 포함) |
| `dock_control.hpp` | 단순 undock 의 제어, 레이저 점으로 뒤/주변 확인 | `test_dock_control.cpp` |
| `docking_config.hpp` | `docking.yaml` 읽기, 단계 값 찾기, 오타 찾기 | `test_docking_config.cpp` |
| `aruco_detector.py` 의 함수들 (`estimate`, `refine_map`, `depth_plane_yaw` ...) | 검출, 자세 추정, 공분산 | `test_aruco_detector.py`: marker 를 알려진 자세로 그린 합성 영상 + Gazebo 영상 |
| `aruco_dock_nodes.cpp`, `dock_nodes.cpp` | 위 계산을 ROS (토픽, TF, BT) 에 잇는 얇은 층 | Gazebo (13장). 작업 서버와 트리의 흐름, 안전 영역 되켜기는 `test_task_server.cpp` (가짜 Nav2) |

계산이 ROS 와 떨어져 있어서 로봇 없이 `colcon test` 로 확인할 수 있고, 실제 로봇에 맞춰 고칠 때 건드릴 곳이 분명하다: 값은 `docking.yaml`, 순서는 `task.xml`, 계산은 위 표의 헤더.

### 4-5. 빌드와 테스트

```bash
cd ~/ros2_ws && colcon build --packages-select simple_line_planner simple_line_controller simple_line_bringup \
    bt_mission_msgs bt_mission
source install/setup.bash
colcon test --packages-select bt_mission && colcon test-result --verbose     # 60 tests, 0 failures
```

필요한 것: ROS 2 Humble, Nav2, `python3-opencv` (4.5.4 이상, aruco 포함), `python3-numpy`, `python3-yaml`, `cv_bridge`, yaml-cpp. 시뮬레이션에는 `gazebo_ros_pkgs`, `turtlebot3_gazebo` 도.

## 5. 해 보기 (시뮬레이션)

```bash
# 터미널 1: 창고 + 4바퀴 메카넘 로봇(컬러 카메라) + 동쪽 벽 앞의 설비 모형 (도킹 면 x = 4.55, y = 1.0)
#          + marker 검출 노드 (aruco_detector.launch.py 를 같이 띄운다. 따로 띄우려면 aruco_detector:=False)
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True
# 터미널 2
ros2 run bt_mission task_server
# 터미널 3
ros2 run bt_mission task_client.py
task> dock:dock_a 3.85,-2.5 3.85,1.0,0    # 마지막 점 = 설비 앞 staging 자리 (planner 가 직선이라 한 점을 거친다)
task> undock                              # 곧게 뒤로 0.4 m
task> dock:dock_a 3.85,1.25,-15           # 옆 0.25 m, 비스듬히 15 도에서
task> undock_precise:dock_a               # marker 를 보며 dock 축을 따라 물러난다
```

marker 가 둘인 설비 (8-3) 로 바꿔 보려면 `dock_markers:=2` 로 띄우고 작업의 dock 이름을 `dock_b` 로 쓴다:

```bash
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True dock_markers:=2
task> dock:dock_b 3.85,-2.5 3.85,1.0,0
task> undock_precise:dock_b
```

시뮬레이션의 카메라는 D455 컬러의 화각(가로 90°)이고 해상도는 `camera_resolution:=848x480` (기본) 이다. 실제 로봇에서 권하는 `1280x720` 으로도 띄울 수 있지만, GPU 없이 그리는 환경에서는 Gazebo 가 무거워져 Nav2 가 제어 주기를 놓쳤다 (staging 까지 가다가 `Failed to make progress`).

실제 로봇과의 차이를 넣어 보려면 (13장):

```bash
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True \
    mecanum_slip:=True camera_tf_error:=1.5,0.0,0.01 aruco_detector:=False
ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True marker_scale:=1.02
```

RViz: `/dock/aruco_debug` (Image: 찾은 marker, 바닥 위 자세로 다시 그린 모서리, 예상 오차), `/dock/aruco_estimate` (PoseStamped: 필터가 잡은 dock 자리), `/dock/final_error` (붙은 뒤 서서 잰 오차).

![camera](docs/aruco_camera_debug.jpg)

붙은 자리(카메라 ↔ marker 0.16 m)에서 본 `/dock/aruco_debug` (D455 화각, 1280 x 720). marker 는 240 px 이고 위아래로 240 px, 좌우로 520 px 남는다. 초록 = 검출한 모서리, 빨간 점 = 추정한 바닥 위 자세로 다시 그린 모서리. 둘이 겹치고 `tilt 0.0deg z +0mm` 이면 카메라 TF 와 설정이 맞다. 첫 줄의 `(1/1)` 은 그 dock 의 marker 가운데 몇 개를 보았는지다.

여러 작업을 잇고 장애물, 가림, 취소 같은 문제를 일부러 넣어 보는 시나리오 아홉 개는 16장에 있다 (`task_client.py --file`, 설비 두 대 `dock_markers:=both`, 참값 판정 `dock_truth.py`).

## 6. 실제 로봇으로 옮기기

코드는 그대로 두고 **`config/docking.yaml` 한 파일**을 로봇에 맞춘다 (7장). 그 밖에 손댈 수 있는 곳은 `task.xml` 의 안전 영역 이름 하나다 (6-3). 시뮬레이션과 실제 로봇이 다르게 쓰는 것은 로봇/카메라 드라이버와 이 설정뿐이고, 도킹 코드와 BT, detector 는 같다.

### 6-1. 로봇이 갖춰야 하는 것

옮기기 전에 로봇 PC 에서 확인한다. 이름은 예이고, 다르면 `docking.yaml` 의 `ros` 와 launch 인자로 맞춘다.

| 필요한 것 | 확인 | 기준 |
|---|---|---|
| 메카넘 구동: `Twist` 의 `linear.x`, **`linear.y`**, `angular.z` 를 따른다 | 조이스틱으로 옆으로 천천히 | 옆으로 곧게 간다. 0.01 ~ 0.03 m/s 의 느린 명령에도 움직이는지 본다 (안 움직이는 속도가 데드밴드, 14-3) |
| 바퀴 odom, TF `odom → base_footprint` | `ros2 run tf2_ros tf2_echo odom base_footprint` | 20 Hz 이상. 서 있을 때 흐르지 않는다 |
| 컬러 영상, `camera_info` | `ros2 topic hz /camera/camera/color/image_raw` | 15 Hz 이상. `camera_info` 의 `k` 가 채워져 있다 |
| TF `base_footprint → 카메라 optical frame` | `ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame` | 줄자로 잰 카메라 자리와 맞다 (optical frame 은 z 가 앞) |
| Nav2 (지도, AMCL, `navigate_to_pose`) | staging 자리로 `go` 작업 | 수 cm, 수 ° 안에 선다 |
| 2D 레이저 (없어도 된다) | `ros2 topic hz /scan` | 없으면 `ros.scan_topic: ""`. 제자리에서 돌며 찾기와 뒤 확인을 하지 않는다 |
| IMU (없어도 된다) | `ros2 topic hz /imu` | `dock_calibrate.py` 가 회전의 기준으로 쓴다 |

### 6-2. 옮기는 순서

바퀴를 굴리지 않고 볼 수 있는 것부터 확인한다. 앞 단계가 통과해야 다음으로 간다.

| | 단계 | 하는 일 | 통과 기준 |
|---|---|---|---|
| 1 | 빌드 | 로봇 PC 에 저장소를 받아 빌드하고 `colcon test` (4-5) | bt_mission 의 테스트가 모두 통과 (자세 추정이 그 PC 의 OpenCV, numpy 로 돈다는 뜻) |
| 2 | 설정 파일 | `docking.yaml` 을 복사해 [로봇], [설비] 값을 채운다 | task_server 로그에 `[dock] config <그 경로>` 가 나오고 `unknown name` 경고가 없다 |
| 3 | marker | 인쇄하고, 재고, 붙인다 (8장) | 같이 찍힌 50 mm 눈금이 50 mm. 잰 한 변을 `size` 에. marker 중심 높이 = 카메라 렌즈 높이 |
| 4 | 카메라 | RealSense 드라이버와 카메라 TF | 6-1 의 영상, TF 항목 |
| 5 | 검출만 | detector 만 띄우고 로봇을 손으로 설비 앞 여러 자리에 둔다 (14-2) | `/dock/aruco` 가 나온다. `/dock/aruco_debug` 에서 rms < 0.5 px. 0.5 m 안에서 `tilt` < 1°, `z` < 5 mm. `camera_distance` 가 줄자와 1 % 안 |
| 6 | 안전 영역 | collision_monitor 의 영역 이름을 확인한다 (6-3) | `ros2 param get /collision_monitor Footprint.enabled` 가 값을 준다 |
| 7 | 첫 도킹 | **`gap: 0.30` 으로 멀찍이**, 비상정지를 잡고 `dock:dock_a <staging>` | 설비 30 cm 앞에 곧게 선다. 로그의 단계가 3-1 의 순서대로 지나간다 |
| 8 | 보정 | `dock_calibrate.py` 로 배율과 데드밴드를 잰다 (14-3) | 값을 `motion` 에 옮겨 적고 다시 도킹했을 때 끝에서 떨거나 못 미치지 않는다 |
| 9 | 15 cm | `gap: 0.15` 로 되돌리고 도킹 | 붙은 자리에서 `border_px` 가 240 안팎 (1280 x 720 에서. marker 가 화면 가운데). refine 이 한두 번 고치고 끝난다 |
| 10 | 가르치기 | 로봇을 정확한 자리에 놓고 `aruco_teach.py` (12장) | 다음 도킹의 로그에 `target (taught ...)` |
| 11 | 확인 | `dock` → `undock` 을 20번, **영상이 아닌 기준**으로 잰다 (14-4) | 필요한 정밀도 안에서 되풀이된다 |
| 12 | undock | `undock`, `undock_precise:dock_a` (11장) | 뒤를 막아 놓으면 서는지도 본다 |

**2. 설정 파일**

```bash
mkdir -p ~/robot_config
cp $(ros2 pkg prefix bt_mission)/share/bt_mission/config/docking.yaml ~/robot_config/
ros2 run bt_mission task_server --ros-args -p docking_config:=$HOME/robot_config/docking.yaml
ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera config_file:=$HOME/robot_config/docking.yaml
```

| 값 | 어떻게 정하나 |
|---|---|
| `robot.front`, `rear`, `half_width` | `base_footprint` (회전 중심) 에서 범퍼 끝까지 줄자로. `front` 는 15 cm 를 정하는 값이다: 설 곳 = `gap` + `front` |
| `robot.turn_radius` | 중심에서 가장 먼 모서리까지 |
| `ros.*` | 로봇의 frame 과 토픽 이름. `cmd_vel_topic` 은 안전 장치(collision_monitor)를 거치는 입력 |
| `docks.dock_a.size`, `height`, `offset`, `lateral` | 인쇄한 marker 와 붙인 자리를 잰 값 (8장) |
| `motion.max_*` | 로봇이 낼 수 있는 속도보다 작게. 처음에는 그대로 둔다 |
| `motion.min_*`, `scale_*`, `odom_*` | 8번 단계에서 재서 넣는다 |

BT 노드는 작업을 시작할 때마다 파일을 다시 읽으므로 값을 고친 뒤 task_server 를 다시 띄울 필요가 없다. detector 는 시작할 때 한 번 읽으므로 `docks` 를 고치면 detector 를 다시 띄운다.

**4. 카메라** (RealSense D455. 해상도와 fps 는 8-1. 옵션 이름은 realsense-ros 판마다 조금 다르니 `ros2 launch realsense2_camera rs_launch.py --show-args` 로 확인한다)

```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x15 enable_sync:=true \
    align_depth.enable:=true          # 깊이를 쓸 때만 (use_depth:=True)
# 카메라가 URDF 에 없으면: 줄자로 잰 값으로 시작한다 (x 앞, y 왼쪽, z 위 [m], 각도 [rad])
ros2 run tf2_ros static_transform_publisher --x 0.15 --y 0 --z 0.13 --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_footprint --child-frame-id camera_link
```

토픽은 `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info` 이므로 detector 에 `camera:=/camera/camera` 를 준다. `camera_link` 에서 `camera_color_optical_frame` 까지는 드라이버가 낸다. TF 는 줄자 값으로 시작해서 5번 단계에서 detector 의 진단(`tilt`, `z`)을 보며 맞춘다. 남는 오차는 가르친 자세가 지운다 (12장).

**7. 첫 도킹**: staging 은 로봇 중심이 설비 면 0.6 ~ 0.9 m 앞에서 설비를 보는 자리다. 로봇을 조이스틱으로 그 자리에 놓고 `ros2 run tf2_ros tf2_echo map base_footprint` 로 지도 좌표를 읽어 `dock:dock_a x,y,방향(도)` 로 쓴다. 처음에는 `gap` 을 넉넉히 두고, 로봇이 marker 를 보며 옆과 방향을 먼저 맞춘 뒤 곧게 들어가는지 본다.

### 6-3. 안전 영역 (collision_monitor)

설비 15 cm 앞까지 가려면 "앞에 무엇이 있으면 선다" 는 평소의 안전 영역을 마지막 구간에서 꺼야 한다. `task.xml` 의 `MuteCollisionMonitor` 가 그 일을 한다: 자식(insert 와 refine, undock)이 도는 동안 Nav2 collision_monitor 의 영역 하나를 끄고(`<영역>.enabled` 파라미터), 끝나면 성공이든 실패든 취소든 다시 켠다. 이 이름이 `docking.yaml` 이 아니라 XML 에 있는 단 하나의 로봇 값이다.

```xml
<MuteCollisionMonitor name="mute_safety" polygon="Footprint">     <!-- monitor="collision_monitor" 가 기본값 -->
```

| 로봇의 구성 | 할 일 |
|---|---|
| Nav2 collision_monitor 를 쓰고 영역 이름이 `Footprint` (이 저장소의 설정과 같다) | 그대로 |
| 영역 이름이나 노드 이름이 다르다 | `task.xml` 의 세 곳 (Dock, Undock, UndockPrecise) 에서 `polygon="..."`, `monitor="..."` 를 고친다. 이름은 `ros2 param list /collision_monitor \| grep enabled` |
| collision_monitor 를 쓰지 않는다 | `task.xml` 에서 `MuteCollisionMonitor` 를 빼고 그 자식만 남긴다 (그대로 두면 2 초 뒤 `could not switch off` 로 도킹이 실패한다). `ros.cmd_vel_topic` 을 로봇이 받는 토픽으로 바꾼다 |
| 로봇 자체의 안전 장치(안전 레이저의 보호 영역, PLC)가 가까운 물체 앞에서 세운다 | 그 장치의 muting 은 로봇 쪽 기능이다. 도킹 구간에서 영역을 바꾸는 신호를 내는 BT 노드를 `MuteCollisionMonitor` 자리에 둔다 |

- 여기서 확인한 Nav2 는 1.1.20 이다. 판에 따라 `<영역>.enabled` 가 없을 수 있으니 6번 단계의 명령으로 먼저 확인한다.
- 영역이 꺼진 동안 로봇을 지키는 것은 `docking.yaml` 의 `safety` 다: 방금 본 marker 가 없으면 설비 쪽으로 가지 않고, 카메라가 `min_camera_distance` 보다 가까워지면 멈춘다 (8장). 속도는 0.05 m/s 다.

### 6-4. 옮긴 뒤 무엇을 어디서 바꾸나

| 바꾸고 싶은 것 | 고치는 곳 |
|---|---|
| 로봇 치수, 토픽, 속도, 허용오차, 붙는 거리 | `docking.yaml` |
| 설비를 더한다 | `docking.yaml` 의 `docks` 에 이름과 아직 안 쓴 marker 번호로 더하고, marker 를 인쇄해 붙이고, 가르친다. 작업은 `dock:<이름> ...` |
| 한 설비의 marker 를 둘로 늘린다 | `docks` 의 그 dock 을 `markers:` 목록으로 (8-3). 한 장에 다시 인쇄하고 다시 가르친다 |
| 순서: 재시도 횟수, 단계 추가, 도킹 뒤 신호 확인 | `behavior_trees/task.xml` |
| 안전 영역의 이름 | `task.xml` 의 `MuteCollisionMonitor` (6-3) |
| 카메라가 다르다 (화각, 해상도) | launch 의 `camera`. marker 크기를 다시 정한다 (8장) |
| 검출 방법을 바꾼다 (다른 marker, 레이저 반사판) | `dock/aruco` 를 같은 형식 (`ArucoDetection`) 으로 내는 노드를 만든다. 도킹 쪽은 그대로 |
| 제어 식, 필터 | `aruco_dock_control.hpp` 와 그 테스트 (ROS 없이 돈다) |

## 7. 설정 파일 하나: docking.yaml

로봇, 카메라, 설비마다 달라지는 값은 모두 `bt_mission/config/docking.yaml` 에 있다. `task.xml` 에는 "무엇을 어떤 순서로" 만 있고, 노드는 XML 의 `stage="approach"` 같은 이름으로 자기 값을 찾는다.

| 구역 | 내용 | 누가 읽나 |
|---|---|---|
| `dictionary`, `docks` | [설비] dock 마다 marker (하나: `marker_id`, 여럿: `markers:` 목록), 크기, 높이, 붙는 거리(`gap`: 로봇 앞면 ↔ 도킹 면) | detector, BT, 도구 |
| `robot` | [로봇] 중심에서 앞/뒤/옆면까지, 제자리 회전 반지름 | BT |
| `ros` | [로봇] odom/base frame, cmd_vel, 레이저, 검출 토픽 | BT, 도구 |
| `motion` | 속도 한계, 가속도, 이득, 허용오차, 깔때기. **[측정]** 데드밴드(`min_*`), 배율(`scale_*`), odom 흐름 | BT |
| `stages` | 단계마다 `motion` 을 덮어쓰는 값과 단계의 동작 (approach, insert, refine, retreat) | BT |
| `detect` | marker 찾기: 보는 시간, 돌아 볼 각도, 가르친 자세 폴더 | BT |
| `safety` | 가까이 붙을 때: 방금 본 marker 만 믿기, 화면 끝 여유, 최소 거리 | BT |
| `undock` | 단순 undock 의 거리와 속도 | BT |

- 값을 찾는 순서: `stages.<stage>.<이름>` → `motion.<이름>` → 코드의 기본값 (파일의 값과 같다).
- 파일 안의 주석이 항목마다 뜻과 단위를 적어 두었다. 코드 쪽은 `include/bt_mission/docking_config.hpp` 하나가 파일을 읽는다 (ROS 와 무관, `test_docking_config.cpp`).
- `task.xml` 을 고칠 일은 순서를 바꿀 때뿐이다 (예: 재시도 횟수, 단계 추가).

## 8. 카메라(D455)와 marker: 크기, 배치, 15 cm

### 8-1. 카메라: RealSense D455 의 컬러

| | D455 컬러 | 이 프로젝트에 주는 뜻 |
|---|---|---|
| 화각 | 가로 약 90°, 세로 약 65° (1280 x 800). 16:9 (1280 x 720, 848 x 480) 로 받으면 세로 약 59° | 붙은 자리(카메라 ↔ 면 0.16 m)에서 가로 0.32 m, 세로 0.18 m 가 보인다. marker 가 잘릴 걱정이 적고, marker 둘을 나란히 볼 수 있다 |
| 화소 밀도 | 1280 x 720 에서 fx ≈ 640 px, 848 x 480 에서 fx ≈ 424 px | 화각이 넓은 만큼 같은 해상도에서 marker 가 작게 찍힌다. **1280 x 720 으로 받는다** |
| 셔터 | global shutter | 움직이며 찍어도 marker 가 찌그러지지 않는다 |
| 초점 | 고정 초점 | 0.16 m 에서는 조금 흐릴 수 있다 (8-5) |
| 깊이의 최소 거리 | 약 0.4 ~ 0.5 m (해상도에 따라) | 깊이는 붙은 자리에서 쓸 수 없다. 접근 초반의 방향만 돕는다 (9-3) |

- **해상도는 1280 x 720, 15 fps 를 권한다** (`rgb_camera.color_profile:=1280x720x15`). 848 x 480 으로 받으면 한 장의 방향 오차가 1.2 ~ 4 배다 (9-2 의 표). detector 의 계산은 이 환경에서 1280 x 720 한 장에 약 25 ms 였다 (받기 6, 변환 5, 검출 6, 자세 5 ~ 8 ms. Python). 30 fps 면 CPU 코어 하나의 70 % 쯤이고 15 fps 면 그 절반이다. 도킹은 0.05 ~ 0.15 m/s 로 움직이므로 15 Hz 면 충분하다 (시뮬레이션도 15 Hz 다).
- **코드는 화각을 가정하지 않는다.** `camera_info` 의 K 와 왜곡 계수를 그대로 쓴다. 위 숫자는 marker 의 크기와 배치를 정하는 데만 썼다. 컬러 화각은 자료마다 90° x 65° 또는 86° x 57° (±3°) 로 적혀 있으므로, 실제 값은 `ros2 topic echo /camera/camera/color/camera_info --once` 의 `k[0]` (fx) 로 확인한다: 가로 화각 = 2 · atan(영상 폭 / 2fx). 아래의 여유는 86° 여도 남는다.
- D455 는 몸체가 넓고(약 124 mm) 컬러 렌즈가 몸체의 가운데에 있지 않다. TF 는 realsense-ros 가 `camera_link → camera_color_optical_frame` 으로 내 주지만, **marker 는 "붙은 자리에서 컬러 렌즈의 정면" 에 오게 붙인다** (`docks.*.lateral`, `height`).

### 8-2. marker 하나 (dock_a)

![marker](docs/aruco_marker_dock_a.png)

**DICT_4X4_50 의 0번, 검은 테두리 바깥 한 변 60 mm, 흰 여백 20 mm (종이 100 x 100 mm).** 카메라가 D455 로 정해진 뒤에도 그대로 쓴다.

| | 값 (D455, 1280 x 720) | 근거 |
|---|---|---|
| 붙은 자리의 카메라 ↔ marker | 0.16 m | 로봇 앞면 ↔ 설비 0.15 m, 카메라는 앞면 1 cm 안쪽 |
| 그 거리에서 보이는 범위 | 가로 0.32 m, 세로 0.18 m | 가로 90°, 세로 59° |
| 60 mm marker 의 크기 | 240 px | 세로 720 px 의 1/3 |
| 남는 여유 | 위아래 240 px (60 mm), 좌우 520 px (130 mm) | 높이가 ±4 cm, 옆이 ±10 cm 틀려도 화면 안 |
| 붙은 자리에서 한 장의 정밀도 (합성 영상) | 옆 0.4 mm, 방향 0.08° | 9-2. 서서 여러 장을 합치면 더 작다 |
| 찾을 수 있는 거리 | 한 변 16 px 이상 = 약 2.4 m. **방향까지 믿을 만한 것은 0.8 m 안** | staging 은 설비 면에서 로봇 중심 0.6 ~ 0.9 m |

- **더 크게 해도 된다.** 세로 여유가 60 mm 라 80 mm 까지는 붙은 자리에서 잘리지 않는다 (여유 50 mm). 80 mm 면 1 m 에서 본 한 장의 방향이 3° 에서 0.8° 로 좋아진다 (9-2). staging 을 1 m 밖에 둘 수밖에 없거나 848 x 480 으로 받아야 하면 80 mm 로 인쇄하고 `size: 0.08` 로 적는다. 그보다 방향이 더 필요하면 크기를 키우지 말고 marker 를 둘로 늘린다 (8-3).
- 다른 거리에 붙는다면: **한 변 ≤ 0.5 x (붙은 자리의 카메라 ↔ marker 거리)**. 세로로 보이는 범위의 절반에서 여유를 뺀 값이다.

### 8-3. marker 둘 (dock_b): 크기, 배치, 달라지는 것

marker 하나의 약점은 방향이다 (2-4). 정사각형 하나가 찌그러진 정도로만 방향을 알 수 있어서다. marker 둘을 가로로 벌려 붙이면 방향이 **두 marker 의 위치 차이**로 정해진다.

![two markers](docs/aruco_marker_dock_b.png)

**권장: 60 mm marker 둘을 같은 높이에 가로로 나란히, 중심 간격 160 mm, A4 가로 한 장에 인쇄.** `docking.yaml` 의 `dock_b` 가 이 배치다.

```yaml
docks:
  dock_b:
    size: 0.06
    height: 0.13                      # marker 중심 = 카메라 렌즈 높이
    gap: 0.15
    markers:
      - {id: 2, lateral: -0.08}       # 로봇이 볼 때 왼쪽
      - {id: 3, lateral: 0.08}        # 오른쪽. 가운데(lateral 0) = 붙은 자리에서 카메라 렌즈의 정면
```

| 정한 것 | 값 | 근거 |
|---|---|---|
| 놓는 방향 | 가로로 나란히, 같은 높이 | 필요한 것은 바닥 위의 방향(yaw)이다. 세로로 벌리면 pitch 만 좋아진다 |
| 중심 간격 | 160 mm | 붙은 자리에서 가로로 0.32 m 가 보인다. marker 의 바깥 끝이 가운데에서 110 mm 이므로 좌우로 50 mm (200 px) 씩 남는다. 화각이 86° 여도 39 mm 남는다. 간격이 넓을수록 방향이 좋아지지만 붙은 자리에서 잘린다 |
| 크기 | 60 mm 그대로 | 방향은 간격이 정하므로 키울 필요가 없다. 50 mm 를 180 mm 간격으로 놓은 것은 가까이서는 비슷하고 멀리서는 더 나빴다 (1.5 m 에서 3.9° 와 2.5°) |
| 인쇄 | 한 장에 (종이 260 x 100 mm, A4 가로) | 두 marker 의 간격이 인쇄로 정해진다. 따로 붙이면 0.5 mm 만 어긋나도 방향 0.2° 의 고정 오차가 된다 |
| 번호 | 한 dock 안에서도, dock 끼리도 겹치지 않게 | `dock_b` 는 2번과 3번. 1번은 `dock_a` 에 하나 더 붙일 때를 위해 비워 두었다 |

![two markers at the docked pose](docs/aruco_camera_debug_two.jpg)

붙은 자리에서 본 `/dock/aruco_debug` (Gazebo, D455 화각, 1280 x 720). 둘 다 화면 안에 있고 좌우로 200 px 씩 남는다. 한 장으로 잰 방향의 σ 가 marker 하나일 때 0.64° 에서 0.07° 로 줄었다 (5장의 그림과 비교).

**얼마나 좋아지나**

| | marker 하나 (60 mm) | marker 둘 (60 mm, 간격 160 mm) |
|---|---|---|
| 한 장의 방향 오차, 0.75 m (staging), 합성 영상 | 1.11° | 0.16° |
| 같은 것, 1.0 m | 3.1°. 30장 가운데 6장에서 뒤집힌 후보가 같이 나옴 | 0.38°. 뒤집힌 후보 없음 |
| 같은 것, 붙은 자리 | 0.08° | 0.01° |
| Gazebo: 처음 찾았을 때 dock 방향의 σ | 0.8 ~ 3.5° | 0.2 ~ 0.7° |
| Gazebo: 붙은 자리의 참값 오차 | 1 mm, 0.02° 안 | 1 mm, 0.01° 안 |
| marker 하나가 가려지면 | 도킹할 수 없다 | 남은 하나로 접근하고 진입한다 (13장) |

- **좋아지는 것**: 멀리서 본 방향 (접근 초반에 좌우로 흔들리며 방향을 찾는 일이 줄고, staging 을 더 멀리, 더 비스듬히 둘 수 있다), 뒤집힌 해가 없어진다, 한쪽이 가려지거나 반사로 안 보여도 계속 간다, 붙은 자리에서 한 장의 정밀도가 좋아져 흔들림이나 조명이 나쁠 때 여유가 생긴다.
- **그대로인 것**: 카메라 TF 처럼 늘 같은 방향으로 틀리는 고정 오차. marker 둘로도 도면 값 목표는 옆 -7 mm, 1.5° 가 남았다 (13장). 가르친 자세가 여전히 필요하다. 그리고 시뮬레이션에서는 marker 하나로도 마지막에 1 mm 안에 서므로, 끝난 자리의 숫자는 달라지지 않는다.

**로직에서 달라지는 것**

| 곳 | marker 하나 | marker 둘 이상 |
|---|---|---|
| 설정 (`docks`) | `marker_id: 0` | `markers: [{id, lateral}, ...]`. `size`, `height`, `offset` 은 dock 에 한 번 적거나 marker 마다 |
| 자세 추정 (detector) | marker 의 모서리 4개로 | dock 마다 자세 하나: 보인 marker 의 모서리를 모두 한 번에 맞춘다 (9-2 의 식에서 모서리 8개). 시작점은 marker 마다의 IPPE 해 |
| 메시지 (`ArucoDetection`) | | `marker_ids` (자세에 쓴 marker), `markers_total` (그 dock 의 marker 수) 가 더해졌다 |
| 화면 끝 여유 `border_px` | 그 marker 의 여유 | 가장 여유 있는 marker 의 값. 하나가 화면 밖으로 나가도 다른 하나로 자세가 나오기 때문이다 |
| `camera_distance` | | 카메라에서 marker 가 붙은 면까지의 수직 거리로 바꿨다. marker 가 정면에 없으면 marker 중심까지의 거리는 면까지의 거리보다 길다 |
| 필터, 접근, 진입 (BT 노드) | | **바뀌지 않는다.** 받는 것은 여전히 "dock 의 자세와 공분산" 이다. 공분산이 작아진 만큼 필터가 저절로 더 믿는다 |
| 서서 재기 (`ArucoRefine`) | | 전부 보인 영상만 쓴다 (`stages.refine.all_markers: true`). 일부만 보고 푼 자세는 가르칠 때(전부 보고)와 다른 고정 오차를 가진다. 전부 보이지 않으면 실패한다. 하나로도 끝내게 하려면 `false` |
| 가르치기 (`aruco_teach.py`) | | 전부 보인 영상만 쓰고, 본 marker 번호를 파일에 적는다. dock 의 marker 구성이 달라지면 그 파일은 쓰이지 않는다 |
| 깊이 평면 (선택) | marker 둘레 | 두 marker 를 감싸는 영역 |
| 인쇄 도구 | | dock 의 marker 를 한 장에 모두 찍는다 |

**dock_a 의 marker 를 둘로 늘리는 순서**

1. `docking.yaml` 에서 `dock_a` 의 `marker_id: 0`, `lateral: 0.0` 두 줄을 `markers: [{id: 0, lateral: -0.08}, {id: 1, lateral: 0.08}]` 로 바꾼다.
2. `python3 tools/make_aruco_dock.py config/docking.yaml dock_a marker_dock_a` → A4 가로에 100 % 로 인쇄한다. 50 mm 눈금과 marker 한 변, 두 marker 의 중심 간격(160.0 mm)을 잰다.
3. 수평으로, 종이의 가운데가 붙은 자리의 카메라 렌즈 정면에 오게 붙인다. 가운데가 틀리면 한쪽 여유가 줄어든다.
4. detector 를 다시 띄운다 (`docks` 는 시작할 때 읽는다).
5. `/dock/aruco_debug` 의 첫 줄이 `dock_a id 0,1 (2/2)` 인지, 붙은 자리에서도 `(2/2)` 인지 본다.
6. 다시 가르친다 (12장). 전에 가르친 파일은 marker 구성이 달라 쓰이지 않는다.

**셋 이상은**: 같은 형식으로 `markers` 에 더하면 코드는 그대로 돈다. 다만 15 cm 에서는 가로로 0.32 m 만 보여서 더 벌릴 수 없고, 위아래(세로 0.18 m)로 더하면 pitch 만 좋아진다. 15 cm 도킹에서는 둘이 실질적인 최대다. 더 멀리 붙는 설비라면 간격을 넓힌다: marker 의 바깥 끝이 "붙은 자리에서 가로로 보이는 범위의 절반 - 50 mm" 안에 오게.

### 8-4. 인쇄와 부착

1. `python3 tools/make_aruco_dock.py config/docking.yaml dock_a marker_dock_a` → `marker_dock_a.svg` (mm 단위). 브라우저나 Inkscape 에서 **실제 크기(100 %)** 로 인쇄한다 ("용지에 맞춤" 을 끈다). 같이 찍히는 50 mm 눈금을 자로 재서 배율을 확인한다. marker 가 둘인 dock 은 A4 가로 한 장에 나온다.
2. marker 한 변을 캘리퍼로 잰다. 60.0 mm 가 아니면 `docking.yaml` 의 `size` 를 잰 값으로 고친다 (1 % 틀리면 거리가 1 % 틀린다).
3. 무광 용지에 레이저 프린터로. 단단하고 평평한 판(아크릴, 알루미늄 복합판)에 붙이거나 설비의 평평한 면에 바로 붙인다. 휘거나 들뜨면 그만큼 자세가 틀린다. 흰 여백을 자르지 않는다.
4. 높이: marker 중심 = 카메라 렌즈 높이 (`docks.*.height`). 수준기로 수평을 맞춘다.
5. 로봇 그림자가 marker 를 덮으므로 설비 쪽 조명이 일정한지 본다. 반사되는 코팅지, 유리 뒤는 피한다.

### 8-5. 15 cm 때문에 바뀐 것

거리 값 말고 달라진 것들이다.

| 걱정 | 한 것 |
|---|---|
| 가까워서 marker 가 화면 밖으로 나간다 | marker 크기, 간격, 높이를 위처럼 정했다. 접근 중에는 "깔때기" 가 옆과 방향을 먼저 맞추므로 (10-2) marker 가 화면 가운데로 온다 |
| 잘린 marker 로 틀린 자세를 낸다 | detector 는 모서리가 화면 끝 6 px 안에 걸린 marker 를 버린다. 검출마다 화면 끝까지의 여유(`border_px`)를 같이 보낸다 |
| marker 를 못 보는데 설비 쪽으로 계속 간다 | **설비 쪽으로 가는 것은 marker 를 0.5 초 안에 보았고 화면 끝에서 10 px 안쪽일 때만.** 아니면 옆과 방향만 고쳐서 marker 를 화면 가운데로 되돌린다. 2 초 동안 못 보면 멈추고 그 자리에서 다시 본다 (`safety.fresh_detection`, `min_border_px`, `stages.*.lost_timeout`) |
| 설비에 부딪힌다 | 카메라가 marker 가 붙은 면에 `safety.min_camera_distance` (0.10 m) 보다 가까워지면 멈추고 실패한다. 마지막 구간은 0.05 m/s, 지나치면 그만큼만 되돌린다 |
| 깊이 카메라가 가까이서 못 잰다 | D455 의 깊이는 약 0.4 ~ 0.5 m 부터 나온다. 붙은 자리의 판단은 컬러 영상만으로 한다. 깊이는 멀리서 방향을 돕는 데만 쓴다 (9-3) |
| 컬러 카메라 초점 | D455 의 컬러는 고정 초점이라 0.16 m 에서 조금 흐릴 수 있다 (실제 카메라에서 확인하지 못했다). 칸이 10 mm 인 4x4 marker 는 흐려도 읽힌다. 흐림 때문에 생기는 모서리 편향은 고정 오차라 가르친 자세가 지운다 (12장) |

## 9. 자세 추정 (aruco_detector.py)

영상 한 장마다 로봇(base_footprint)에서 본 dock 의 바닥 위 자세 (x, y, yaw) 후보 1 ~ 2 개와 각각의 공분산을 `dock/aruco` 로 낸다 (`bt_mission_msgs/ArucoDetection`, 영상을 찍은 시각).

dock 좌표계 D: 원점 = 바닥에서 도킹 면의 가운데, x = 설비에서 로봇 쪽, z = 위. 로봇은 D 에서 (gap + 로봇 앞면까지 = 0.31 m, 0, 180°) 에 선다.

### 9-1. 왜 그냥 PnP 가 아닌가

2-4 에서 본 대로, 정사각형 하나(점 4개)의 PnP 는 IPPE 로 풀면 **해가 둘** 나온다. 정면에 가깝거나 멀어서 원근이 약하면 두 해의 재투영 오차가 거의 같아, 오차가 작은 쪽을 고르면 가끔 법선이 뒤집힌 쪽을 고른다. 뒤집히지 않아도 모서리 넷으로는 **방향(yaw)과 기울기(pitch, roll)가 잘 갈리지 않는다**: yaw 가 바뀌면 왼쪽과 오른쪽 변의 길이가 조금 달라질 뿐이다.

### 9-2. 바닥 사전분포를 둔 6자유도 MAP

설비는 바닥에 서 있고, 카메라가 로봇 어디에 달렸는지(TF)와 marker 의 높이를 안다. 그렇다고 "기울기 0, 높이 0" 을 못박으면(3자유도) 카메라 TF 의 높이가 1 cm, pitch 가 1° 만 틀려도 풀이가 방향을 비틀어 옆으로 수 cm 틀린다 (이전 구성의 실험에서 2 ~ 15 cm). 그래서 6자유도 p = (x, y, yaw, z, roll, pitch) 를 풀되 "바닥에 서 있다" 를 **사전분포**로 둔다:

```
p* = argmin_p   Σᵢ ‖ π(K, T_cam←base · T_base←D(p) · cᵢ) − uᵢ ‖² / σ_px²      (모서리 4개 = 식 8개. marker 가 둘이면 8개 = 식 16개)
             +  z² / σ_z²  +  roll² / σ_tilt²  +  pitch² / σ_tilt²             (사전분포: σ_z 1 cm, σ_tilt 1.5°)
             +  (yaw − yaw_depth)² / σ_depth²                                   (깊이를 쓸 때만, 9-3)
풀이: Levenberg-Marquardt. 시작점은 IPPE 의 두 해 (marker 가 여럿이면 marker 마다의 해 가운데 전체 오차가 작은 것과, 그것과 다른 골짜기의 것 하나)
```

- 가까이서 영상이 기울기를 잘 말해 주면 영상이 이긴다 (TF 가 틀려도 따라간다). 멀거나 정면이라 말해 주지 못하면 사전분포가 뒤집힌 해와 기울기-방향 섞임을 막는다.
- 두 시작점이 같은 자리로 모이면 후보 하나, 다른 자리로 모이면 둘 다 낸다. 고르는 것은 BT 의 필터다 (지금까지의 추정에 가까운 쪽).
- 공분산: 정보 행렬 JᵀJ 를 뒤집어 (x, y, yaw) 부분을 꺼낸다. 멀면 크고 가까우면 작다. 필터가 측정마다 무게를 다르게 주는 근거다.
- 진단: 추정한 dock 의 기울기(`tilt_deg`)와 바닥 높이(`height_offset`). 가까이서 늘 3°, 1.5 cm 를 넘으면 경고한다 (카메라 TF, marker 높이 점검).

합성 영상(D455: 가로 90°, 블러, 잡음)으로 잰 오차. 거리마다 로봇을 옆과 방향으로 조금씩 흩어 놓은 30장의 RMS 이고, "옆" 은 그 추정 한 장으로 붙었을 때 로봇이 옆으로 어긋날 거리다. 괄호는 30장 가운데 뒤집힌 후보가 같이 나온 장 수다.

1280 x 720 (fx 640 px), 방향 [°] / 옆 [mm]:

| 로봇 중심 ↔ 설비 | 60 mm 의 크기 | 하나, 60 mm | 하나, 80 mm | **둘, 60 mm, 간격 160 mm** |
|---|---|---|---|---|
| 붙은 자리 0.31 m | 240 px | 0.08 / 0.44 | 0.06 / 0.34 | **0.01 / 0.06** |
| 0.56 m = approach 가 끝나는 곳 | 94 px | 0.63 / 3.5 | 0.37 / 2.1 | **0.05 / 0.26** |
| 0.75 m = staging | 64 px | 1.11 / 6.0 | 0.81 / 4.5 | **0.16 / 0.87** |
| 1.0 m | 45 px | 3.1 / 17 (6) | 0.78 / 4.2 | **0.38 / 2.1** |
| 1.5 m | 28 px | 10.7 / 57 (20) | 4.2 / 23 (9) | 2.5 / 13 (0) |

848 x 480 (fx 424 px) 으로 받으면:

| 로봇 중심 ↔ 설비 | 60 mm 의 크기 | 하나, 60 mm | 하나, 80 mm | 둘, 60 mm, 간격 160 mm |
|---|---|---|---|---|
| 붙은 자리 0.31 m | 159 px | 0.16 / 0.88 | 0.09 / 0.52 | 0.02 / 0.10 |
| 0.56 m | 62 px | 0.98 / 5.4 | 0.49 / 2.7 | 0.10 / 0.55 |
| 0.75 m | 42 px | 1.30 / 7.0 | 1.13 / 6.1 | 0.38 / 2.1 |
| 1.0 m | 30 px | 11.9 / 64 (13) | 3.4 / 19 (4) | 1.6 / 8.8 (0) |

- 한 장의 값이다. 접근하며 필터가 수십 장을 합치고, 마지막에는 붙은 자리에서 서서 10장 넘게 합치므로 실제로 남는 오차는 이보다 훨씬 작다 (13장).
- **marker 하나로는 1 m 밖에서 본 한 장의 방향을 믿을 수 없다.** 그래서 staging 은 설비에서 0.6 ~ 0.9 m 에 두고, 한 장으로 정하지 않고 다가가며 정확해지는 측정을 필터로 합친다 (10-1).
- **marker 둘은 1 m 안의 모든 거리에서 방향 오차가 하나일 때의 1/5 ~ 1/12 이고, 뒤집힌 후보가 나오지 않는다** (8-3).
- 해상도: 848 x 480 은 1280 x 720 보다 오차가 1.2 ~ 4 배다. D455 는 1280 x 720 으로 받는다 (8-1).
- 사전분포의 효과 (marker 하나, `test_aruco_detector.py`): 0.75 m 에서 6자유도 PnP 를 그대로 쓰는 것보다 방향과 옆 오차가 작고, 카메라 TF 의 높이가 1 cm, pitch 가 1° 틀려도 붙은 자리의 추정이 0.3°, 3 mm 안에 머문다.

### 9-3. 깊이로 방향 돕기 (선택, `use_depth:=True`)

RealSense 의 컬러에 맞춘 깊이(`aligned_depth_to_color`)에서 marker 둘레 면(marker 가 둘이면 둘을 감싸는 영역)의 점들에 평면을 맞춰 방향을 재고, MAP 에 관측 한 줄로 더한다. 모서리 넷의 원근 대신 수백 ~ 수천 점으로 재므로 marker 가 작게 보이는 거리에서 방향이 좋아진다.

- Gazebo: staging(카메라 0.55 m)에서 잡은 dock 방향의 σ 가 2.8° → 0.10° (marker 하나, 카메라를 D455 화각으로 바꾸기 전에 잰 값).
- 깊이가 유효하지 않은 거리(`depth_min_range`, 기본 0.4 m 안. D455 의 깊이는 약 0.4 ~ 0.5 m 부터 나온다)에서는 저절로 빠지고 컬러만 쓴다. 즉 깊이는 staging 에서 approach 가 끝날 때까지만 돕는다. 붙은 자리의 정밀도는 깊이와 무관하다.
- marker 가 붙은 면이 평평해야 한다 (종이보다 넓게, 10 cm 사방). 깊이의 고정 편향이 있으므로 σ 를 0.3° 아래로는 믿지 않는다 (`depth_yaw_sigma_deg`).
- 실제 깊이 카메라에서는 확인하지 못했다. 처음에는 끄고(`use_depth:=False`) 쓴다. 접근 초반에 로봇이 좌우로 흔들리며 방향을 찾는 것이 보이면 marker 를 둘로 늘리는 쪽(8-3)이 먼저다: 컬러만으로 되고 가까이서도 듣는다.

그 밖에: OpenCV 4.5 (Ubuntu 22.04 의 `python3-opencv`) 와 4.7 이상을 모두 쓴다. `stamp_offset` 은 영상 stamp 에 더하는 시간이다 (드라이버의 stamp 가 실제 찍은 때보다 늦으면 음수).

## 10. 도킹 순서와 제어

![Dock BT](bt_mission/doc/dock.png)

```
find_marker   DetectArucoDock: 그 자리에서 보고, 안 보이면 제자리에서 돌며 찾는다 (10-4). 못 찾으면 작업 실패
3번까지:
  approach    ArucoServo: 설 곳 0.25 m 앞까지, 0.15 m/s, collision_monitor 켠 채. 이미 더 가까우면 건너뛴다
  insert      [collision_monitor 끔] ArucoServo: 마지막 0.25 m, 0.05 m/s
  refine      ArucoRefine: 서서 재고 고치기
  실패하면    look_again: 그 자리에 서서 다시 본다 (돌지 않는다, 뒤로 가지 않는다) → 지금 자리에서 다음 시도
```

### 10-1. dock 의 자리: Kalman 필터 (`DockFilter`)

- 측정: detector 가 낸 base 기준 자세를, 영상을 **찍은 시각**의 TF (odom ← base_footprint) 로 odom 에 옮긴다. 움직이며 찍은 영상이 50 ~ 150 ms 늦게 와도 그만큼 틀리지 않는다.
- 상태: D 의 odom 자세 (x, y, yaw). 설비는 움직이지 않으므로 예측은 "그대로" 이고, odom 이 흐르는 만큼만 불확실도를 키운다: 시간에 비례(`odom_drift`) + **움직인 거리에 비례**(`odom_slip`, `odom_yaw_per_m`. 메카넘은 옆으로 갈 때 특히 크다).
- 갱신: 측정마다 공분산이 다르므로 멀리서 본 것보다 가까이서 본 것이 수백 배 크게 반영된다. 다가갈수록 저절로 정확해진다.
- 후보가 둘이면 필터에 Mahalanobis 거리로 가까운 쪽. 거리² > 16.3 (99.9 %) 이면 버리고, 8번 연달아 버리면 다시 잡는다.
- 서서 볼 때는 여러 장을 합친다 (`fuseDetections`: 다수결로 자리를 잡고, 장마다 그 자리에 가까운 후보로 다시 정보 가중 평균).

### 10-2. 제어: dock 좌표계의 오차와 깔때기 (`mecanumStep`)

```
dock 좌표계의 속도:  v_along = -k_along · along · gate · advance     (설비 쪽으로 갈 때만 gate, advance)
                    v_lat   = -k_lat · lat
                    ω       = -k_yaw · yaw
gate (깔때기): 남은 거리 d 에서 폭 w(d) = 4 mm + 0.15 · d. 옆 오차가 w/2 안이면 1, w 밖이면 0, 그 사이는 직선
               (방향도 같은 식: 0.6° + 14°/m · d). 옆과 방향의 gate 를 곱한다
advance: marker 를 방금 보았고 화면 안쪽에 있으면 1, 아니면 0 (8장)
로봇 좌표계로 돌려서 축마다 최대 속도 (방향은 유지), 최소 속도(데드밴드), 명령 배율, 가속도 제한
```

- 메카넘이라 세 축을 동시에 줄이고, 길은 dock 좌표계에서 곧다.
- 깔때기: 옆과 방향이 남은 거리에 비해 크게 틀리면 앞으로 가지 않고 옆과 방향만 고친다. 비스듬히 들어가 모서리가 먼저 닿는 일을 막고, marker 를 화면 가운데에 둔다.

| 메카넘의 성질 | 한 것 |
|---|---|
| 옆으로 갈 때 롤러가 미끄러진다 (명령의 80 ~ 95 %) | 끝까지 영상으로 닫힌 제어. 옆 속도와 가속도를 낮게. 잰 배율 `scale_vy` 로 미리 보정. 필터의 불확실도를 간 거리에 비례 |
| 옆으로 가면 조금씩 돈다 | 세 축을 동시에 제어 |
| 아주 느린 명령은 안 움직인다 | 축마다 최소 속도 (`min_vx`, `min_vy`, `min_wz`) |
| 롤러가 바뀔 때의 진동 | 마지막 판단은 서서 본 영상으로 (10-3) |

### 10-3. 서서 재고 고치기 (`ArucoRefine`)

```
반복 (최대 3번):
  서기 0.5 s
  1 s 동안 marker 를 보고 합친다. 그동안 측정이 1.5 mm 넘게 흩어져 있으면 (아직 흔들린다) 4 s 까지 더 기다린다
  (marker 가 둘인 dock 은 둘 다 보인 영상만 쓴다: all_markers)
  남은 오차가 1.5 mm, 1.5 mm, 0.2° 안이면 끝
  아니면 odom 으로 그만큼 고친다 (0.03 m/s)
끝나면 서서 잰 마지막 오차를 dock/final_error 로 낸다 (기록해서 추이를 본다)
```

미끄러져 덜 가면 다음 측정이 잡아서 다시 고친다. odom 의 정확도와 상관없이 마지막 mm 를 맞춘다.

### 10-4. 안 보일 때, 놓쳤을 때: 뒤로 가지 않고 그 자리에서

물러나도 더 잘 보이는 것이 아니고 (못 찾는 이유는 대개 방향이나 잠깐 가려짐), 뒤에 레이저보다 낮은 물건이 있을 수 있다. 그래서 그 자리에서 해결한다.

- **처음 찾기** (`find_marker`): 서서 3 s 본다. 안 보이면 처음 방향에서 +30°, -30°, +60°, -60° 로 제자리 회전하며 다시 본다 (화각 90° 와 합쳐 ±105°). 돌기 전과 도는 동안 레이저로 몸이 쓸고 갈 원 (`robot.turn_radius` + `detect.clearance`) 안을 보고, 무엇이 있으면 돌지 않는다. detector 가 아예 없으면 바로 실패한다.
- **들어가다 놓쳤을 때** (`look_again`): 그 자리에 서서 다시 본다 (돌지 않는다). 다음 시도의 approach 는 `hold_if_closer` 라 이미 더 가까우면 건너뛰고, 설비에서 멀어지는 쪽 명령은 내지 않는다.

## 11. undock 두 가지

| | `undock` (기본) | `undock_precise:dock_a` |
|---|---|---|
| 방법 | 지금 방향 그대로 곧게 뒤로 0.4 m (`BackOff`, odom 기준) | marker 를 보며 dock 축을 따라 물러나 "붙은 자리 + 0.40 m" 에 선다 (`ArucoServo`, stage `retreat`) |
| 뒤 확인 | 레이저로 몸이 지나갈 사각형을 본다. 막혀 있으면 선다 | 같다 |
| 선 자리 | 붙어 있던 자세와 바퀴 미끄러짐을 그대로 물려받는다 | 설비 기준으로 정해진다: 옆과 방향을 맞추며 나온다 |
| marker 가 안 보이면 | 상관없다 | 단순 undock 으로 바꿔서 한다 |
| 쓸 때 | 보통. 설비에서 떨어지기만 하면 될 때 | 나온 자리가 정확해야 할 때: 좁은 통로, 바로 옆 설비로 이동, 나온 자리에서 다시 정밀 작업 |

![UndockPrecise BT](bt_mission/doc/undock.png)

Gazebo (미끄러짐 + 카메라 TF + marker 크기 오차를 넣은 조건, 참값. marker 하나, 카메라를 D455 화각으로 바꾸기 전에 잰 값):

| | 선 자리 (설비 면에서 로봇 중심) | 옆 | 방향 |
|---|---|---|---|
| `undock_precise`, 3회 | 697.7 ~ 698.3 mm (흩어짐 0.6 mm) | -0.1 ~ -0.2 mm | 0.00 ~ 0.01° |
| `undock`, 가르친 자세로 붙은 뒤 2회 | 692.8 ~ 694.6 mm | -0.1 ~ -0.4 mm | 0.00 ~ 0.01° |
| `undock`, 도면 값으로 붙은 뒤 (옆 -6 mm, 1.5° 틀린 채) 2회 | 689.0 ~ 690.7 mm | **+3.5 ~ +3.7 mm** | **1.5°** |

- 단순 undock 은 곧게 뒤로 가므로 붙어 있던 자세의 오차가 그대로 남고(방향 1.5° 면 0.4 m 뒤에서 옆 1 cm), 앞뒤는 미끄러진 만큼 덜 간다.
- marker 둘 (dock_b, 차이 없는 조건, D455 화각) 의 `undock_precise` 한 번: 705.0 mm (목표 710, 허용오차 5 mm), 옆 -1.2 mm, 방향 0.00°.
- 정밀 undock 은 선 자리가 0.6 mm 안에서 되풀이된다. 710 mm 가 아니라 698 mm 에 선 것은 넣어 둔 marker 크기 오차 2 % 때문이다 (늘 같은 값이라 되풀이에는 영향이 없다. 실제로는 marker 를 재서 `size` 를 맞춘다).

## 12. 가르친 자세: 고정 오차 지우기

도면 값(`gap`)으로 설 곳을 계산하면 카메라 TF, marker 의 크기와 붙인 자리, 카메라 내부값, 서브픽셀 모서리의 편향이 모두 들어간다. 각각 mm 와 0.x° 단위로 틀리기 쉽고, **몇 번을 봐도 그대로 남는다.** 로봇이 서서 다시 재도 같은 틀린 값으로 재므로 스스로 알아채지 못한다 (13장).

**가르친 자세**: 로봇을 한 번 손으로(조이스틱으로) 정확한 도킹 자리에 놓고 `aruco_teach.py` 로 그때 detector 가 본 dock 의 자세 q* 를 남긴다. 설 곳 = (q*)⁻¹.

```bash
ros2 run bt_mission aruco_teach.py --ros-args -p dock:=dock_a     # ~/.ros/aruco_taught/dock_a.yaml
```

```
detector 가 보는 것      q = f(로봇 자세; TF, marker 치수, 내부값, 편향)     (f 는 틀렸지만 매번 같다)
설 곳에 섰을 때          q = q*  ⇔  f(지금) = f(가르칠 때)  ⇔  지금 = 가르칠 때
```

- "정확한 도킹 자리" 는 설비 작업이 실제로 되는 자리다 (지그, 게이지 블록, 설비의 기준면으로 맞춘다). 가르친 자세의 정밀도는 이때 로봇을 얼마나 잘 놓았는지가 정한다.
- `DetectArucoDock` 는 `~/.ros/aruco_taught/<dock>.yaml` 이 있으면 그것을 설 곳으로 쓴다 (없으면 도면 값). 로그의 `target (taught ...)` 로 확인한다.
- 카메라를 다시 달거나 marker 를 다시 붙이면 다시 가르친다. dock 마다 가르친다.
- marker 가 여럿인 dock 은 전부 보인 영상으로만 가르치고, 그때 본 marker 번호를 파일에 적는다 (`marker_ids`). dock 의 marker 구성을 바꾸면 그 파일은 쓰지 않고 (`taught target not used ... other markers`) 도면 값으로 가면서 다시 가르치라고 알린다.
- `aruco_teach.py` 가 적는 `robot_in_dock` 과 도면 값 (0.31 m, 0, 180°) 의 차이가 TF 와 치수의 오차다. 13장에서는 "313.6 mm, +6.4 mm, 178.50°" (marker 둘이면 311.0 mm, +6.4 mm, 178.50°) 로, 넣은 오차가 그대로 드러났다.

## 13. 시뮬레이션 실험

실제 로봇에서 생길 차이 셋을 Gazebo 에 일부러 넣고, 무엇이 그것을 지우는지 보았다.

| 넣은 차이 | 방법 | 값 |
|---|---|---|
| 메카넘 미끄러짐, 바퀴 odom | `mecanum_slip:=True`. 몸체는 명령의 일부만 가고, odom 은 바퀴처럼 명령을 적분한다 | 앞 98 %, 옆 90 %, 회전 96 %, 옆 1 m 에 0.04 rad 돎, 속도 잡음 2 % |
| 카메라 TF | `camera_tf_error:=1.5,0.0,0.01`. TF 의 카메라 자리만 바꾼다 | yaw 1.5°, 옆 1 cm |
| marker 크기 | `marker_scale:=1.02`. detector 가 marker 를 실제보다 2 % 크다고 믿는다 | 2 % |

staging 세 곳(로봇 중심이 설비 면 0.70 ~ 0.75 m 앞, 옆 0 ~ 0.25 m, 방향 0 ~ 15° 틀어짐)에서 `dock` → 끝난 자리를 Gazebo 참값으로 잼 → `undock` 을 되풀이했다. 카메라는 D455 의 화각에 848 x 480 이다 (1280 x 720 은 이 환경에서 Gazebo 가 무거워 주행이 끊겼다. 실제 권장보다 불리한 조건이다).

| | 조건 | marker | 횟수 | 앞뒤 [mm] (+ 덜 감) | 옆 [mm] | 방향 [°] |
|---|---|---|---|---|---|---|
| E0 | 차이 없음, 도면 값 | 하나 | 3 | -0.7 ~ +0.9 | -0.3 ~ -0.4 | 0.01 ~ 0.02 |
| | | 둘 | 3 | +0.5 ~ +0.9 | -0.4 | 0.00 |
| E1 | 차이 넣음, 도면 값 | 하나 | 2 | -2.3 ~ -2.9 | **-5.3 ~ -6.8** | **+1.48 ~ +1.64** |
| | | 둘 | 2 | -0.2 ~ +0.6 | **-6.9 ~ -7.1** | **+1.45 ~ +1.48** |
| E2 | 차이 넣음, 가르친 자세 | 하나 | 2 | +0.5 ~ +1.4 | -0.5 ~ +0.1 | 0.02 ~ 0.05 |
| | | 둘 | 3 | -0.2 ~ +1.4 | -0.3 ~ -0.6 | 0.00 ~ 0.04 |

1. **미끄러짐은 거의 영향이 없다.** marker 를 끝까지 보며 닫힌 제어를 하므로 앞뒤 1 mm 안팎에 선다.
2. **고정 오차는 로봇이 스스로 보지 못한다.** E1 에서 로봇은 서서 재고 허용오차 안이라고 판단했지만 참값은 옆 5 ~ 7 mm, 1.5° 였다. 실제 로봇의 정밀도는 반드시 **영상이 아닌 기준**으로 재야 한다 (14-4).
3. **가르친 자세가 고정 오차를 지운다** (E2).
4. **marker 를 둘로 늘려도 고정 오차는 그대로다** (E1 의 둘: 옆 -7 mm, 1.5°). 카메라 TF 가 틀리면 marker 를 아무리 정확히 봐도 "로봇에서 본 dock" 이 그만큼 돌아가 있기 때문이다. marker 둘이 줄이는 것은 무작위 오차다: 처음 찾았을 때(`find_marker`) dock 방향의 불확실도 σ 가 하나일 때 0.9 ~ 3.4° 에서 둘일 때 0.2 ~ 0.7° 로, 붙은 자리에서 한 장의 σ 가 0.64° 에서 0.07° 로 줄었다. 시뮬레이션에서는 하나로도 마지막에 1 mm 안에 서므로 끝난 자리의 차이는 보이지 않는다. 차이가 나는 것은 영상이 나쁠 때(흔들림, 조명, 멀거나 비스듬한 staging)와 한쪽이 가려졌을 때다.
5. E1 의 앞뒤가 marker 둘에서 작은 것(-0.2 ~ +0.6)은 실험의 한계다: `marker_scale` 은 marker 의 크기만 2 % 키우고 둘 사이의 간격은 그대로 둔다. 실제 인쇄 배율이 틀리면 간격도 같이 틀려서 하나일 때처럼 앞뒤 오차가 난다.

그 밖에 확인한 것 (marker 둘에 관한 두 줄 말고는 카메라를 D455 화각으로 바꾸기 전에 잰 것이다):

| 경우 | 결과 |
|---|---|
| insert 중에 검출을 1.3 s 끊음 | 설비 쪽 전진을 0.9 s 멈췄다가 이어서 도킹. 앞뒤 -0.7 mm, 옆 -0.3 mm |
| `safety.min_camera_distance` 를 0.20 m 로 | 카메라 0.197 m 에서 "camera too close" 로 멈춤 (로봇 중심 347 mm). 더 가지 않음 |
| 설정 파일을 `docking_config` 로 바꿔 주고 이름 하나를 틀리게 | 그 파일을 읽고 `unknown name 'motion.max_vxx' (typo?)` 경고 |
| 깊이 (`camera_depth:=True`, `use_depth:=True`) | staging 에서 dock 방향 σ 2.8° → 0.10°. 붙은 자리 +0.7 mm, -0.3 mm (깊이 없이와 같다) |
| `dock_calibrate.py` (미끄러짐 조건) | scale_vx 0.995, scale_vy 1.093, scale_wz 1.053, odom_yaw_per_m 0.044 (넣은 값: 1.02, 1.11, 1.04, 0.044. marker 크기 오차 2 % 만큼 작게 나온다) |
| marker 둘 가운데 하나를 상자로 가림 | 남은 하나로 찾고, 접근하고, 진입했다 (참값 +1.8 mm, -0.3 mm, 0.03°). 서서 재기는 "둘 다 보인 영상이 없다" 로 실패했다 (`stages.refine.all_markers: true`). `false` 로 바꾸면 하나로 끝까지: +0.9 mm, -0.5 mm, -0.01° |
| marker 둘로 가르친 파일의 `marker_ids` 를 하나로 바꿔 놓음 | 그 파일을 쓰지 않고 경고와 함께 도면 값으로 도킹 |
| marker 가 화면 밖 (70° 비껴 봄), 옆에 상자, detector 없음 | 10-4 의 동작 (제자리에서 찾음 / 돌지 않고 실패 / 이유와 함께 바로 실패) |

여러 기능을 잇고 문제를 넣은 시험은 16장에 있다.

시뮬레이션이 보여 주지 못한 것: 영상의 흔들림과 모션 블러, 노출과 조명, 카메라와 odom 의 시각 어긋남, 고정 초점의 흐림, 바닥에 따라 바뀌는 미끄러짐, 실제 깊이 영상. 14장의 순서로 실제 로봇에서 재고 맞춘다.

## 14. 실제 로봇에서의 정밀도

### 14-1. 오차 예산

| 오차의 근원 | 크기 (이 구성) | 성질 | 다루는 곳 |
|---|---|---|---|
| 모서리 위치 잡음 | 0.1 ~ 0.3 px (붙은 자리에서 1 px = 0.25 mm) | 무작위 | 여러 장 합치기, 서서 재기 (10-3) |
| marker 의 방향 | 한 장으로: 붙은 자리 0.1°, 0.6 m 에서 1°, 1 m 밖은 못 믿음. marker 둘이면 그 1/7 | 무작위 + 뒤집힘 | MAP (9-2), 필터 (10-1), marker 둘 (8-3), 깊이 (9-3) |
| 카메라 TF (extrinsic) | yaw 1.5°, 옆 1 cm → 옆 6.5 mm, 방향 1.5° | **고정** | 가르친 자세 (12장) |
| marker 크기, 붙인 자리 | 2 % → 앞뒤 3 mm | **고정** | 캘리퍼로 재기, 가르친 자세 |
| 서브픽셀 모서리 편향, 초점 흐림 | 안쪽으로 0.1 ~ 0.3 px | **고정** | 가르친 자세 |
| 메카넘 미끄러짐, 데드밴드 | 옆 10 % 안팎, 수 mm/s | 바닥, 하중마다 다름 | 닫힌 제어, 서서 고치기, `dock_calibrate.py` (14-3) |
| 늦음 (영상 → 명령 → 바퀴) | 50 ~ 150 ms | 고정 | 영상 시각의 TF, 느린 마지막 접근, 서서 재기, `stamp_offset` |
| 로봇이 기운다 (가감속), 진동 | 카메라 pitch 가 순간 바뀐다 | 무작위 | MAP 의 사전분포, 서서 재되 흩어짐이 가라앉을 때까지 기다린다 |

무작위 오차는 많이 보고 평균하면 줄지만 고정 오차는 그대로 남는다. 실제 로봇에서 mm 를 맞추는 것은 **가르친 자세 + 서서 재고 고치기 + 잰 배율** 셋이 한다.

### 14-2. 카메라와 TF 점검

1. detector 만 띄우고 로봇을 손으로 설비 앞 여러 자리에 둔다. `/dock/aruco_debug` 에서 rms (< 0.5 px), 빨간 점과 초록 모서리가 겹치는지 본다.
2. 가까이(0.5 m 안)에서 `tilt` < 1°, `z` < 5 mm 가 되도록 TF(카메라 높이, pitch)와 `docks.*.height` 를 고친다.
3. 노출: 자동 노출을 끄고 짧게 고정한다 (모션 블러, 로봇 그림자로 밝기가 바뀌는 것). 서 있을 때와 움직일 때 `/dock/aruco` 의 흩어짐을 비교한다.
4. 시각: `enable_sync`, 하드웨어 시각. 영상 stamp 와 수신 시각의 차이가 일정하게 크면 `stamp_offset` 으로 보정한다.

### 14-3. 배율과 데드밴드 재기 (`dock_calibrate.py`)

로봇을 marker 정면 0.5 ~ 0.7 m 에 세우고 (옆, 뒤로 0.3 m 는 비운다. 짐을 실은 상태로):

```bash
ros2 run bt_mission dock_calibrate.py                                  # 배율 (약 1 분)
ros2 run bt_mission dock_calibrate.py --ros-args -p deadband:=True     # 데드밴드도
```

- 축마다 갔다가 돌아오며 (앞뒤 0.10 m, 옆 0.15 m, 회전 12°) **바퀴 odom 이 말하는 이동**과 **실제 이동**을 비교한다. 실제 이동은 로봇에서 본 marker 의 자리(방위와 거리)와 IMU 의 yaw 로 잰다.
- 결과: `scale_vx`, `scale_vy`, `scale_wz` (바퀴가 간 만큼 / 실제), `odom_slip`, `odom_yaw_per_m`, (`deadband:=True`) `min_vx`, `min_vy`, `min_wz`. 화면과 `~/.ros/dock_calibration.yaml` 에 나온 값을 `docking.yaml` 의 `motion` 에 옮겨 적는다.
- IMU 가 없으면 marker 의 방향을 쓰는데 덜 정확하다 (경고가 나온다). 그때는 더 가까이(0.4 ~ 0.5 m) 서서 잰다.
- 바닥이나 하중이 바뀌면 다시 잰다. 옆으로 갈 때 방향이 많이 틀어지면(`odom_yaw_per_m` 이 0.1 넘게) 바퀴 마모, 하중 치우침, 롤러를 점검한다.

### 14-4. 확인 순서

1. 가르친다 (12장).
2. `dock` → `undock` 을 20번. 매번 로봇이 선 자리를 **영상이 아닌 기준**(설비의 기준면과 다이얼 게이지, 바닥 눈금, 작업 결과)으로 잰다.
3. `ros2 topic echo /dock/final_error` 를 같이 기록한다. 로봇이 스스로 잰 오차(늘 허용오차 안)와 바깥 기준의 차이가 고정 오차다.
4. 평균(치우침)이 크면 다시 가르치거나 TF 를 고친다. 흩어짐이 크면 허용오차, 최소 속도, 배율, 노출을 본다.
5. 오래 쓰면서 `final_error`, `tilt`, 다시 잡은 횟수(`re-initialized`)가 늘면 카메라가 움직였거나 marker 가 손상된 것이다.

### 14-5. 더 높이려면

1. marker 를 둘로 (8-3). 구현되어 있다: `docks` 의 `markers:` 목록. 방향이 원근이 아니라 두 marker 의 위치 차이로 정해져서 멀리서도 방향이 정확하다.
2. 깊이 (9-3) 또는 레이저로 설비 면의 직선을 맞춰 방향을 따로 잰다.
3. 컬러 1280 x 720 (8-1). D455 의 컬러는 global shutter 라 움직이며 찍어도 찌그러지지 않는다.
4. IMU 와 섞은 odom (robot_localization): 접근 중 방향 흐름이 준다.
5. 기계적 마무리: 설비 쪽 V 자 가이드나 핀이 남은 1 ~ 2 mm 를 받아 준다.

## 15. find_marker 가 실패할 때

`find_marker failed` 이면 task_server 로그의 `[aruco] dock '...' not found (...)` 줄이 이유를 말한다.

| 로그 | 원인 | 할 일 |
|---|---|---|
| `aruco_detector is not running` | 검출 노드가 없다. 또는 띄웠지만 죽었다 | `simple_line.launch.py robot:=mecanum dock:=True` 는 같이 띄운다. 따로 띄웠다면 그 터미널의 에러를 본다: `bt_mission_msgs` 를 새로 빌드하지 않았거나 (`colcon build --packages-select bt_mission_msgs bt_mission simple_line_bringup` 후 `source install/setup.bash`), OpenCV/numpy 를 못 불러온다 |
| detector 가 `_ARRAY_API not found`, `numpy.core.multiarray failed to import` 로 죽는다 | pip 로 깐 numpy 2.x 가 apt 의 numpy 1.21.5 를 가린다 (OpenCV, cv_bridge 는 1.x 로 빌드되어 있다) | `aruco_detector.launch.py` 는 `PYTHONNOUSERSITE=1` 로 `~/.local` 의 pip 패키지를 보지 않게 한다. `ros2 run` 으로 직접 띄울 때는 `PYTHONNOUSERSITE=1 ros2 run bt_mission aruco_detector.py ...`. `sudo pip` 로 `/usr/local` 에 깔았다면 이 방법이 듣지 않으니 `sudo pip3 uninstall numpy` |
| `running but saw no marker` | 화면에 marker 가 없다 | 카메라가 나오는가 (`ros2 topic hz .../color/image_raw`). staging 이 설비 앞 0.6 ~ 0.9 m 이고 설비를 보는가. `/dock/aruco_debug` 를 RViz 로 |
| `saw only other docks: 'dock_a'` | 작업의 dock 이름이 다르다 | `dock:dock_a` 처럼 `docking.yaml` 의 이름으로 |
| `detections dropped: no TF odom <- base_footprint` | 영상 시각의 로봇 자리를 모른다 | TF 가 끊기지 않았는지, 시계가 섞이지 않았는지 (시뮬레이션이면 detector 는 `use_sim_time:=True`) |
| `only N usable detections` | 너무 느리다 | `ros2 topic hz /dock/aruco`. `detect.look_time` 을 늘리거나 `min_found` 를 줄인다 |
| `Did not turn to search: ...` | 옆에 무엇이 있어 돌지 않았다 | 그 이유를 치우거나 staging 자리를 옮긴다 |

들어가는 중의 실패는 `[aruco] insert: ...` 줄에 이유가 있다: `lost the marker` (2 초 못 봄), `camera too close to the marker` (`safety.min_camera_distance`), `timed out`. `[dock] could not switch off collision_monitor/Footprint` 는 안전 영역을 끄지 못한 것이다 (6-3). `[aruco] refine: marker not seen while standing (... only part of the dock's markers ...)` 는 marker 가 여럿인 dock 에서 붙은 자리에 전부 보이지 않은 것이다: `/dock/aruco_debug` 의 `(1/2)` 로 어느 것이 안 보이는지 보고, 가린 것을 치우거나 marker 의 `lateral` 과 간격을 확인한다 (8-3).

## 16. 시나리오로 시험하기

5장의 예는 기능을 하나씩 보여 준다. 이 장의 시나리오는 실제 작업처럼 **여러 기능을 잇고**, 일부러 **문제를 넣어** 로봇이 안전하게 대처하는지 본다. 모두 창고 월드에서 돌려 보았고, 끝난 자리는 Gazebo 의 참값으로 판정했다. 시나리오가 찾아낸 문제를 고치는 동안(16-12) 돌린 결과도 있다. 도킹 쪽(marker 를 보고 붙는 부분)은 S4 의 수정 뒤로 바뀌지 않았고, controller 의 도착 판정은 세 번 바꿨다. 마지막 판으로는 S2, S6 과 도착 시험을 다시 돌렸다. 실제 로봇에서는 좌표만 바꿔 같은 순서로 한다 (16-11).

![scenario map](docs/scenario_map.png)

| | 시나리오 | 이어지는 기능 | 넣는 문제 | 실행 | Gazebo 결과 |
|---|---|---|---|---|---|
| S1 | 점검 순회 → 도킹 → 작업 → 정밀 undock → 복귀 | inspect, scan, dock, wait, undock_precise, look_around, go | | 파일 | 두 번 모두 6개 작업 성공 (약 2분 50초). 붙은 자리 +0.3 ~ +0.5 / -0.4 mm, 0.00° |
| S2 | 설비 두 대를 차례로 (marker 하나 → 둘) | dock 두 번, undock_precise, undock | | 파일 | 두 번 모두 성공. A +0.1 ~ +0.2 / -0.0 ~ -0.3 mm, 0.05° 안, B +0.6 ~ +0.8 / -0.4 mm, 0.00° |
| S3 | staging 에서 marker 가 화면 밖 | 제자리에서 돌며 찾기, 비스듬하고 먼 접근 | 시야 | 파일 | 두 번 모두 세 번 돌아 찾음 (18.8 ~ 18.9 s). +0.4 ~ +0.9 / -0.4 ~ -0.6 mm, 0.03° 안 |
| S4 | 들어가는 도중 marker 가 가려짐 | 놓침 → 멈춤 → 다시 보기 → 이어서 붙기 | 기둥 | 파일 + 손 | 가려진 6 s 동안 그 자리에 섬. 치우자 이어서 +0.5 mm |
| S5 | 도킹 중 취소, 선점 | 안전 영역 되켜기 | 사람의 명령 | 대화형 | 1 s 안에 다시 켜지고 로봇은 섬 |
| S6 | 붙어 있는 동안 뒤가 막힘 | undock 의 레이저 뒤 확인 | 기둥 | 파일 + 손 | 0.1 ~ 0.2 s 만에 섬 (움직이지 않음). 치운 뒤 물러남 |
| S7 | staging 으로 가는 길의 장애물 | stop & go, 도킹 | 기둥 | 파일 + 손 | 0.53 m 앞에서 섬, 치우고 2 s 뒤 출발, 도킹 성공 |
| S8 | 실제 로봇과의 차이 + 가르치기 + 되풀이 | aruco_teach, 정밀도 | 미끄러짐, 카메라 TF, marker 크기 | 스크립트 | 도면 값: 옆 -6.3 mm, 1.54° (차이가 남음). 가르친 뒤 4번 모두 1.4 mm, 0.03° 안 |
| S9 | 틀린 입력 | 실패 이유, 안전한 실패 | 없는 dock, 다른 설비, 틀린 형식 | 파일 | 기대대로 실패하고 이유를 알려 줌 |

### 16-1. 준비

```bash
# 터미널 1: 창고 + 메카넘 + 설비 두 대 (dock_a: marker 하나, dock_b: marker 둘) + marker 검출
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True dock_markers:=both
# 터미널 2
ros2 run bt_mission task_server
# 터미널 3: 시나리오 파일을 돌린다
SC=$(ros2 pkg prefix bt_mission)/share/bt_mission/scenarios
ros2 run bt_mission task_client.py --file $SC/s1_patrol_dock.txt
# 터미널 4: 장애물을 놓고 치우기, 참값 보기
ros2 run simple_line_bringup obstacle.py put 4.47,1.0 --radius 0.05
ros2 run simple_line_bringup dock_truth.py dock_a
```

| 도구 | 하는 일 |
|---|---|
| `task_client.py --file <파일>` | 파일의 줄을 차례로 한다. 줄의 형식은 대화형과 같다 (작업 한 줄, `wait <초>`, `#` 주석). **하나라도 실패하면 남은 줄은 하지 않는다** (`--keep-going` 이면 계속). 끝에 작업마다 결과와 걸린 시간, 종료 코드 0 = 모두 성공. `task_client.py < 파일` 로 넣어도 같다 |
| `wait <초>` | 앞 작업이 끝난 뒤 그만큼 기다린다 (설비 작업 흉내). 대화형에서도 쓴다 |
| `dock_truth.py <dock>` (simple_line_bringup) | Gazebo 참값으로 로봇이 dock 좌표계의 어디에 섰는지. 허용오차(`--tol`, 기본 3 mm, 3 mm, 0.5°) 안이면 `PASS` 와 종료 코드 0. `--target 0.71,0,180` 은 undock 뒤의 자리, `--place` 는 로봇을 그 자리에 옮겨 놓는다 (가르치기) |
| `obstacle.py put x,y [--radius r]`, `remove` | Gazebo 에 기둥을 놓고 치운다 ([README.md](README.md) 6-1). 뜨는 데 몇 초 걸린다 |
| `dock_markers:=both` | 설비 두 대. Gazebo 의 모형 이름이 dock 이름과 같다 (`dock_a`, `dock_b`) |

시나리오 파일은 `bt_mission/scenarios/` 에 있다. 파일 끝의 요약은 이렇게 나온다 (S1):

```
summary:
  t1   inspect                SUCCEEDED    31.6 s  done
  t2   scan                   SUCCEEDED    27.5 s  done
  t3   dock:dock_a            SUCCEEDED    21.8 s  done
  t4   undock_precise:dock_a  SUCCEEDED     8.6 s  done
  t5   look_around            SUCCEEDED    18.9 s  done
  t6   go                     SUCCEEDED    46.1 s  done
  ALL SUCCEEDED
```

- 시나리오는 출발점 (-4, -2.5) 에서 시작해 거기로 돌아온다 (S6 만 붙은 자리에서 멈춘다). 이어서 돌려도 된다.
- 이 시뮬레이션의 planner 는 점과 점을 직선으로 잇고, 그 선이 물체에 닿으면 실패한다. 점은 선이 물체에서 0.4 m 넘게 떨어지게 골랐다. 동쪽 통로는 x = 4.0 으로 지난다: 처음에는 staging 과 같은 x = 3.85 로 했는데, 선반 B 끝에서 0.35 m 라 위치 추정이 몇 cm 흐르는 조건(S8 의 미끄러짐)에서 controller 가 선반 모서리를 길 위의 장애물로 보고 섰다. 좌표를 바꿀 때는 두 점 사이의 직선과 그 여유를 확인한다.
- 설비 모형은 지도에 없다. 그래서 설비 앞에서는 AMCL 의 위치가 참값과 5 ~ 7 cm 틀렸다 (레이저가 벽 대신 설비를 본다). staging 은 그 정도 틀려도 된다 (도킹은 marker 로 한다). 실제 현장의 설비도 지도에 넣는 것이 좋다.

### 16-2. S1: 점검 순회 → 도킹 → 작업 → 정밀 undock → 복귀

하루 일과 한 바퀴. 내비게이션 작업과 도킹 작업이 섞여도 서로 방해하지 않는지 본다.

```
inspect:5 -1.0,-2.5 1.5,-2.0,90      # 선반 B 앞에서 선반을 보고 5 초 점검
scan 4.0,-2.5 4.0,0.0                # 동쪽 통로를 지나며 0.5 m 마다 스캔
dock:dock_a 3.85,1.0,0               # 설비 A 앞 staging 에서 도킹
wait 10                              # 설비 작업 10 초
undock_precise:dock_a                # marker 를 보며 물러나 설비 기준 0.4 m 뒤에 선다
look_around 2.0,0.3                  # 선반 사이 빈 곳에서 한 바퀴
go -1.0,0.0 -1.0,-2.5 -4.0,-2.5,0    # 선반 B 를 돌아 출발점으로
```

| 합격 기준 | 확인한 결과 |
|---|---|
| 요약이 `ALL SUCCEEDED` | 두 번 모두 6개 성공. 점검 31.5 s, 스캔 27.5 s, 도킹 22 ~ 24 s, undock 9 s, 둘러보기 19 s, 복귀 46 ~ 53 s |
| `wait 10` 동안 `dock_truth.py dock_a` 가 PASS | +0.5 mm, -0.4 mm, 0.00° 와 +0.3 mm, -0.4 mm, 0.00° |
| undock 뒤 collision_monitor 가 다시 켜짐 (`[dock] collision_monitor: Footprint on again`) | 켜짐 |

### 16-3. S2: 설비 두 대를 차례로

marker 하나짜리 설비 A 와 marker 둘짜리 설비 B 를 한 작업 묶음에서 차례로 들른다. 다른 설비의 marker 가 섞이지 않는지, 정밀 undock 뒤 바로 다음 설비로 가는 이동이 되는지 본다.

```
dock:dock_a 4.0,-2.5 3.85,1.0,0      # 설비 A
wait 10
undock_precise:dock_a
dock:dock_b 4.0,0.0 3.85,-2.0,0      # 남쪽으로 3 m 가서 설비 B
wait 10
undock                               # B 에서는 곧게 뒤로
go 3.85,-2.5 -4.0,-2.5,0
```

| 합격 기준 | 확인한 결과 |
|---|---|
| `ALL SUCCEEDED` | 두 번 모두 5개 성공 (도킹 A 67 s 안팎, B 40 ~ 45 s) |
| 두 번의 `wait 10` 동안 각각 `dock_truth.py dock_a`, `dock_truth.py dock_b` 가 PASS | A: +0.2 mm, -0.0 mm, 0.05° / +0.1 mm, -0.3 mm, 0.01°. B: +0.8 mm, -0.4 mm, 0.00° / +0.6 mm, -0.4 mm, 0.00° |
| B 에서는 두 marker 를 다 보고 붙는다 (`/dock/aruco_debug` 첫 줄이 `dock_b id 2,3 (2/2)`) | 붙은 자리의 화면은 8-3 의 그림과 같다 |

처음 돌렸을 때 이 시나리오는 설비 B 의 staging 에서 멈춰 450 s 뒤 실패했다. controller 의 문제였고 고쳤다 (16-12).

### 16-4. S3: staging 에서 marker 가 화면 밖

staging 이 비스듬하고(-50°) 조금 멀어서(설비 면에서 0.9 m, 옆 0.4 m) 도착했을 때 marker 가 화면 밖에 있다. 뒤로 물러나지 않고 그 자리에서 레이저로 주변을 확인한 뒤 돌며 찾는지(10-4) 본다.

```
dock:dock_a 4.0,-2.5 4.0,0.0 3.65,0.6,-50     # 선반 B 모서리를 피해 (4.0, 0) 을 거친다
undock_precise:dock_a
go 4.0,0.0 4.0,-2.5 -4.0,-2.5,0
```

| 합격 기준 | 확인한 결과 |
|---|---|
| `ALL SUCCEEDED` | 두 번 모두 성공 |
| 로그에 `not seen: turning in place to +30 deg` ... `seen while turning: stopping here to look` | +30°, -30° 에서는 안 보이고 +60° 로 도는 도중 보여서 멈춤. `found 'dock_a' ... (8 of 8, 3 turn(s), 18.9 s)` (두 번째 18.8 s) |
| 붙은 자리 PASS | +0.9 mm, -0.4 mm, 0.01° 와 +0.4 mm, -0.6 mm, 0.03° |

두 번째 실행에서 이어지는 `undock_precise` 가 한 번 `only 1 usable detections` 로 marker 를 잡지 못하고, 설계대로 단순 undock 으로 곧게 물러났다 (성공). 그 순간 참값을 기록하는 프로세스 둘이 같이 떠서 시뮬레이션이 느려졌던 때다. 실제 로봇에서 이 줄이 자주 보이면 `ros2 topic hz /dock/aruco` 로 검출 빈도를 본다 (15장).

옆에 물건이 있어 돌 수 없을 때(레이저가 몸이 쓸고 갈 원 안에 무엇을 보면 돌지 않고 `Did not turn to search: ...` 로 실패한다)는 13장에서 확인했다. 이 창고에서 그대로 만들기는 어렵다: 돌 원(0.27 m) 안의 물건은 collision_monitor 의 원(0.3 m) 안에도 들어서, 로봇이 staging 에 닿기 전에 collision_monitor 가 먼저 세운다.

### 16-5. S4: 들어가는 도중 marker 가 가려짐

사람이나 케이블이 잠깐 marker 앞을 가린다. 가려진 동안 설비 쪽으로 가지 않고(8-5 의 안전), 다시 보이면 뒤로 물러나지 않고 그 자리에서 이어서 붙는지 본다.

```bash
ros2 run bt_mission task_client.py --file $SC/s4_marker_hidden.txt
# 터미널 3 에 "RUNNING insert" 가 나오면, 터미널 4 에서 marker 앞 7 cm 에 가는 기둥을 세운다
ros2 run simple_line_bringup obstacle.py put 4.47,1.0 --radius 0.05
# 5 ~ 10 초 뒤 치운다
ros2 run simple_line_bringup obstacle.py remove
```

기대하는 흐름 (`task_server` 로그):

```
[aruco] insert: lost the marker in 3.4 s, along +174.2 mm, ... (held 1.4 s for the marker)   2 s 동안 못 봄 -> 멈추고 실패
[dock] collision_monitor: Footprint on again
[aruco] look_again: looking for dock 'dock_a' from here                                      그 자리에 서서 다시 본다 (돌지 않는다)
[aruco] dock 'dock_a' not found (not seen from here) after 3.5 s                            아직 가려져 있다 -> 다음 시도
[aruco] approach: already 76 mm closer than its goal, not backing up                         물러나지 않는다
[aruco] insert (stage insert): ...                                                           기둥을 치우면 여기서 이어서
[aruco] insert: reached in 7.3 s, along +2.1 mm, ...
[aruco] refine: done after 1 correction(s), 3.4 s
```

| 합격 기준 | 확인한 결과 |
|---|---|
| 가려진 동안 설비 쪽으로 가지 않음 (참값을 0.25 s 마다 잼) | 가려진 6 s 동안 설비 면에서 482.8 mm 에 그대로 섬 |
| 치운 뒤 그 자리에서 이어서 붙음 (3번 시도 안에) | 두 번째 시도에서 붙음. 310.5 mm (+0.5 mm) |
| 너무 오래 가려 두면 (세 번 다 실패) 작업이 실패하고 로봇은 그 자리에 섬 | 돌려 보지 않았다. 시도마다 약 6 s (놓침 2 s + 다시 보기 3.5 s) 이므로 15 ~ 20 s 넘게 가리면 실패한다 |

처음 돌렸을 때는 가려진 채 두 번째 시도를 시작하면서 로봇이 설비 쪽으로 25 mm 더 갔다. 고쳤다 (16-12).

### 16-6. S5: 도킹 중 취소와 선점

collision_monitor 를 끈 채 설비 앞에 붙는 도중에 사람이 작업을 취소하거나 다른 작업을 보낸다. 로봇이 서고, **안전 영역이 다시 켜지는지** 본다. 키보드로 하는 대화형 시나리오다.

```bash
ros2 run bt_mission task_client.py
task> dock:dock_a 4.0,-2.5 3.85,1.0,0
# "RUNNING insert" 가 나오면 (collision_monitor 가 꺼져 있다)
task> cancel
# 다른 터미널에서
ros2 param get /collision_monitor Footprint.enabled        # True 여야 한다
ros2 run simple_line_bringup dock_truth.py dock_a --tol 1000,50,10   # 두 번 재서 같으면 서 있는 것
task> dock:dock_a 3.85,1.0,0                                # staging 으로 돌아가 다시 붙는다
# 또는 선점: 붙는 도중에 다른 작업을 바로 보낸다
task> !go 3.0,0.3,180
```

| 합격 기준 | 확인한 결과 |
|---|---|
| 취소: 결과 `CANCELED`, 1 s 안에 `Footprint on again`, 로봇이 섬 | `t1 CANCELED`. 1 s 안에 다시 켜짐 (`Footprint.enabled = True`). 설비 면에서 374.0 mm 에 서서 움직이지 않음 |
| 취소 뒤 다시 보낸 도킹이 붙음 | 성공 (25.8 s). 설비 앞 0.21 m 에서 돌아 staging 으로 갔다가 다시 들어갔다 (돌 원이 설비에 닿지 않는 거리) |
| 선점: 앞 작업이 `ABORTED: preempted by ...`, 1 s 안에 다시 켜지고 새 작업이 돎 | `t1 ABORTED preempted by t2`, 1 s 안에 다시 켜짐, `go` 성공 |

### 16-7. S6: 붙어 있는 동안 뒤가 막힘

설비 작업 중에 로봇 뒤에 대차나 상자가 놓였다. 단순 undock 과 정밀 undock 이 모두 뒤를 보고 서는지 본다 (collision_monitor 는 undock 동안 꺼져 있으므로 이것이 유일한 보호다).

```bash
ros2 run bt_mission task_client.py --file $SC/s6_blocked_undock.txt     # dock -> wait 20 -> undock
# 도킹이 끝나 wait 20 이 도는 동안 (꼭 붙은 뒤에. 붙기 전에 놓으면 로봇과 겹친다)
ros2 run simple_line_bringup obstacle.py put 3.90,1.0 --radius 0.06    # 로봇 뒷면에서 12 cm
# undock 이 실패하고 파일이 멈춘다. 정밀 undock 도 해 본다
ros2 run bt_mission task_client.py undock_precise:dock_a               # 이것도 실패해야 한다
ros2 run simple_line_bringup obstacle.py remove
ros2 run bt_mission task_client.py undock                              # 이제 물러난다
```

| 합격 기준 | 확인한 결과 |
|---|---|
| `undock`: `something is behind the robot, stopped`, 로봇이 움직이지 않음 | 0.2 s 만에 섬. 참값 310.4 mm 그대로 |
| `undock_precise`: 같다 (`retreat: something is behind the robot, stopped`) | 0.1 s 만에 섬. 310.8 mm 그대로 |
| 치운 뒤 undock 이 됨 | `undock` 701.6 mm (목표 710 mm), `undock_precise` 701.3 mm |

### 16-8. S7: staging 으로 가는 길의 장애물

지나가는 사람이나 대차 때문에 staging 으로 가다 선다. 치우면 다시 가서 평소처럼 도킹하는지 본다 (README 의 stop & go + 도킹).

```bash
ros2 run bt_mission task_client.py --file $SC/s7_obstacle_on_route.txt
# 로봇이 출발하면 (5 s 뒤쯤) 길 위에 기둥
ros2 run simple_line_bringup obstacle.py put 0.0,-2.5
# 로봇이 그 앞에 선 것을 보고 (Gazebo, RViz) 치운다
ros2 run simple_line_bringup obstacle.py remove
```

| 합격 기준 | 확인한 결과 |
|---|---|
| 장애물 앞에서 감속해 섬 (`obstacle on the path ... slowing down to stop`) | 0.53 m 앞에서 섬 |
| 치우면 다시 감 (`path clear, going again`) | 치우고 2 s 뒤 출발 |
| `ALL SUCCEEDED` (기다린 시간이 30 s 를 넘지 않게) | 도킹 80.2 s (기다림 포함), undock, 복귀 모두 성공 |

30 s 넘게 막혀 있으면 Nav2 가 재시도를 시작한다 (README 6-3).

### 16-9. S8: 실제 로봇과의 차이, 가르치기, 되풀이

13장의 실험을 스크립트 하나로. 실제 로봇에서 생길 차이를 넣은 시뮬레이션에서 (1) 도면 값으로 붙어 차이가 남는 것을 보고, (2) 정확한 자리에서 가르치고, (3) staging 을 바꿔 가며 되풀이해 정밀도를 잰다.

```bash
ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True \
    dock_markers:=both mecanum_slip:=True camera_tf_error:=1.5,0.0,0.01 aruco_detector:=False
ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True marker_scale:=1.02
ros2 run bt_mission task_server
bash $(ros2 pkg prefix bt_mission)/share/bt_mission/scenarios/s8_repeat.sh dock_a 4     # dock, 횟수, [허용오차]
```

스크립트가 하는 일과 확인한 결과:

| 단계 | 하는 일 | 합격 기준 | 확인한 결과 |
|---|---|---|---|
| 1 | 가르친 자세 없이(있으면 `.bak` 으로 옮긴다) 도면 값 목표로 도킹하고 참값을 잰다 | 실패가 정상이다: 넣은 차이가 그대로 남는다 | -2.5 mm, **-6.3 mm, +1.54°** (FAIL) |
| 2 | `dock_truth.py --place` 로 로봇을 정확한 붙은 자리에 옮기고 `aruco_teach.py` | `robot_in_dock` 이 도면 값과 차이를 보인다 (그만큼이 TF 와 치수의 오차) | 313.6 mm, +6.5 mm, 178.50° (도면 값 310, 0, 180) |
| 3 | staging 세 곳을 돌아가며 `undock` → `dock` 을 되풀이하고 매번 참값 | 모두 허용오차(기본 2 mm, 2 mm, 0.3°) 안, 종료 코드 0 | 4번 모두 PASS: +0.6 ~ +1.4 mm, -0.1 ~ -0.3 mm, 0.00 ~ 0.03° |

`dock_b` 로도 돌릴 수 있다 (`s8_repeat.sh dock_b 4`). 횟수를 늘려 실제 로봇의 확인(14-4)과 같은 20번을 해 볼 수 있다 (한 번에 약 1분).

### 16-10. S9: 틀린 입력

사람이 틀리게 보낸 작업이 위험하지 않게, 이유를 알려 주며 실패하는지 본다. 실패가 정상인 파일이라 `--keep-going` 으로 돌린다.

```
dock:dock_c 4.0,-2.5 3.85,1.0,0      # docking.yaml 에 없는 dock
dock:dock_b 3.85,1.0,0               # 다른 설비(A) 앞에서 B 를 찾음
undock_precise:dock_b                # 붙어 있지 않은데 정밀 undock (B 가 안 보임)
inspect:abc 1,1                      # 형식이 틀린 줄
go 4.0,0.0 4.0,-2.5 -4.0,-2.5,0      # 출발점으로
```

| 줄 | 기대 | 확인한 결과 |
|---|---|---|
| `dock:dock_c` | `find_marker failed`. 돌며 찾아본 뒤 이유를 알려 줌 | `'dock_c' not found (looked everywhere it could) after 30.9 s, 4 turn(s): saw only other docks: 'dock_a'. Check the dock name ...` |
| `dock:dock_b` (A 앞) | 같은 이유로 실패. A 에 붙지 않는다 | 같은 메시지 (`saw only other docks: 'dock_a'`) |
| `undock_precise:dock_b` | B 가 안 보이므로 단순 undock 으로 0.4 m | `undock_without_marker: back off 0.40 m`, 성공 |
| `inspect:abc` | 보내지 않음 | `! duration 'abc' is not a number`, 요약에 `(1 line(s) could not be read)` |
| 요약 | `FAILED`, 종료 코드 1 | 그렇다 |

### 16-11. 실제 로봇에서

| 시나리오 | 바꿀 것 | 판정 |
|---|---|---|
| 모두 | 좌표 (출발점, staging, 점검 자리)를 현장의 지도에 맞춰 파일을 복사해 고친다. dock 이름은 `docking.yaml` 의 것 | `dock_truth.py` 대신 바깥 기준: 설비 기준면과 다이얼 게이지, 바닥 눈금, 작업 결과 (14-4). `ros2 topic echo /dock/final_error` 를 같이 기록 |
| S4 | 기둥 대신 종이나 손으로 marker 를 가린다 | 가린 동안 로봇이 설비 쪽으로 오지 않는지 (줄자, 바닥 표시) |
| S5, S6 | 비상정지를 손에 쥔다. S6 은 가볍고 무르고 레이저 높이에 걸리는 물건 (빈 상자) | S6 이 실패하면 **현장에서 쓰지 않는다**: `ros.scan_topic`, 레이저 TF, 레이저가 로봇 뒤를 보는지 (뒤가 가려지는 로봇이면 undock 거리를 줄이거나 뒤 센서를 단다) |
| S7 | 사람이 지나가거나 상자를 놓는다 | collision_monitor 와 controller 의 장애물 감속이 로봇에 맞게 설정되어 있는지 |
| S8 | `dock_truth.py --place` 대신 손(조이스틱)으로 정확한 자리에 놓고 `aruco_teach.py` | 14-4 의 20회 반복 |

권하는 순서: S9 (안전한 실패) → S6 (뒤 확인) → S5 (취소) → S1 (한 대) → S3 → S4 → S7 → S2 (두 대) → S8 (정밀도). 앞의 셋은 로봇이 설비에 붙기 전, 사람이 옆에 있을 때 먼저 확인할 안전 항목이다.

### 16-12. 시나리오가 찾아낸 것

한 기능씩 볼 때는 드러나지 않던 문제 두 개를 시나리오가 찾았고, 고쳤다.

| 찾은 시나리오 | 문제 | 원인 | 고친 것 |
|---|---|---|---|
| S2 (설비 B 의 staging), S6 (undock 뒤 같은 staging 으로 다시) | 로봇이 staging 근처에서 움직이지 않고 Nav2 가 `Failed to make progress` 로 끝내 실패했다 (450 s) | controller 의 "도착 고정": 허용오차(5 cm) 안에 한 번 들면 15 cm 밖으로 밀려나기 전에는 위치를 더 맞추지 않는다 (도착 뒤 위치 추정이 흔들릴 때 움찔하지 않게 넣은 것). 그런데 goal checker 가 처음부터 다시 볼 때 (Failed to make progress 뒤 Nav2 가 같은 goal 로 다시 시작, undock 처럼 Nav2 밖에서 움직인 뒤 같은 goal) 로봇이 5 ~ 15 cm 에 있으면, goal checker 는 "아직", controller 는 "도착" 이라 서로 기다렸다 | `simple_line_controller`: 도착해서 방향까지 맞추고 섰는데 2 s 넘게 끝나지 않고 위치가 허용오차 밖이면 다시 다가간다 (`approaching it again`). 정상적인 도착은 그 전에 끝나므로 평소 움직임은 그대로다. 같은 상황(도착 뒤 로봇을 뒤로 5.5 cm, 옆으로 8 cm 옮기고 같은 goal)을 만들어 두 번 모두 2 s 뒤 다시 다가가 도착하는 것을 확인했다. 그 뒤의 S2, S6, 도착 회전 시험(goal 6개, 방향 오차가 다시 커진 양 2.1° 이하)에서는 한 번도 풀리지 않았다. 처음에는 "허용오차의 절반 안에 들어야 도착" 으로 고쳤는데, 옆으로 3 ~ 5 cm 비껴 들어온 로봇이 goal 점을 향해 제자리에서 도는 움직임이 다시 생기고 (도착 회전 시험에서 146°), 경유점을 지나쳐 서는 일(한 번은 45 cm 지나 설비 B 아래 벽 쪽)이 생겨서 되돌렸다 |
| S4 | marker 가 가려진 채 다음 시도를 시작하면, 처음 0.5 s 동안 설비 쪽으로 갔다 (시도마다 약 25 mm) | 진입 단계(`ArucoServo`)가 시작할 때 "마지막으로 marker 를 본 때" 를 지금으로 두어서, 아직 한 번도 못 봤는데 "방금 봤다" 로 판정했다 | 그 노드에서 marker 를 실제로 받아들인 뒤에만 설비 쪽으로 간다. 놓침 판정(2 s)은 시작한 때부터 센다. 고친 뒤 가려진 6 s 동안 로봇이 0.1 mm 도 움직이지 않았다 |

## 17. 파일

```
bt_navigator/
├── DOCKING.md                                    이 문서
├── DOCKING_BIG_PICTURE.md                        그림으로 보는 큰 그림과 "왜" (코드 지도, 좌표계, odom, PnP·IPPE, 칼만)
├── DOCKING_STUDY.md                              공부 노트: 기초 개념부터 판단 로직까지, 숫자 예제
├── MECANUM_STUDY.md                              공부 노트: 메카넘 바퀴의 구조, 힘, 운동학, 실제의 한계
├── docs/aruco_marker_dock_a.svg, .png            인쇄용 marker 하나 (SVG 를 100 % 로)
├── docs/aruco_marker_dock_b.svg, .png            인쇄용 marker 둘, A4 가로 한 장
├── docs/aruco_principle.png, docking_overview.png    원리와 구조 그림 (2장, 1-3)
├── docs/scenario_map.png                        시나리오의 경로와 장애물 자리 (16장)
├── docs/study_filter.png                         공부 노트 3-6 의 그림 (필터가 다가가며 좋아지는 모습)
├── docs/mecanum_wheel.png, mecanum_motions.png, mecanum_limits.png    메카넘 공부 노트의 그림
├── docs/bp_*.png                                 DOCKING_BIG_PICTURE.md 의 그림 12장
├── docs/aruco_results.png, aruco_camera_debug.jpg, aruco_camera_debug_two.jpg    결과, 붙은 자리의 카메라 영상
├── bt_mission_msgs/msg/ArucoDetection.msg        marker 하나: 후보(DockHypothesis), 화면 끝 여유, 진단
└── bt_mission/
    ├── config/docking.yaml                       ★ 로봇, 카메라, 설비마다 고치는 값 전부
    ├── behavior_trees/task.xml                   순서만: Dock, Undock, UndockPrecise 트리
    ├── include/bt_mission/docking_config.hpp     docking.yaml 읽기 (ROS 없음)
    ├── include/bt_mission/aruco_dock_control.hpp 계산 (ROS 없음): 필터, 후보 고르기, 깔때기, mecanumStep
    ├── include/bt_mission/aruco_dock_nodes.hpp   DetectArucoDock, ArucoServo, ArucoRefine
    ├── include/bt_mission/dock_nodes.hpp         BackOff (단순 undock), MuteCollisionMonitor, 설정 읽기
    ├── include/bt_mission/dock_control.hpp       계산 (ROS 없음): 단순 undock, 레이저 점으로 뒤와 주변 확인
    ├── src/aruco_dock_nodes.cpp, dock_nodes.cpp  위 노드들의 구현
    ├── src/task_server_main.cpp, scripts/task_client.py   작업 서버와 클라이언트 (README.md)
    ├── scripts/aruco_detector.py                 검출, IPPE, 바닥 사전분포 MAP, 깊이 평면, 공분산, 진단
    ├── scripts/aruco_teach.py                    가르친 자세 남기기
    ├── scripts/dock_calibrate.py                 배율, 데드밴드 재기
    ├── launch/aruco_detector.launch.py
    ├── tools/make_aruco_dock.py                  인쇄용 SVG/PNG, Gazebo 설비 모형
    ├── scenarios/                                시험 시나리오 (task_client.py --file), s8_repeat.sh (16장)
    ├── tools/study/                              공부 노트의 숫자를 다시 계산하는 스크립트 (메카넘 ex_mecanum.py, IPPE ex_ippe.py, 필터 비교 ex_why_filter.cpp)
    └── test/                                     test_docking_config, test_aruco_dock_control, test_dock_control (gtest),
                                                  test_aruco_detector.py (합성 영상, Gazebo 영상)

simple_line/simple_line_bringup/
├── robots/make_mecanum.py, mecanum.urdf, mecanum.model   4바퀴 메카넘 로봇
├── models/aruco_dock/, aruco_dock_two/           설비 모형: marker 하나 (dock_a), 둘 (dock_b, dock_markers:=2)
├── scripts/mecanum_slip_sim.py                   메카넘 미끄러짐과 바퀴 odom 흉내 (mecanum_slip:=True)
├── scripts/dock_truth.py                         Gazebo 참값으로 붙은 자리 판정, 로봇 옮겨 놓기 (16장)
└── launch/simple_line.launch.py                  robot:=mecanum, dock:=True, dock_markers, camera_resolution, camera_depth, mecanum_slip, camera_tf_error
```

## 참고 자료

- Nav2 docking 서버 (opennav_docking): staging 자세 → 외부 검출 자세 → 필터 → 제어의 구조. [index.ros.org/r/opennav_docking](https://index.ros.org/r/opennav_docking/), [docs.nav2.org: Docking Server](https://docs.nav2.org/configuration/packages/configuring-docking-server.html)
- RealSense D455 의 사양 (화각, 해상도, 깊이의 최소 거리): [intelrealsense.com: Depth Camera D455](https://intelrealsense.com/depth-camera-d455/), [store.intelrealsense.com](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d455.html). 컬러 화각을 86° x 57° (±3°) 로 적은 곳도 있다: [mouser.co.uk](https://www.mouser.co.uk/new/intel/intel-realsense-depth-camera-d455)
- ArUco marker: S. Garrido-Jurado 외, "Automatic generation and detection of highly reliable fiducial markers under occlusion", Pattern Recognition 2014. [doi:10.1016/j.patcog.2014.01.005](https://doi.org/10.1016/j.patcog.2014.01.005)
- OpenCV ArUco 검출: [docs.opencv.org: Detection of ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- OpenCV PnP: [docs.opencv.org: Perspective-n-Point (PnP) pose computation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)
- IPPE: T. Collins, A. Bartoli, "Infinitesimal Plane-Based Pose Estimation", IJCV 2014. [doi:10.1007/s11263-014-0725-5](https://doi.org/10.1007/s11263-014-0725-5)
- marker 자세의 뒤집힘(ambiguity): [arXiv:1909.11888](https://arxiv.org/abs/1909.11888)
- marker 하나로 로봇 자세 추정: [arXiv:2104.12954](https://arxiv.org/abs/2104.12954)
- 평면 운동 제약을 둔 PnP: [arXiv:2312.08488](https://arxiv.org/abs/2312.08488)
- 도킹의 hybrid 시각 서보: [arXiv:2509.07413](https://arxiv.org/abs/2509.07413)
