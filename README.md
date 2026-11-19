# nav2_dwb

Nav2 개념을 단계별로 쌓아 가며 실습하는 저장소.

- `dwb_steps.py`: DWA 를 4단계로 나눠 쌓아 가며 비교하는 Python 스크립트
- [`simple_line/`](simple_line/README.md): 직선 경로 planner + controller 최종 구현과 실행 설정 (ROS 2 Humble, C++)
  - [`simple_line/MULTI_GOAL.md`](simple_line/MULTI_GOAL.md): 명령 하나로 여러 goal 경유하기 (`NavigateThroughPoses`, `waypoint_follower`)
  - [`simple_line/AMR.md`](simple_line/AMR.md): 실제 AMR 로 옮기기 (지도 만들기와 AMCL, 부드러운 가감속, 장애물 감속, 실제 로봇 개발 방식)
- [`bt_navigator/`](bt_navigator/README.md): Nav2 명령을 상위 작업 BT 로 조합한다. 작업(들를 점 + 할 일)을 액션으로 받아 점마다 멈추며 실행한다. 장애물이 있으면 멈췄다가 치우면 다시 간다 (Nav2 collision_monitor). 아래층은 `simple_line` 을 쓴다

이 저장소의 로봇은 TurtleBot3 waffle (차동 구동) 이다. 4바퀴 메카넘 로봇 **pmbot** 을 위해 시작한 부분(설비 앞 ArUco marker 정밀 도킹, 메카넘 로봇 모델, RealSense D455, 관련 문서와 도구)은 별도 저장소 [pm_project](https://github.com/1-roro/pm_project) 로 옮겼다. pm_project 는 이 저장소의 `simple_line` 과 `bt_navigator` 를 복사해 이름을 바꾼 패키지로 혼자 빌드된다.
