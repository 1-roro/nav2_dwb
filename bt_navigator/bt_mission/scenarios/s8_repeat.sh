#!/bin/bash
# 시나리오 8: 실제 로봇과의 차이를 넣은 시뮬레이션에서 가르치고, 여러 staging 에서 되풀이해 정밀도를 잰다
# (bt_navigator/DOCKING.md 16-9). Gazebo 의 참값(simple_line_bringup 의 dock_truth.py)으로 판정한다.
#
#   ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True \
#       dock_markers:=both mecanum_slip:=True camera_tf_error:=1.5,0.0,0.01 aruco_detector:=False
#   ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True marker_scale:=1.02
#   ros2 run bt_mission task_server
#   bash $(ros2 pkg prefix bt_mission)/share/bt_mission/scenarios/s8_repeat.sh [dock] [횟수] [허용오차 mm,mm,deg]
#
# 순서: (1) staging 으로 가서 도면 값 목표로 한 번 붙고 참값을 잰다 (넣은 차이가 그대로 남는다)
#       (2) 로봇을 참값으로 정확한 붙은 자리에 옮겨 놓고 aruco_teach.py 로 가르친다
#       (3) staging 을 바꿔 가며 undock -> dock 을 되풀이하고 매번 참값을 잰다
# 끝에 합격 횟수를 낸다. 종료 코드 0 = 모두 허용오차 안
set -u
dock=${1:-dock_a}
count=${2:-5}
tol=${3:-2,2,0.3}
case $dock in
  dock_a) stagings=("3.85,1.0,0" "3.85,1.25,-15" "3.80,0.80,12") ; first="4.0,-2.5 3.85,1.0,0" ;;
  dock_b) stagings=("3.85,-2.0,0" "3.85,-1.80,-12" "3.80,-2.20,12") ; first="4.0,-2.5 3.85,-2.0,0" ;;
  *) echo "dock_a or dock_b"; exit 2 ;;
esac
taught=~/.ros/aruco_taught/$dock.yaml

echo "== 1. geometric target (no taught pose)"
if [ -f "$taught" ]; then mv "$taught" "$taught.bak"; echo "   (moved the old taught pose to $taught.bak)"; fi
ros2 run bt_mission task_client.py dock:$dock $first > /dev/null || { echo "dock failed"; exit 1; }
ros2 run simple_line_bringup dock_truth.py $dock --tol $tol

echo "== 2. teach at the true docked pose"
ros2 run simple_line_bringup dock_truth.py $dock --place
sleep 2
ros2 run bt_mission aruco_teach.py --ros-args -p dock:=$dock || exit 1

echo "== 3. repeat $count times on the taught target"
pass=0
for i in $(seq 1 $count); do
  staging=${stagings[$(( (i - 1) % ${#stagings[@]} ))]}
  ros2 run bt_mission task_client.py undock > /dev/null
  if ros2 run bt_mission task_client.py dock:$dock $staging > /dev/null; then
    line=$(ros2 run simple_line_bringup dock_truth.py $dock --tol $tol) && pass=$((pass + 1))
    echo "   $i from $staging: $line"
  else
    echo "   $i from $staging: dock FAILED"
  fi
done
echo "== $pass of $count within $tol (mm, mm, deg)"
[ $pass -eq $count ]
