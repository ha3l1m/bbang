#!/usr/bin/env bash
# 시뮬레이션 환경 띄우기: 실제 E0509 + RH-P12-RN-A (에뮬레이터) + RViz(환경 마커 포함). 이 PC 로컬 전용(도메인 7).
#   ./sim_up.sh         → 다른 터미널에서  python3 run_sim.py --step pour_batter
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash; source "${WS:-$HOME/boong/ros2_ws}/install/setup.bash"
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
ros2 launch rhp12_gripper dsr_rviz_gripper.launch.py mode:=virtual gui:=false &
sleep 20
exec rviz2 -d "$HERE/../rviz_demo/boong_real.rviz" --ros-args -r __ns:=/dsr01 -r __node:=rviz2
