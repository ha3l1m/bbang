#!/usr/bin/env bash
# 우분투(ROS 2 Jazzy 24.04 / Humble 22.04)에서 RViz 로 붕봇 동작 확인.
#   ./run_rviz_demo.sh                       단팥 1개
#   ./run_rviz_demo.sh --flavor cream --count 2 --speed 5
#   ./run_rviz_demo.sh --step flip_mold      동작 하나만
#   ./run_rviz_demo.sh --loop                계속 반복
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export BOONG_REPO="${BOONG_REPO:-$(cd "$HERE/.." && pwd)}"
[ -f "$BOONG_REPO/ros2_ws/src/boong_robot/config/poses.yaml" ] || { echo "poses.yaml 을 못 찾음: rviz_demo 폴더를 레포 맨 위(boong/)에 두거나 BOONG_REPO 를 지정하세요"; exit 1; }
# shellcheck disable=SC1091
for d in jazzy humble; do [ -f "/opt/ros/$d/setup.bash" ] && { source "/opt/ros/$d/setup.bash"; break; }; done
[ -n "$ROS_DISTRO" ] || { echo "ROS 2 (Jazzy/Humble) 가 없어요 → ./setup_ubuntu.sh 먼저"; exit 1; }
echo "ROS_DISTRO=$ROS_DISTRO"

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id base_link >/dev/null 2>&1 &
pids+=($!)
rviz2 -d "$HERE/boong.rviz" >/dev/null 2>&1 &
pids+=($!)
python3 "$HERE/viz_demo.py" "$@"
