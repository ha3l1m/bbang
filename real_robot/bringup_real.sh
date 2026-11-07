#!/usr/bin/env bash
# 실제 로봇 연결 + RViz. (터미널 1) 서보 ON/원격 제어 허용은 티치펜던트에서 먼저.
#   ROBOT_IP=110.120.1.64 ./bringup_real.sh
ROBOT_IP="${ROBOT_IP:-110.120.1.64}"
PORT="${PORT:-12345}"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo "→ 컨트롤러 $ROBOT_IP:$PORT 연결 확인"
ping -c 2 -W 2 "$ROBOT_IP" >/dev/null || { echo "ping 실패: 랜선/노트북 IP(110.120.1.x, /24)/티치펜던트 네트워크 확인"; exit 1; }
exec ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:="$ROBOT_IP" port:="$PORT" model:=e0509
