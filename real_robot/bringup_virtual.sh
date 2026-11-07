#!/usr/bin/env bash
# 로봇 없이 '두산 에뮬레이터 + 실제 E0509 모델' 로 같은 코드(run_real.py)를 먼저 돌려 보기. Docker 필요.
#   처음 한 번: cd ~/ros2_ws/src/doosan-robot2 && sudo ./install_emulator.sh
# (터미널 1) ./bringup_virtual.sh   → RViz 에 진짜 E0509 가 뜸
# (터미널 2) python3 run_real.py --step pour_batter     ← real 과 똑같은 명령, 로봇 대신 에뮬레이터가 받음
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:="${PORT:-12345}" model:=e0509
