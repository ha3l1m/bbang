#!/usr/bin/env bash
# ROS 2 + rviz2 설치. Ubuntu 24.04 → Jazzy, 22.04 → Humble 을 자동 선택. 이미 있으면 건너뜀. 한 번만 실행.
set -e
. /etc/os-release
case "$VERSION_ID" in 24.04) D=jazzy;; 22.04) D=humble;; *) echo "지원: Ubuntu 24.04(jazzy) / 22.04(humble) — 현재 $VERSION_ID"; exit 1;; esac
if [ ! -f /opt/ros/$D/setup.bash ]; then
  sudo apt-get update && sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-$D-desktop
fi
sudo apt-get install -y ros-$D-tf2-ros python3-yaml python3-pytest
echo "OK ($D). 이제 ./run_rviz_demo.sh"
