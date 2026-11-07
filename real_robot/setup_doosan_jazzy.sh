#!/usr/bin/env bash
# Ubuntu 24.04 + ROS 2 Jazzy 에 doosan-robot2(jazzy 브랜치) 설치/빌드. 처음 한 번.
# 컨트롤러가 DRCF 2.x 이므로 -DDRCF_VER=3 은 쓰지 않는다 (기본 빌드가 2.x).
set -e
[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy 먼저: ../rviz_demo/setup_ubuntu.sh"; exit 1; }
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y git libpoco-dev libyaml-cpp-dev wget \
  ros-jazzy-control-msgs ros-jazzy-realtime-tools ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-gazebo-msgs ros-jazzy-moveit-msgs \
  dbus-x11 ros-jazzy-moveit-configs-utils ros-jazzy-moveit-ros-move-group \
  python3-colcon-common-extensions python3-rosdep
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
[ -d doosan-robot2 ] || git clone -b jazzy https://github.com/doosan-robotics/doosan-robot2.git
cd ~/ros2_ws
sudo rosdep init 2>/dev/null || true; rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
rm -rf build install log
colcon build            # DRCF 2.x (기본)
echo
echo "빌드 끝. 아래 두 줄을 ~/.bashrc 에 추가하세요:"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  export PYTHONPATH=\$PYTHONPATH:\$HOME/ros2_ws/install/dsr_common2/lib/dsr_common2/imp"
