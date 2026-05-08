#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

sudo apt-get update
sudo apt-get install -y \
  curl \
  gnupg \
  lsb-release \
  locales \
  software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo add-apt-repository universe -y

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
  arch="$(dpkg --print-architecture)"
  echo "deb [arch=${arch} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ubuntu_codename} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  v4l-utils \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-tf2-geometry-msgs"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi

rosdep update
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro "${ROS_DISTRO}" \
  --skip-keys "ament_python"

sudo mkdir -p /opt/jetauto_orin_amr/models/{yolo,rf_detr,baseline}
sudo chown -R "${USER}:${USER}" /opt/jetauto_orin_amr
