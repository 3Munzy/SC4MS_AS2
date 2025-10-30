#!/bin/bash
set -e
sudo apt update
sudo apt install -y python3-venv python3-pip curl

# --- Python venv ---
python3 -m venv ~/rosenv
source ~/rosenv/bin/activate
pip install --upgrade pip

# Create requirements file inline
cat <<EOF > requirements.txt
numpy==1.26.4
scipy>=1.10,<1.12
opencv-contrib-python==4.10.0.84
spatialmath-python>=1.1,<1.2
roboticstoolbox-python>=1.1,<1.2
roslibpy>=1.5
pyrealsense2>=2.54
websockets==10.4
EOF

pip install -r requirements.txt

# --- ROS base + Gazebo + UR3 ---
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-sim-demos \
  ros-humble-ur \
  ros-humble-ur-description \
  ros-humble-ur-gazebo \
  ros-humble-ur-msgs \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-tf-transformations \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-sensor-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rviz2

# --- Build tools ---
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  python3-empy \
  build-essential

sudo rosdep init || true
rosdep update

echo "✅ Full environment ready: Python + ROS 2 Humble + Gazebo + UR3 + RealSense + IBVS stack"
