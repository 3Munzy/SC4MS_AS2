# UR3 Eye-in-Hand Visual Servoing  
### D435i + ArUco + RMRC + ROS 2 Gazebo Integration  

This project enables a UR3 robot to visually track an ArUco marker using an Intel RealSense D435i camera mounted on its end-effector.  
The system combines Image-Based Visual Servoing (IBVS) for real-world operation and Position-Based Visual Servoing (PBVS) for simulation, alongside Resolved Motion Rate Control (RMRC), enabling the robot to continuously align its gripper to the marker in real time — either in simulation (Gazebo) or on the real UR3 via ROS 2 Humble.

---

## Project Overview

The camera captures color and depth data, detects the marker, computes the spatial error, and commands the robot to move to reduce that error.  
The control loop runs at approximately 30 Hz and can operate either in simulation or on a real robot.

---

## Code Modules

**camera_core.py**  
Handles Intel RealSense D435i input. Streams RGB + depth frames and builds the intrinsic matrix (K).  

**target_aruco.py**  
Detects ArUco markers using OpenCV and computes the marker pose (u, v, Z, yaw).  

**ibvs.py**  
Implements the IBVS control law — converts image-space error into velocity commands in the camera frame.  

**handeye.py**  
Stores the camera-to-end-effector calibration matrix (hand–eye transform).  

**ur3_rmrc_node.py**  
Computes joint velocities from end-effector velocities using the robot’s Jacobian (RMRC).  

**ibvs_pub.py**  
The main runtime node that connects all components — camera, marker detection, IBVS, hand–eye transform, and RMRC control.  

**ibvs_dryrun.py**  
Runs the visual servoing logic without commanding the robot, for debugging.  

**aruco_live_demo.py**  
Standalone ArUco detection demo for calibration and testing.  

**publish_fake_aruco_marker.py**  
Publishes a simulated marker (visualization_msgs/Marker) for use in RViz or Gazebo.  

**publish_marker_servo_twist.py**  
Generates PBVS/IBVS velocity commands (geometry_msgs/TwistStamped) based on marker pose.  

**ibvs_to_ur3_vel.py**  
Implements the control strategy used during Gazebo simulation.  
This node subscribes to visual servoing twist commands (`/ibvs/camera_twist`), computes resolved joint velocities through the Jacobian, and publishes velocity vectors directly to the UR3 controllers (`/joint_group_velocity_controller/command`).  
In simulation mode, this serves as the main bridge between the IBVS algorithm and the robot’s motion — converting visual servoing output into physical actuation.  

**ur3_gazebo_launch.py / ur3_simulation_launch.py**  
Example launch files for Gazebo integration.  

**ur3_controllers.yaml**  
Defines controller configurations (trajectory and velocity controllers) for ROS 2.  

---

## Control Flow

1. The D435i captures RGB and depth frames (`camera_core.py`) and computes the camera matrix (K).  
2. The marker is detected (`target_aruco.py`) to obtain pixel coordinates, depth, and orientation.  
3. IBVS computes camera velocity to minimize image-space error (`ibvs.py`).  
4. Hand–eye calibration transforms the camera velocity into the robot base frame (`handeye.py`).  
5. RMRC converts end-effector velocity into joint velocities (`ur3_rmrc_node.py`).  
6. The main node (`ibvs_pub.py`) runs this loop continuously at 30 Hz.  

---

## Core Ideas

- **IBVS (Image-Based Visual Servoing):** Control derived directly from image features rather than a full 3D model.  
- **RMRC (Resolved Motion Rate Control):** Converts end-effector velocities into joint space using the Jacobian.  
- **Hand–Eye Calibration:** Determines the exact spatial relationship between the camera and robot tool frame.  

---

## Environment Setup (using setup_full_env.sh)

Instead of manually installing dependencies with requirements.txt, this project now includes an automated setup script (`setup_full_env.sh`) that installs all Python and ROS 2 dependencies in one step.

### To install:

```bash
bash setup_full_env.sh
```

The script will:
1. Create and activate a Python virtual environment (`~/rosenv`).  
2. Install all required Python libraries (NumPy, OpenCV, RealSense, etc.).  
3. Install ROS 2 Humble, Gazebo, and the UR3 control stack.  
4. Initialize rosdep and colcon for workspace builds.  

After completion, you will have a fully configured environment for both simulation and real hardware.  

---

## Simulation and ROS Integration

1. **Source your workspace**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/your_ws/install/setup.bash
   ```

2. **Launch the UR3 simulation (Gazebo + RViz)**
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py        ur_type:=ur3e robot_ip:=fake        __ns:=/ur robot_controller:=joint_trajectory_controller        launch_rviz:=true
   ```

3. **Enable the velocity controller**
   ```bash
   ros2 control switch_controllers        --activate joint_group_velocity_controller        --deactivate scaled_joint_trajectory_controller
   ```

4. **Start the fake marker publisher**
   ```bash
   ros2 run your_package publish_fake_aruco_marker.py
   ```

5. **Run the PBVS/IBVS node**
   ```bash
   ros2 run your_package publish_marker_servo_twist.py
   ```

6. **Run the IBVS → UR3 Velocity Bridge**
   ```bash
   ros2 run your_package ibvs_to_ur3_vel.py
   ```
   This node subscribes to `/ibvs/camera_twist` and converts the published visual servoing velocities into resolved joint velocities for the UR3.  
   It then sends these vectors to the active velocity controller (`/joint_group_velocity_controller/command`) in the Gazebo simulation.  
   Essentially, it forms the **bridge between vision-based control and robot motion**, allowing the simulated robot to respond to camera-based servo commands in real time.

7. **Visualize in RViz**
   - Add `/visualization_marker` to view the marker.
   - Verify TF connections between `base_link`, `tool0`, and the marker.  

---

## Typical Testing Workflow

1. Run `aruco_live_demo.py` to confirm camera and marker detection.  
2. Run `ibvs_dryrun.py` to verify control law without physical motion.  
3. Run `ibvs_pub.py` to execute full closed-loop visual servoing.  

---

## Troubleshooting

- **Robot not moving:** Ensure the velocity controller is activated.  
- **Marker not detected:** Check TAG_ID and dictionary in `target_aruco.py`.  
- **Orientation errors:** Update `R_offset` in `publish_marker_servo_twist.py`.  
- **Import errors:** Re-run `bash setup_full_env.sh`.  
- **TF frame issues:** Verify that `/tf` includes `base_link`, `tool0`, and marker frames.  

---

## Developer Notes

- Update the hand–eye transform in `handeye.py` with your calibration data.  
- Tune control gains in `ibvs.py` for smoother response.  
- Replace URDF references in `ur3_rmrc_node.py` with more detailed models if needed.  
- For debugging, run Gazebo in paused mode to step through motion frames.  

---

## Summary

This repository provides a complete eye-in-hand visual servoing pipeline. The camera detects the ArUco marker, the control algorithm computes motion commands, and the UR3 executes precise movements to maintain alignment — all in real time.
