UR3 Eye-in-Hand Visual Servoing
D435i + ArUco + RMRC Project

This project was created to make the UR3 robot track an ArUco marker using a camera mounted on its end-effector. The camera (an Intel RealSense D435i) captures both color and depth information, and the robot uses that visual feedback to move automatically and keep the marker centered in view. The whole system uses Image-Based Visual Servoing (IBVS) and Resolved Motion Rate Control (RMRC) to convert what the camera sees into actual robot movement.

The setup can run either in simulation (using Swift) or on the real UR3 through ROS.

Below is a breakdown of how all the files work together and what each one does.

#camera_core.py
This file handles the D435i camera. It reads both the RGB image and the depth data, and it also builds the camera intrinsic matrix (K), which defines how the camera translates between pixels and real-world distances. This is what feeds the visual data into the rest of the system.

#target_aruco.py
This detects the ArUco marker in the camera feed using OpenCV. It calculates the marker’s pixel position (u, v), the distance from the camera (Z), and the rotation angle (yaw). These values are the input for the visual control stage.

#ibvs.py
This file defines how the robot should respond based on what the camera sees. It takes the error between where the marker is and where it should be (the center of the image) and converts that into a velocity command in the camera’s coordinate system. Basically, it tells the camera, “move left, right, up, down, closer, or further” to keep the tag centered.

#handeye.py
This contains the calibration result that defines the camera’s position relative to the robot’s end-effector. This relationship is needed because the camera is not the tool itself—it’s mounted on the tool. This file helps translate movements from the camera’s perspective into the robot’s coordinate frame.

#ur3_rmrc_node.py
This is where the robot control happens. It takes the desired motion (from the previous step) and calculates how each of the UR3’s joints should move to achieve that motion. It uses Resolved Motion Rate Control (RMRC), which relies on the robot’s Jacobian to convert the end-effector velocity into individual joint velocities.

#ibvs_pub.py
This is the main file that ties everything together. It runs the full loop: grabbing images, detecting the ArUco marker, running the IBVS control law, applying the hand–eye transformation, and then sending the joint velocity commands to the robot. It can run in simulation or on the real robot.

#ibvs_dryrun.py
This is a test version of ibvs_pub.py. It runs the visual servoing logic without actually moving the robot, so it’s useful for debugging the control and vision parts safely.

#aruco_live_demo.py
A standalone demo that only detects and displays the ArUco marker in real time, without controlling the robot. It’s good for testing camera calibration and marker tracking.

#requirements.txt
Lists the Python libraries you need to install (like OpenCV, Robotics Toolbox, SpatialMath, and roslibpy).

#Industrialenv_packages.txt
An export of the full environment used during development. This helps recreate the same setup if needed.

The control process follows a simple flow:

1. The camera (camera_core.py) captures a color and depth image and gives us the camera matrix K.
2. The marker detection (target_aruco.py) finds the position (u, v), depth (Z), and yaw of the marker.
3. The IBVS control law (ibvs.py) calculates how the camera should move to center the tag.
4. The hand–eye calibration (handeye.py) converts the camera’s motion commands into the robot’s coordinate system.
5. The RMRC node (ur3_rmrc_node.py) calculates the necessary joint velocities for the robot.
6. The main loop (ibvs_pub.py) runs all these steps continuously at about 30 frames per second.

To run in simulation using Swift:
python ibvs_pub.py

To run with the real robot through ROS:
USE_ROS=True python ibvs_pub.py

Before using the real robot, make sure that:

* rosbridge_server is running
* The RealSense camera is publishing color and depth data
* The UR3 is connected to ROS

Dependencies can be installed using:
pip install -r requirements.txt
or by recreating the Conda environment:
conda create --name Industrialenv --file Industrialenv_packages.txt

The main ideas behind this project are:
Image-Based Visual Servoing (IBVS): Using what the camera sees to directly control motion, without needing a full 3D model of the environment.
Resolved Motion Rate Control (RMRC): Converting a desired end-effector velocity into joint movements using the robot’s kinematics.
Hand–Eye Calibration: Knowing exactly where the camera is relative to the robot’s tool, so movements are accurate.

Typical workflow for testing:

1. Run aruco_live_demo.py to make sure marker detection works.
2. Run ibvs_dryrun.py to test that the control code works without hardware.
3. Run ibvs_pub.py to run the full system in simulation or on the robot.

Developer notes:
Make sure TAG_DICT, TAG_ID, and TAG_SIZE in target_aruco.py match the printed ArUco marker.
Update the transform in handeye.py with your calibrated hand–eye values.
Tune the control gains in ibvs.py (LAM_XY, LAM_Z, K_YAW, Z_DES) to get smooth and stable motion.
If you want to see a detailed UR3 model in the simulator, replace the default model in ur3_rmrc_node.py with a URDF that includes STL mesh files.

In simple terms, this project connects computer vision and robot control. The camera sees where the marker is, the code figures out how far off it is from the center, and the robot moves to correct that error. This happens continuously in real time, creating a complete closed-loop visual servoing system.
