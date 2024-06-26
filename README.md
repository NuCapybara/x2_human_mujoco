# Mujoco Simulation for Human Exoskeleton Interaction
* Jialu Yu
* Summer 2024
* Shirley Ryan AbilityLab &  Northwestern University


# Package Description

The repo is mainly designed for controlling the exoskeleton and the interaction between humanbody and exoskeleton. Currently I only implemented the PD control for left knee joint and left hip joint. It's still under progress.

# File description
- x2.xml: Converted from x2_fixed_base.urdf.xacro. lower-limb exoskeleton xml file, using the stl files in this repo for the meshes. 2 actuators on the hip and knees were added to the xml files, more changes under progress.
- x2_manipulator.ipynb: Applying force torque to knee/hip joints to control.
- x2_pd_control.py: User inputs starting joint angles,  ending joint angles and time for left hip and knee joint. and using PD control to output the torque on each actuators to achieve the desires joint angles.
- pend.xml: a pendulum composed of two bar and one joint for a human leg reference. [Not used]


# Mujoco starter reminder
This repository is Non-ROS, files are only used for Mujoco with exoskeleton control. Here is a kind reminder for Mujoco beginners:
- Mujoco does not accept xacro.urdf file, please use ROS to help convert. Here is an example terminal command.

    `xacro.urdf to urdf. xacro x2_fixed_base.urdf.xacro > x2_fixed_base.urdf`
    
- You can drag your urdf file to Mujoco and save them to xml file, which is Mujoco's default file style. (It's easier to change features on xml file.)

    - Mujoco accept stl file as meshes, please convert all your .dae file into stl to successfully load the model.
    - Stl file must not exceeding 200000 faces, please use Mesh software(e.g. MeshLab) to simplify the model and reduce the faces.



## Demo Videos# ME495 Sensing, Navigation and Machine Learning For Robotics
* Jialu Yu
* Winter 2022
# Package List
This repository has one helper Libraries(Non-ROS)
- turtlelib:  A library for handling transformations in SE(2) and other turtlebot/lidar-related math.

    - diff_drive - related to the Kinematics of wheeled mobile robots, and is part of the turtlelib package
    - frame_main - generate a user-specified vector in visualization with a svg file
    - geometry2d - utilities for two-dimensional geometric operations, such as comparing floating-point numbers, converting between degrees and radians, normalizing angles, and manipulating 2D points and vectors.
    - ekf_slam - perform Extended Kalman Filter algorithm 
    - lidar - perform the line to circle intersection algorithm
    - circle_fitting - perform circle fitting algorithm

This repository consists of several ROS packages 
- nuturtle_description - Visualize turtlebots with user-specified parameters
- nusim - Simulates a Turtlebot3 in an rviz environment, mimicing  the real turtlebot
- nuturtle_control - provides an odometry estimate for the turtlebot that can interface with either a simulated or real robot.
- nuslam - Performs EKF slam to generate a map of the environment.




## Demo Videos
### EKF Slam Simulation video (HW3)
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/a1ad52a8-5b35-4b64-8e63-6002c8d7f1ff" controls title="EKF Simulation"></video>

### Turtlebot simulation Video
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/08c6739b-bc16-438b-948c-263814212cb3" controls title="Simulation"></video>

### Turtlebot Performance
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/7f7e29e0-7a01-46e1-9e1d-21a7796f2f04" controls title="Real scene"></video>

### EKF Slam Simulation video (HW3)
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/a1ad52a8-5b35-4b64-8e63-6002c8d7f1ff" controls title="EKF Simulation"></video>

### Turtlebot simulation Video
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/08c6739b-bc16-438b-948c-263814212cb3" controls title="Simulation"></video>

### Turtlebot Performance
<video src="https://github.com/ME495-Navigation/slam-project-NuCapybara/assets/144244355/7f7e29e0-7a01-46e1-9e1d-21a7796f2f04" controls title="Real scene"></video>
