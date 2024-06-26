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


