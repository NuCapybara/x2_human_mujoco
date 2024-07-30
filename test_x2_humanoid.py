import mujoco
import numpy as np
import os
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import csv

# Path to the XML file
xml_path = "body_humanoid.xml"

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Input parameters. The desired starting point and ending point(in angle)
q0_init = 0  # Initial joint angle of human knee joint
q0_end = -np.pi / 2  # desired end joint angle of human knee joint

# Time duration for the motion
t_init = 0
t_end = 10

t = []
qact0 = []
qref0 = []
qact_exo_lknee_inertia = []
qact_exo_lknee = []
human_knee_torque = []
knee_joint_smooth_force = []
knee_joint_bias_force = []

# Generate the trajectory
def generate_trajectory(t0, tf, q0, qf):
    time_diff3 = (tf - t0) ** 3
    a0 = qf * (t0**2) * (3 * tf - t0) + q0 * (tf**2) * (tf - 3 * t0)
    a0 = a0 / time_diff3
    a1 = 6 * t0 * tf * (q0 - qf)
    a1 = a1 / time_diff3
    a2 = 3 * (tf + t0) * (qf - q0)
    a2 = a2 / time_diff3
    a3 = 2 * (q0 - qf)
    a3 = a3 / time_diff3
    return a0, a1, a2, a3

def init_controller(model, data):
    global a_jnt0
    a_jnt0 = generate_trajectory(t_init, t_end, q0_init, q0_end)

def controller(model, data):
    global a_jnt0

    time = data.time
    if time > t_end:
        time = t_end
    if time < t_init:
        time = t_init

    q0_ref = (
        a_jnt0[0] + a_jnt0[1] * time + a_jnt0[2] * (time**2) + a_jnt0[3] * (time**3)
    )
    q0dot_ref = a_jnt0[1] + 2 * a_jnt0[2] * time + 3 * a_jnt0[3] * (time**2)

    kp = 5
    kd = 0.1
    data.ctrl[14] = kp * (q0_ref - data.qpos[28]) + kd * (q0dot_ref - data.qvel[28])

    t.append(data.time)
    qact0.append(data.qpos[28])
    qact_exo_lknee_inertia.append(data.qpos[4])
    qact_exo_lknee.append(data.qpos[3])
    qref0.append(q0_ref)
    human_knee_torque.append(data.ctrl[14])
    knee_joint_smooth_force.append(data.qfrc_smooth[3])
    knee_joint_bias_force.append(data.qfrc_bias[3])

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    button_middle = (
        glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
    )
    button_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if not button_left and not button_middle and not button_right:
        return

    width, height = glfw.get_window_size(window)
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT

    if button_right:
        action = (
            mujoco.mjtMouse.mjMOUSE_MOVE_H
            if mod_shift
            else mujoco.mjtMouse.mjMOUSE_MOVE_V
        )
    elif button_left:
        action = (
            mujoco.mjtMouse.mjMOUSE_ROTATE_H
            if mod_shift
            else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        )
    else:
        action = mujoco.mjtMouse.mjMOUSE_ZOOM
    mujoco.mjv_moveCamera(model, action, dx / height, dy / height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mujoco.mjtMouse.mjMOUSE_ZOOM
    mujoco.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

def get_sensor_sensordata():
    return mujoco.MjData(model).sensordata

# get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mujoco.MjModel.from_xml_path(xml_path)  # MuJoCo model
print("THE NQ AND NV is", model.nq, model.nv)
data = mujoco.MjData(model)  # MuJoCo data
cam = mujoco.MjvCamera()  # Abstract camera
opt = mujoco.MjvOption()

for i in range(model.nu):  # model.nu is the number of actuators
    actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"Index {i} corresponds to actuator '{actuator_name}'")

for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"Index {i} corresponds to joint '{joint_name}'")

left_knee_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee_left")
exo_left_knee_joint_inertia_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "left_knee_joint_drive"
)
exo_left_knee_joint_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "left_knee_joint"
)
print("The left knee joint id is: ", left_knee_joint_id)
print("The left knee joint inertia id is: ", exo_left_knee_joint_inertia_id)
print("The left knee joint id is: ", exo_left_knee_joint_id)

print(model.jnt_qposadr[left_knee_joint_id], "the human knee left joint qposadr! 28")
print(
    model.jnt_dofadr[3], "the EXO knee left joint dofadr! For the frc_smooth[] using! 3"
)
print(
    model.jnt_qposadr[exo_left_knee_joint_inertia_id],
    "the EXO knee left joint inertia qposadr! 4",
)
print(model.jnt_qposadr[exo_left_knee_joint_id], "the EXO knee left joint qposadr! 3")

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mujoco.mjv_defaultCamera(cam)
mujoco.mjv_defaultOption(opt)
scene = mujoco.MjvScene(model, maxgeom=10000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

