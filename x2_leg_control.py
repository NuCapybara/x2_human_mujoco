import mujoco
import numpy as np
import os
import mediapy as media
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import csv

# Path to the XML file
xml_path = "simplify_leg_exo.xml"

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Input parameters. The desired starting point and ending point(in angle)
q0_init = 0 # Initial joint angle of human knee joint
q0_end = - np.pi /2   # desired end joint angle of human knee joint

# Time duration for the motion
t_init = 0
t_end = 4

t = []
qact0 = []
qref0 = []
human_knee_torque = []
knee_joint_passive_force = []

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

    kp = 150
    kd = 3

    data.ctrl[2] = kp * (q0_ref - data.qpos[9]) + kd * (q0dot_ref - data.qvel[9])

    # Append to lists
    t.append(data.time)
    qact0.append(data.qpos[9])
    qref0.append(q0_ref)
    human_knee_torque.append(data.ctrl[2])
    knee_joint_passive_force.append(data.qfrc_smooth[3])


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    button_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
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
        action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
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
data = mujoco.MjData(model)  # MuJoCo data
cam = mujoco.MjvCamera()  # Abstract camera
opt = mujoco.MjvOption()

for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"Index {i} corresponds to joint: {joint_name}")

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

# Set camera configuration
cam.azimuth = 90
cam.elevation = 5
cam.distance = 6
cam.lookat = np.array([0.0, 0.0, 0.0])

simend = t_end  # simulation time

# Initialize the controller
data.qpos[9] = q0_init

init_controller(model, data)
mujoco.set_mjcb_control(controller)

initial_positions = {
    "human_knee_top": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "human_knee_top"),
    "feet_pad_center": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "feet_pad_center")
}

initial_coords = {
    "human_knee_top": data.site_xpos[initial_positions["human_knee_top"]],
    "feet_pad_center": data.site_xpos[initial_positions["feet_pad_center"]]
}

print("Initial coords: ", initial_coords)

# Simulation parameters
duration = 5  # (seconds)
framerate = 60  # (Hz)
with open("sensor_data.csv", mode="a") as file:
    writer = csv.writer(file)
    while not glfw.window_should_close(window):
        time_prev = data.time

        while data.time - time_prev < 1.0 / 60.0:
            mujoco.mj_step(model, data)
            
            # knee_joint_passive_force_ = data.qfrc_smooth[3]
            # # print(knee_joint_passive_force_)
            # #i found the qfrc_smooth[3] is within [0, 5] as qfrc_passive[3]

            # # knee_joint_passive_force_ = data.qfrc_constraint[3]
            # # print(data.qfrc_constraint[3])
            # # print(data.qfrc_smooth[3] - data.qfrc_passive[3])
            # knee_joint_passive_force.append(knee_joint_passive_force_)
            writer.writerow(data.sensordata)

        # print(knee_joint_passive_force)
        if data.time >= simend:
            min_length = min(len(t), len(knee_joint_passive_force), len(human_knee_torque))
            t = t[:min_length]
            knee_joint_passive_force = knee_joint_passive_force[:min_length]
            human_knee_torque = human_knee_torque[:min_length]

            plt.figure(1)
            plt.subplot(2, 1, 1)
            plt.plot(t, np.subtract(qref0[:min_length], qact0[:min_length]), "k")
            plt.plot(t, qref0[:min_length], "r")
            plt.plot(t, qact0[:min_length], "b")
            plt.legend(["error", "qref_knee", "qact_knee"])
            plt.ylabel("error position joint 0")

            plt.subplot(2, 1, 2)
            plt.plot(t, knee_joint_passive_force, 'g-')
            plt.plot(t, human_knee_torque, 'r')
            plt.legend(["passive force", "active torque from human knee"])
            plt.ylabel("force (N)")

            plt.show(block=True)
            break
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

        mujoco.mjv_updateScene(
            model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scene
        )
        mujoco.mjr_render(viewport, scene, context)

        glfw.swap_buffers(window)
        glfw.poll_events()
# # Specify the file name
# file_name = "knee_joint_passive_force.csv"

# # Writing to csv file
# with open(file_name, mode='w', newline='') as file:
#     writer = csv.writer(file)
#     # Write the header
#     writer.writerow(["Knee_Joint_Passive_Force"])
#     # Write the data
#     for item in knee_joint_passive_force:
#         writer.writerow([item])

# print(f"Data has been written to {file_name}")

glfw.terminate()
