import mujoco
import numpy as np
import os
import mediapy as media
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import csv
import math

# Path to the XML file
xml_path = "free_system.xml"

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Input parameters. The desired starting point and ending point(in angle [rad])

q0_init = 0  # Initial joint angle of human knee joint (left human knee)
q0_end = 0  # desired end joint angle of human knee joint (left human knee)
q1_init = 0  # Initial joint angle of human hip joint (left human hip)
q1_end = 0  # desired end joint angle of human hip joint (left human hip)

q2_init = 0  # Initial joint angle of human knee joint (right human knee)
q2_end = 0  # desired end joint angle of human knee joint (right human knee)
q3_init = 0  # Initial joint angle of human hip joint (right human hip)
q3_end = 0  # desired end joint angle of human hip joint (right human hip)


# Time duration for the motion
t_init = 0
t_end = 5
t = []
qact0 = []
qref0 = []
qact1 = []
qref1 = []
qact2 = []
qref2 = []
qact3 = []
qref3 = []


qact_exo_lknee_inertia = []
qact_exo_lknee = []
# human_knee_torque appends the control signal calculated to the human knee joint
human_knee_torque = []
knee_joint_smooth_force = []
knee_joint_bias_force = []
torque_calculated = []
knee_passive_force = []
exo_knee_act_force = []
exo_knee_contraint_force = []

grav = []
left_foot_sensor = []
right_foot_sensor = []
# contact_force = []

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
    global a_jnt0, a_jnt1, a_jnt2, a_jnt3
    a_jnt0 = generate_trajectory(t_init, t_end, q0_init, q0_end)
    a_jnt1 = generate_trajectory(t_init, t_end, q1_init, q1_end)
    a_jnt2 = generate_trajectory(t_init, t_end, q2_init, q2_end)
    a_jnt3 = generate_trajectory(t_init, t_end, q3_init, q3_end)


def controller(model, data):
    global a_jnt0, a_jnt1, a_jnt2, a_jnt3

    time = data.time
    if time > t_end:
        time = t_end
    if time < t_init:
        time = t_init

    q0_ref = (
        a_jnt0[0] + a_jnt0[1] * time + a_jnt0[2] * (time**2) + a_jnt0[3] * (time**3)
    )
    q0dot_ref = a_jnt0[1] + 2 * a_jnt0[2] * time + 3 * a_jnt0[3] * (time**2)

    q1_ref = (
        a_jnt1[0] + a_jnt1[1] * time + a_jnt1[2] * (time**2) + a_jnt1[3] * (time**3)
    )
    q1dot_ref = a_jnt1[1] + 2 * a_jnt1[2] * time + 3 * a_jnt1[3] * (time**2)

    q2_ref = (
        a_jnt2[0] + a_jnt2[1] * time + a_jnt2[2] * (time**2) + a_jnt2[3] * (time**3)
    )
    q2dot_ref = a_jnt2[1] + 2 * a_jnt2[2] * time + 3 * a_jnt2[3] * (time**2)

    q3_ref = (
        a_jnt3[0] + a_jnt3[1] * time + a_jnt3[2] * (time**2) + a_jnt3[3] * (time**3)
    )
    q3dot_ref = a_jnt3[1] + 2 * a_jnt3[2] * time + 3 * a_jnt3[3] * (time**2)

    kp = 8
    kd = 0.1
    # control_signal = kp * (q0_ref - data.qpos[22]) + kd * (q0dot_ref - data.qvel[22])
    # right knee joint
    # data.ctrl[10] = kp * (q0_ref - data.qpos[16]) + kd * (q0dot_ref - data.qvel[16])
    # left knee joint
    data.ctrl[16] = kp * (q0_ref - data.qpos[34]) + kd * (
        q0dot_ref - data.qvel[34]
    )  # left knee
    data.ctrl[15] = kp * (q1_ref - data.qpos[33]) + kd * (
        q1dot_ref - data.qvel[33]
    )  # left hip
    data.ctrl[10] = kp * (q2_ref - data.qpos[28]) + kd * (
        q2dot_ref - data.qvel[28]
    )  # right knee
    data.ctrl[9] = kp * (q3_ref - data.qpos[27]) + kd * (
        q3dot_ref - data.qvel[27]
    )  # right hip

    # Append to lists
    t.append(data.time)

    qact0.append(data.qpos[34])  # left knee
    qact1.append(data.qpos[33])  # left hip
    qact2.append(data.qpos[28])  # right knee
    qact3.append(data.qpos[27])  # right hip

    #sum up body mass
    mass = 0
    for i in range(model.nbody):
        mass += model.body_mass[i]

    grav.append(mass * 9.81) #change the index and see what happened 97.38*9.81 = 955.2978
    # contact_force.append(data.sensordata[0])

    left_foot_sensor.append(data.sensordata[0])
    right_foot_sensor.append(data.sensordata[1])



    qact_exo_lknee_inertia.append(data.qpos[4])
    bias_torque_calculated = (
        math.sin(math.pi - data.qpos[4]) * (1.6105) * 9.81 * 0.312012
    )
    torque_calculated.append(bias_torque_calculated)
    qact_exo_lknee.append(data.qpos[3])

    # print(data.qpos[22])
    qref0.append(q0_ref)
    qref1.append(q1_ref)
    qref2.append(q2_ref)
    qref3.append(q3_ref)
    human_knee_torque.append(data.ctrl[14])

    # qfrc_smooth index depends on nv, degree of freedom
    knee_joint_smooth_force.append(data.qfrc_smooth[3])
    knee_joint_bias_force.append(data.qfrc_bias[3])
    knee_passive_force.append(data.qfrc_passive[3])
    exo_knee_act_force.append(data.qfrc_actuator[3])
    exo_knee_contraint_force.append(data.qfrc_constraint[3])
    # print(
    #     "The knee joint bias force is: ",
    #     data.qfrc_bias[3],
    #     "the knee joint grav forces is",
    #     bias_torque_calculated,
    # )
    # knee_joint_passive_force.append(data.qfrc_bias[3])


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
left_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hip_y_left")
right_knee_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee_right")
right_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hip_y_right")
exo_left_knee_joint_inertia_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "left_knee_joint_drive"
)
exo_left_knee_joint_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "left_knee_joint"
)


print("The left knee joint id is: ", left_knee_joint_id)
print("The left hip joint id is: ", left_hip_joint_id)
print("The right knee joint id is: ", right_knee_joint_id)
print("The right hip joint id is: ", right_hip_joint_id)
print("The left knee joint inertia id is: ", exo_left_knee_joint_inertia_id)
print("The exo left knee joint id is: ", exo_left_knee_joint_id)
# We use jnt_qposadr to get the address of the joint position in qpos with the input of joint id
print(model.jnt_qposadr[left_knee_joint_id], "the human knee left joint qposadr!")
print(model.jnt_qposadr[left_hip_joint_id], "the human hip left joint qposadr!")
print(model.jnt_qposadr[right_knee_joint_id], "the human knee right joint qposadr!")
print(model.jnt_qposadr[right_hip_joint_id], "the human hip right joint qposadr!")
print(
    model.jnt_dofadr[3], "the EXO knee left joint dofadr! For the frc_smooth[] using! "
)
print(
    model.jnt_qposadr[exo_left_knee_joint_inertia_id],
    "the EXO knee left joint inertia qposadr!",
)
print(model.jnt_qposadr[exo_left_knee_joint_id], "the EXO knee left joint qposadr! ")
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
data.qpos[34] = q0_init
data.qpos[33] = q1_init
data.qpos[28] = q2_init
data.qpos[27] = q3_init

print("************** the init knee joint angle is: ", data.qpos[34], " and ", q0_init)
print("************** the init hip joint angle is: ", data.qpos[33], " and ", q1_init)
print("************** the init knee joint angle is: ", data.qpos[28], " and ", q2_init)
print("************** the init hip joint angle is: ", data.qpos[27], " and ", q3_init)

init_controller(model, data)
mujoco.set_mjcb_control(controller)

initial_positions = {
    "human_knee_top": mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, "human_knee_top"
    ),
    "feet_pad_center": mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, "feet_pad_center"
    ),
}

initial_coords = {
    "human_knee_top": data.site_xpos[initial_positions["human_knee_top"]],
    "feet_pad_center": data.site_xpos[initial_positions["feet_pad_center"]],
}

print("Initial coords: ", initial_coords)

# Simulation parameters

with open("sensor_data.csv", mode="a") as file:
    writer = csv.writer(file)
    while not glfw.window_should_close(window):
        time_prev = data.time

        while data.time - time_prev < 1.0 / 60.0:
            mujoco.mj_step(model, data)
            writer.writerow(data.sensordata)

        # print(knee_joint_passive_force)
        if data.time >= simend:
            min_length = min(
                len(t), len(knee_joint_smooth_force), len(human_knee_torque)
            )
            t = t[:min_length]
            knee_joint_smooth_force = knee_joint_smooth_force[:min_length]
            knee_joint_bias_force = knee_joint_bias_force[:min_length]
            human_knee_torque = human_knee_torque[:min_length]

            plt.figure(1)
            plt.subplot(5, 1, 1)
            plt.plot(t, np.subtract(qref0[:min_length], qact0[:min_length]), "k")
            plt.plot(t, qref0[:min_length], "r")
            plt.plot(t, qact0[:min_length], "b")
            plt.legend(["error", "qref_left_knee", "qact_left_knee"])
            plt.ylabel("position/angle (rad)")

            plt.subplot(5, 1, 2)
            plt.plot(t, np.subtract(qref1[:min_length], qact1[:min_length]), "k")
            plt.plot(t, qref1[:min_length], "r")
            plt.plot(t, qact1[:min_length], "b")

            plt.legend(["error", "qref_left_hip", "qact_left_hip"])
            plt.ylabel("position/angle (rad)")

            plt.subplot(5, 1, 3)
            # plt.plot(t, qact_exo_lknee, "g-")
            # plt.plot(t, qact_exo_lknee_inertia, "b-")
            plt.plot(t, np.subtract(qref2[:min_length], qact2[:min_length]), "k")
            plt.plot(t, qref2[:min_length], "r")
            plt.plot(t, qact2[:min_length], "b")

            plt.legend(
                [
                    "error",
                    "qref_right_knee",
                    "qact_right_knee",
                ]
            )
            plt.ylabel("position/angle (rad)")
            # plt.plot(t, mujoco.mju_sub(knee_joint_smooth_force, knee_joint_bias_force, model.nv), "y")

            plt.subplot(5, 1, 4)
            # plt.plot(t, qact_exo_lknee, "g-")
            # plt.plot(t, qact_exo_lknee_inertia, "b-")
            plt.plot(t, np.subtract(qref2[:min_length], qact2[:min_length]), "k")
            plt.plot(t, qref3[:min_length], "r")
            plt.plot(t, qact3[:min_length], "b")

            plt.legend(
                [
                    "error",
                    "qref_right_hip",
                    "qact_right_hip",
                ]
            )
            plt.ylabel("position/angle (rad)")


            plt.subplot(5, 1, 5)
            # plt.plot(t, qact_exo_lknee, "g-")
            # plt.plot(t, qact_exo_lknee_inertia, "b-")
            plt.plot(t, grav[:min_length], "k")
            plt.plot(t, left_foot_sensor[:min_length], "r")
            plt.plot(t, right_foot_sensor[:min_length], "b")
            plt.plot(t, np.add(left_foot_sensor[:min_length], right_foot_sensor[:min_length]), "g")
            # plt.plot(t, contact_force[:min_length], "r")
            plt.legend(
                [
                    "gravity",
                    "left_foot_sensor",
                    "right_foot_sensor",
                    "total_force"
                    # "contact_force",
                ]
            )
            plt.ylabel("Force(N)")


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

glfw.terminate()
