import numpy as np

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a rotation matrix.

    Parameters:
    q (list or np.array): Quaternion [q_w, q_x, q_y, q_z]

    Returns:
    np.array: Corresponding rotation matrix (3x3)
    """
    q_w, q_x, q_y, q_z = q

    # Compute the rotation matrix elements
    R = np.array([
        [1 - 2 * (q_y**2 + q_z**2), 2 * (q_x * q_y - q_z * q_w), 2 * (q_x * q_z + q_y * q_w)],
        [2 * (q_x * q_y + q_z * q_w), 1 - 2 * (q_x**2 + q_z**2), 2 * (q_y * q_z - q_x * q_w)],
        [2 * (q_x * q_z - q_y * q_w), 2 * (q_y * q_z + q_x * q_w), 1 - 2 * (q_x**2 + q_y**2)]
    ])

    return R
def transformaiton(R, t):
    """
    Convert a rotation matrix and translation vector into a transformation matrix.

    Parameters:
    R (np.array): Rotation matrix (3x3)
    t (list or np.array): Translation vector [t_x, t_y, t_z]

    Returns:
    np.array: Corresponding transformation matrix (4x4)
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def rotation_matrix_to_quaternion(T):
    """
    Convert a rotation matrix into a quaternion.

    Parameters:
    R (np.array): Rotation matrix (3x3)

    Returns:
    np.array: Corresponding quaternion [q_w, q_x, q_y, q_z]
    """
    R = T[:3, :3]
    q_w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
    q_x = (R[2, 1] - R[1, 2]) / (4.0 * q_w)
    q_y = (R[0, 2] - R[2, 0]) / (4.0 * q_w)
    q_z = (R[1, 0] - R[0, 1]) / (4.0 * q_w)

    return np.array([q_w, q_x, q_y, q_z])


# Define the transformation matrix for Tworld_backpack
q_w_bp = [0.5, 0.5, 0.5, 0.5]
rotation_matrix = quaternion_to_rotation_matrix(q_w_bp)
print("Rotation Matrix:\n", rotation_matrix)
trans_w_bp = [0, 0, 1.05]
Tw_bp = transformaiton(rotation_matrix, trans_w_bp)
print("Transformation Matrix:\n", Tw_bp)

# Define the transformation matrix for Tbackpack_leftthigh
trans_bp_lt = [0.182275, -0.211166, 0.13831]
q_bp_lt = [-1, 0, 0, 0]
Rbp_lt = quaternion_to_rotation_matrix(q_bp_lt)
Tbp_lt = transformaiton(Rbp_lt, trans_bp_lt)
print("Transformation Matrix:\n", Tbp_lt)


# Define the transformation matrix for Tleftthigh_leftshank
trans_lt_ls = [-0.01687, -0.37, 0]
q_lt_ls = [0, 0, 1, 0]
Rlt_ls = quaternion_to_rotation_matrix(q_lt_ls)
Tlt_ls = transformaiton(Rlt_ls, trans_lt_ls)
print("Transformation Matrix:\n", Tlt_ls)

#Define the transformation matrix for Tleftshank_leftfoot
trans_ls_lf = [-0.006, -0.36, 0]
q_ls_lf = [0.707107, -0.707107, 0, 0]
Rls_lf = quaternion_to_rotation_matrix(q_ls_lf)
Tls_lf = transformaiton(Rls_lf, trans_ls_lf)
print("Transformation Matrix:\n", Tls_lf)
q_w_lf = rotation_matrix_to_quaternion(Tls_lf)
trans_w_lf = Tls_lf[:3, 3]
print("Quaternion for world2left foot:\n", q_w_lf)
print("Translation for world2left foot:\n", trans_w_lf) 

#result of Tworld_leftshank
Tw_ls = Tw_bp @ Tbp_lt @ Tlt_ls
print("Transformation Matrix:\n", Tw_ls)
q_world_to_leftshank = rotation_matrix_to_quaternion(Tw_ls)
print("Quaternion for world2left shank:\n", q_world_to_leftshank)
trans_world_to_leftshank = Tw_ls[:3, 3]
print("Translation for world2left shank:", trans_world_to_leftshank)


#result of Tworld_humanleg

Tw_hl = np.eye(4)
q_w_hl = [0.5, 0.5, -0.5, -0.5]
Rw_hl = quaternion_to_rotation_matrix(q_w_hl)
trans_w_hl = [0.1381, 0.09, 0.2344]
T_w_hl_before = transformaiton(Rw_hl, trans_w_hl)
R_90 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
trans_90 = [0, 0, 0]
T_90 = transformaiton(R_90, trans_90)
T_after_rotate_90 = T_w_hl_before * T_90
q_world_to_human = rotation_matrix_to_quaternion(T_after_rotate_90)
print("Quaternion for world2human leg:\n", q_world_to_human)