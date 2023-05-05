import numpy as np


def axis_angle_rot_matrix(k, q):
    """
    Creates a 3x3 rotation matrix in 3D space from an axis and an angle.

    Input
    :param k: A 3 element array containing the unit axis to rotate around (kx,ky,kz)
    :param q: The angle (in radians) to rotate by

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """

    # 15 pts
    c_theta = np.cos(q)
    s_theta = np.sin(q)
    v_theta = 1 - np.cos(q)
    kx = k[0]
    ky = k[1]
    kz = k[2]

    # First row of the rotation matrix
    r00 = kx * kx * v_theta + c_theta
    r01 = kx * ky * v_theta - kz * s_theta
    r02 = kx * kz * v_theta + ky * s_theta

    # Second row of the rotation matrix
    r10 = kx * ky * v_theta + kz * s_theta
    r11 = ky * ky * v_theta + c_theta
    r12 = ky * kz * v_theta - kx * s_theta

    # Third row of the rotation matrix
    r20 = kx * kz * v_theta - ky * s_theta
    r21 = ky * kz * v_theta + kx * s_theta
    r22 = kz * kz * v_theta + c_theta

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix
