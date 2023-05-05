import numpy as np

from scipy.spatial.transform import Rotation

ROTATION_SEQUENCE = 'zyz'


def full_transformation(dh_table, thetas):
    transformation = np.eye(4)
    for dh, ang in zip(dh_table, thetas):
        theta, offset, alpha, length = dh.values()
        a = transformation_matrix(theta + ang, offset, alpha, length)
        transformation = transformation @ a if dh_table.index(dh) != 0 else a
    return transformation


def transformation_matrix(theta, offset, alpha, length):
    theta, alpha = np.deg2rad(theta), np.deg2rad(alpha)
    sin_t, sin_a = np.sin(theta), np.sin(alpha)
    cos_t, cos_a = np.cos(theta), np.cos(alpha)
    return np.array([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def rot2eul(matrix, deg):
    rot = Rotation.from_matrix(matrix)
    return rot.as_euler(ROTATION_SEQUENCE, degrees=deg)
