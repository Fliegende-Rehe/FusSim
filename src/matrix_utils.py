import numpy as np


def transformation_matrix(theta, offset, alpha, length):
    theta, alpha = np.deg2rad(theta), np.deg2rad(alpha)
    sin_t, cos_t = np.sin(theta), np.cos(theta)
    sin_a, cos_a = np.sin(alpha), np.cos(alpha)
    return np.array([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def extract_position(matrix):
    return [float("{:f}".format(float(row))) for row in matrix[:3, 3]]


def extract_orientation(matrix):
    _, _, nz = matrix[:3, 0]
    _, _, oz = matrix[:3, 1]
    ax, ay, az = matrix[:3, 2]
    angles = [np.arctan2(ay, ax),
              np.arctan2(np.sqrt(1 - az ** 2), az),
              np.arctan2(oz, -nz)]
    return [np.rad2deg(ang) for ang in angles]


def get_transformation_matrix(dh_table, thetas):
    transformation = []
    for dh, ang in zip(dh_table, thetas):
        theta, offset, alpha, length = dh.values()
        a = transformation_matrix(theta + ang, offset, alpha, length)
        transformation = transformation @ a if dh_table.index(dh) != 0 else a
    return transformation
