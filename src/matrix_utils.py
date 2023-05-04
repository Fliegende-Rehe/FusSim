from numpy import cos, sin, eye, deg2rad, array
from scipy.spatial.transform import Rotation
from sympy import symbols, zeros

ROTATION_SEQUENCE = 'zyz'


def full_transformation(dh_table, thetas):
    transformation = eye(4)
    for dh, ang in zip(dh_table, thetas):
        theta, offset, alpha, length = dh.values()
        a = transformation_matrix(theta + ang, offset, alpha, length)
        transformation = transformation @ a if dh_table.index(dh) != 0 else a
    return transformation


def transformation_matrix(theta, offset, alpha, length):
    theta, alpha = deg2rad(theta), deg2rad(alpha)
    sin_t, cos_t = sin(theta), cos(theta)
    sin_a, cos_a = sin(alpha), cos(alpha)
    return array([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def rot2eul(matrix, deg):
    rot = Rotation.from_matrix(matrix)
    return rot.as_euler(ROTATION_SEQUENCE, degrees=deg).tolist()


def eul2rot(euler_angle):
    r = Rotation.from_euler(ROTATION_SEQUENCE, euler_angle, degrees=True)
    return r.as_matrix()


def normalize_row(row):
    return [float("{:f}".format(float(row))) for row in row]

