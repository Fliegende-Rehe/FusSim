import numpy as np


def rotate_x(angle):
    """
    Returns the 3x3 rotation matrix for a rotation around the x-axis by the given angle in radians.
    """
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])


def rotate_y(angle):
    """
    Returns the 3x3 rotation matrix for a rotation around the y-axis by the given angle in radians.
    """
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])


def rotate_z(angle):
    """
    Returns the 3x3 rotation matrix for a rotation around the z-axis by the given angle in radians.
    """
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
