import numpy as np


def dh_table(dh_param):
    theta, offset, alpha, length = dh_param.values()
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    sin_t, cos_t = np.sin(theta), np.cos(theta)
    sin_a, cos_a = np.sin(alpha), np.cos(alpha)
    transformation = np.array([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],

        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],

        [0, sin_a, cos_a, offset],

        [0, 0, 0, 1]
    ])
    return transformation
