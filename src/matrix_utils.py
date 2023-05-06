import sympy as sp
import numpy as np

def full_transformation(thetas, dh_table, base_frame=sp.eye(4)):
    for alpha, length, offset, theta in zip(dh_table['alpha'], dh_table['length'],
                                            dh_table['offset'], thetas):
        a = transformation_matrix(sp.rad(alpha), length, theta, offset)
        base_frame = base_frame * a

    return base_frame


def transformation_matrix(alpha, length, theta, offset):
    sin_t, sin_a = sp.sin(theta), sp.sin(alpha)
    cos_t, cos_a = sp.cos(theta), sp.cos(alpha)

    return sp.Matrix([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def rot2eul(matrix):
    beta = sp.acos(matrix[2, 2])
    alpha = sp.atan2(matrix[0, 2], -matrix[1, 2])
    gamma = sp.atan2(matrix[2, 0], matrix[2, 1])

    return sp.Matrix([alpha, beta, gamma])


def jacobian_matrix(matrix, joints):
    jacobian = []
    for p in matrix:
        jacobian.append([sp.diff(p, j) for j in joints])

    return sp.Matrix(jacobian)

