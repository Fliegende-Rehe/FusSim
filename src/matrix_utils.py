import sympy as sp
import numpy as np


def ee_transformation(angles, dh_table, frame=sp.eye(4)):
    for alpha, length, offset, theta, angle in zip(dh_table['alpha'], dh_table['length'],
                                                   dh_table['offset'], dh_table['theta'],
                                                   angles):
        a = transformation_matrix(sp.rad(alpha), length, sp.rad(theta) + angle, offset)
        frame = frame * a
    return frame


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
    alpha = np.degrees(np.arctan2(matrix[0, 2], -matrix[1, 2]))
    beta = np.degrees(np.arccos(matrix[2, 2]))
    gamma = np.degrees(np.arctan2(matrix[2, 0], matrix[2, 1]))
    return [alpha, beta, gamma]


def target_transformation(target):
    px, py, pz, alpha, beta, gamma = target
    return Rz(alpha) * Ry(beta) * Rz(gamma) * Txyz(px, py, pz)


def Ry(q):
    return sp.Matrix([
        [sp.cos(q), 0, sp.sin(q), 0],
        [0, 1, 0, 0],
        [-sp.sin(q), 0, sp.cos(q), 0],
        [0, 0, 0, 1]
    ])


def Rz(q):
    return sp.Matrix([
        [sp.cos(q), -sp.sin(q), 0, 0],
        [sp.sin(q), sp.cos(q), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def Txyz(px, py, pz):
    return sp.Matrix([
        [1, 0, 0, px],
        [0, 1, 0, py],
        [0, 0, 1, pz],
        [0, 0, 0, 1]
    ])
