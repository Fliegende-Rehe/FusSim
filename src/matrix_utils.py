import sympy as sp
import numpy as np


def ee_transformation(thetas, dh_table, base_frame=sp.eye(4)):
    for alpha, length, offset, theta in zip(dh_table['alpha'], dh_table['length'],
                                            dh_table['offset'], thetas):
        a = transformation_matrix(sp.rad(alpha), length, theta, offset)
        base_frame = base_frame * a
    return base_frame


def target_transformation(target):
    px, py, pz, alpha, beta, gamma = target
    # return eul2rot(alpha, beta, gamma) * translation_matrix(px, py, pz)
    return translation_matrix(px, py, pz) * eul2rot(alpha, beta, gamma)


def transformation_matrix(alpha, length, theta, offset):
    sin_t, sin_a = sp.sin(theta), sp.sin(alpha)
    cos_t, cos_a = sp.cos(theta), sp.cos(alpha)
    return sp.Matrix([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def eul2rot(alpha, beta, gamma):
    # return Rz(alpha) * Ry(beta) * Rz(gamma)
    return Rz(alpha)


def rot2eul(matrix):
    alpha = sp.atan2(matrix[0, 2], -matrix[1, 2])
    beta = sp.acos(matrix[2, 2])
    gamma = sp.atan2(matrix[2, 0], matrix[2, 1])
    return sp.Matrix([alpha, beta, gamma])


def jacobian_matrix(matrix, thetas):
    jacobian = []
    for cell in matrix:
        j = [sp.diff(cell, theta) for theta in thetas]
        jacobian.append(j)
    return sp.Matrix(jacobian)

def inverse_jacobian(J, thetas):
    return sp.Matrix(np.linalg.pinv(J(thetas)))

def translation_matrix(px, py, pz):
    return sp.Matrix([
        [1, 0, 0, px],
        [0, 1, 0, py],
        [0, 0, 1, pz],
        [0, 0, 0, 1]
    ])


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
