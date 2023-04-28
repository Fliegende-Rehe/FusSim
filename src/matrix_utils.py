import numpy as np
import sympy as sp


def Rx(q):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(q), - np.sin(q), 0],
                     [0, np.sin(q), np.cos(q), 0],
                     [0, 0, 0, 1]])


def Ry(q):
    return np.array([[np.cos(q), 0, np.sin(q), 0],
                     [0, 1, 0, 0],
                     [- np.sin(q), 0, np.cos(q), 0],
                     [0, 0, 0, 1]])


def Rz(q):
    return np.array([[np.cos(q), - np.sin(q), 0, 0],
                     [np.sin(q), np.cos(q), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Tx(d):
    return np.array([[1, 0, 0, d],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Ty(d):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, d],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Tz(d):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])


def compute_Jacobian(T_matr, joints):
    pos = T_matr  # [:3, 3]
    J = []
    for p in pos:
        Ji = []
        for j in joints:
            Ji.append(sp.diff(p, j))  # .simplify())
        J.append(Ji)
    J = sp.Matrix(J)
    return J


def Check_singular(Jacobian):
    return np.linalg.matrix_rank(Jacobian) < 6


def inverse_Jacobian(J, qi):
    return sp.Matrix(np.linalg.pinv(J(float(qi[0]), float(qi[1]),
                                      float(qi[2]), float(qi[3]), float(qi[4]))))


def compute_error(des_pos, f, q, qi):
    vect = des_pos - sp.Matrix(f(float(qi[0]), float(qi[1]),
                                 float(qi[2]), float(qi[3]), float(qi[4])))
    angle = sp.Matrix([q(float(qi[0]), float(qi[1]),
                         float(qi[2]), float(qi[3]), float(qi[4]))])
    error = vect.row_insert(4, angle)
    dist = sp.sqrt(vect.dot(vect)).evalf()
    unit_dist = np.abs(angle[0])
    return dist, unit_dist, error
