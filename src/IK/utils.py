import numpy as np


def Rx(q):
    """
    Rotation matrix for rotation about the x-axis by angle q in radians.
    """
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(q), -np.sin(q), 0],
                     [0, np.sin(q), np.cos(q), 0],
                     [0, 0, 0, 1]])


def Ry(q):
    """
    Rotation matrix for rotation about the y-axis by angle q in radians.
    """
    return np.array([[np.cos(q), 0, np.sin(q), 0],
                     [0, 1, 0, 0],
                     [-np.sin(q), 0, np.cos(q), 0],
                     [0, 0, 0, 1]])


def Rz(q):
    """
    Rotation matrix for rotation about the z-axis by angle q in radians.
    """
    return np.array([[np.cos(q), -np.sin(q), 0, 0],
                     [np.sin(q), np.cos(q), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Tx(d):
    """
    Translation matrix for translation along the x-axis by distance d.
    """
    return np.array([[1, 0, 0, d],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Ty(d):
    """
    Translation matrix for translation along the y-axis by distance d.
    """
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, d],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Tz(d):
    """
    Translation matrix for translation along the z-axis by distance d.
    """
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])


def Jacobian(F):
    """
    Calculates the Jacobian matrix given a list of transformation matrices F.
    """
    Fext = [np.eye(4)] + F
    u = [2, 0, 0, 2, 1, 2]  # fixed coordinate system axes
    U = [Fext[i][:3, u[i]] for i in range(6)]  # unit vectors along the axes

    O = [Fext[i][:3, 3] for i in range(7)]  # origin points of the coordinate systems

    J = np.zeros((6, 6))  # initialize the Jacobian matrix
    for i in range(6):
        J[:, i] = np.concatenate([np.cross(U[i], (O[6] - O[i])), U[i]])

    return J


def Check_singular(J):
    """
    Checks whether the given Jacobian matrix J is singular.
    """
    return np.linalg.matrix_rank(J) < 6
