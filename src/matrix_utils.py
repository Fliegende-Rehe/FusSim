import numpy as np
import sympy as sp

def euler_angles(transformation_matrix):
    nx, ny, nz = transformation_matrix[:3, 0]
    ox, oy, oz = transformation_matrix[:3, 1]
    ax, ay, az = transformation_matrix[:3, 2]
    z = np.arctan2(ay, ax)
    y_i = np.arctan2(np.sqrt(1 - az ** 2), az)
    z_ii = np.arctan2(oz, -nz)
    return np.array([z, y_i, z_ii])


def dh_table(theta, alpha, offset, length):
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    sin_t, cos_t = np.sin(theta), np.cos(theta)
    sin_a, cos_a = np.sin(alpha), np.cos(alpha)
    transformation = np.array([
        [cos_t, -sin_t * cos_a, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])
    return transformation


def compute_jacobian(transformation_matrix, joints):
    positions = transformation_matrix[:3, 3]
    return sp.Matrix([[sp.diff(p, j) for j in joints] for p in positions])


def Check_singular(jacobian):
    return np.linalg.matrix_rank(jacobian) < 6


def inverse_Jacobian(jacobian, qi):
    qi = [float(val) for val in qi]
    return sp.Matrix(np.linalg.pinv(jacobian(*qi)))


def compute_error(des_pos, f, q, qi):
    qi = [float(val) for val in qi]
    pos = sp.Matrix(f(*qi))
    vect = des_pos - pos
    angle = sp.Matrix([q(*qi)])
    error = sp.Matrix.vstack(vect, angle)
    dist = sp.sqrt(vect.dot(vect)).evalf()
    unit_dist = abs(angle[0])
    return dist, unit_dist, error
