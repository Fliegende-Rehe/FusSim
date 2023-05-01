import numpy as np
import sympy as sp


def euler_angles(transformation_matrix):
    nx, ny, nz = transformation_matrix[:3, 0]
    ox, oy, oz = transformation_matrix[:3, 1]
    ax, ay, az = transformation_matrix[:3, 2]
    z = np.arctan2(ay, ax)
    y_i = np.arctan2(np.sqrt(1 - az ** 2), az)
    z_ii = np.arctan2(oz, -nz)
    return np.array([np.rad2deg(z), np.rad2deg(y_i), np.rad2deg(z_ii)])


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


def euler_yzx_to_axis_angle(y_euler, z_euler, x_euler, normalize=True):
    c1 = np.cos(y_euler / 2)
    s1 = np.sin(y_euler / 2)
    c2 = np.cos(z_euler / 2)
    s2 = np.sin(z_euler / 2)
    c3 = np.cos(x_euler / 2)
    s3 = np.sin(x_euler / 2)
    c1c2 = c1 * c2
    s1s2 = s1 * s2
    w = c1c2 * c3 - s1s2 * s3
    x = c1c2 * s3 + s1s2 * c3
    y = s1 * c2 * c3 + c1 * s2 * s3
    z = c1 * s2 * c3 - s1 * c2 * s3
    angle = 2 * np.acos(w)
    if normalize:
        norm = x * x + y * y + z * z
        if norm < 0.001:
            x = 1
            y = 0
            z = 0
        else:
            norm = np.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    return x, y, z, angle
