import numpy as np
import sympy as sp


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

# from jacobian import jacobian
# import numpy as np


# def jacobian(theta, dh_param):
#     # Calculate the Jacobian matrix using the current joint angles
#     J = sp.zeros(6, 6)
#     for i in range(6):
#         for j in range(6):
#             J[i, j] = sp.diff(symbol_forward_kinematics(theta, dh_param)[i], theta[j])
#     return J

# # Define the robot arm parameters
# L1 = 2.0
# L2 = 3.0
# L3 = 2.0
# L4 = 1.5
# L5 = 1.0
# L6 = 0.5
#
# # Define the target end-effector pose (position and orientation)
# px = 4.0
# py = 1.0
# pz = 3.0
# ox = 0.0
# oy = 0.0
# oz = 0.0
#
# # Define the maximum number of iterations and the error tolerance
# max_iter = 100
# tol = 0.01
#
# # Define the initial joint angles
# theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#
# # Perform the inverse kinematics iteration
# for i in range(max_iter):
#     # Calculate the current end-effector pose using the current joint angles
#     e = np.array(
#         [L2 * np.cos(theta[0]) + L3 * np.cos(theta[0] + theta[1]) + L4 * np.cos(theta[0] + theta[1] + theta[2]),
#          L2 * np.sin(theta[0]) + L3 * np.sin(theta[0] + theta[1]) + L4 * np.sin(theta[0] + theta[1] + theta[2]),
#          L1 + L5 * np.sin(theta[3]) + L6 * np.sin(theta[3] + theta[4]),
#          np.cos(theta[0] + theta[1] + theta[2]) * np.cos(theta[3] + theta[4] + theta[5]) - np.sin(
#              theta[0] + theta[1] + theta[2]) * np.sin(theta[4] + theta[5]),
#          np.sin(theta[0] + theta[1] + theta[2]) * np.cos(theta[3] + theta[4] + theta[5]) + np.cos(
#              theta[0] + theta[1] + theta[2]) * np.sin(theta[4] + theta[5]),
#          np.sin(theta[3] + theta[4] + theta[5]) * np.cos(theta[4])])
#
#     # Calculate the error between the current end-effector pose and the target pose
#     error = np.array([px, py, pz, ox, oy, oz]) - e
#
#     # Check if the error is below the tolerance
#     if np.linalg.norm(error) < tol:
#         print("Inverse kinematics converged after %d iterations" % (i + 1))
#         break
#
#     # Calculate the Jacobian matrix using the current joint angles
#     J = jacobian(theta)
#
#     # Calculate the joint angle increments using the pseudoinverse of the Jacobian matrix
#     dtheta = np.linalg.pinv(J) @ error
#
#     # Update the joint angles
#     theta += dtheta
#
# # Print the final joint angles
# print("Final joint angles:")
# print(theta)
