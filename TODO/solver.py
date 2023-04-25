import numpy as np
from .kinematics import RobotArm


def solve_ik(robot: RobotArm, target_pos, target_ori, max_iterations=1000, tolerance=1e-5):
    """
    Solve the inverse kinematics for the desired end-effector position and orientation
    using an iterative method.

    Parameters:
    robot (RobotArm): the robot arm object
    target_pos (list): the desired end-effector position [x, y, z]
    target_ori (list): the desired end-effector orientation [roll, pitch, yaw] in degrees
    max_iterations (int): the maximum number of iterations to perform (default: 1000)
    tolerance (float): the tolerance for the error in the end-effector position (default: 1e-5)

    Returns:
    joint_angles (list): the resulting joint angles [theta1, theta2, ..., theta6] in degrees
    """
    # Convert the target orientation from degrees to radians
    target_ori = np.deg2rad(target_ori)

    # Initialize the joint angles to some initial guess
    joint_angles = np.zeros(6)

    # Perform the iterations until the error is below the tolerance or the maximum iterations are reached
    for i in range(max_iterations):
        # Compute the current end-effector position and orientation for the current joint angles
        current_pos, current_ori = robot.forward_kinematics(joint_angles)[:2]

        # Compute the error between the current and desired end-effector position and orientation
        error_pos = np.linalg.norm(target_pos - current_pos)
        error_ori = np.linalg.norm(target_ori - current_ori)

        # Check if the error is below the tolerance
        if error_pos < tolerance and error_ori < tolerance:
            break

        # Compute the Jacobian matrix for the current joint angles
        J = robot.jacobian(joint_angles)

        # Compute the joint angle update using the Moore-Penrose pseudoinverse
        delta_theta = np.linalg.pinv(
            J) @ (np.hstack((target_pos - current_pos, target_ori - current_ori)))

        # Update the joint angles
        joint_angles += delta_theta

    # Convert the joint angles from radians to degrees
    joint_angles = np.rad2deg(joint_angles)

    return joint_angles
