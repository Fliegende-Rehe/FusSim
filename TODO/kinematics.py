import numpy as np
from .utils import rotate_x, rotate_y, rotate_z


class RobotArm:
    def __init__(self, links, lengths):
        """
        Initializes the robot arm with the given links and lengths.

        Parameters:
        links (list): the joint types ('revolute' or 'prismatic') for each joint
        lengths (list): the lengths of the links for each joint
        """
        self.links = links
        self.lengths = lengths

    def forward_kinematics(self, joint_angles):
        """
        Computes the forward kinematics for the given joint angles.

        Parameters:
        joint_angles (list): the joint angles [theta1, theta2, ..., theta6] in degrees

        Returns:
        pos (list): the position of the end-effector [x, y, z]
        ori (list): the orientation of the end-effector [roll, pitch, yaw] in degrees
        """
        # Convert the joint angles from degrees to radians
        joint_angles = np.deg2rad(joint_angles)

        # Compute the homogeneous transformation matrices for each joint
        T1 = np.array([
            [np.cos(joint_angles[0]), -np.sin(joint_angles[0]), 0, 0],
            [np.sin(joint_angles[0]), np.cos(joint_angles[0]), 0, 0],
            [0, 0, 1, self.lengths[0]],
            [0, 0, 0, 1]
        ])
        T2 = np.array([
            [np.cos(joint_angles[1]), -np.sin(joint_angles[1]),
             0, self.lengths[1]*np.cos(joint_angles[1])],
            [np.sin(joint_angles[1]), np.cos(joint_angles[1]),
             0, self.lengths[1]*np.sin(joint_angles[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T3 = np.array([
            [np.cos(joint_angles[2]), -np.sin(joint_angles[2]),
             0, self.lengths[2]*np.cos(joint_angles[2])],
            [np.sin(joint_angles[2]), np.cos(joint_angles[2]),
             0, self.lengths[2]*np.sin(joint_angles[2])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T4 = np.array([
            [np.cos(joint_angles[3]), -np.sin(joint_angles[3]),
             0, self.lengths[3]],
            [np.sin(joint_angles[3]), np.cos(joint_angles[3]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T5 = np.array([
            [np.cos(joint_angles[4]), -np.sin(joint_angles[4]),
             0, self.lengths[4]],
            [np.sin(joint_angles[4]), np.cos(joint_angles[4]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T6 = np.array([
            [np.cos(joint_angles[5]), -np.sin(joint_angles[5]),
             0, self.lengths[5]],
            [np.sin(joint_angles[5]), np.cos(joint_angles[5]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Compute the homogeneous transformation matrix for the end-effector
        T = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)

        # Extract the position and orientation from the homogeneous transformation matrix
        pos = T[:3, 3]
        ori = np.zeros(3)
        ori[0] = np.arctan2(T[2, 1], T[2, 2]) * 180 / np.pi
        ori[1] = np.arctan2(-T[2, 0], np.sqrt(T[2, 1] **
                            2 + T[2, 2]**2)) * 180 / np.pi
        ori[2] = np.arctan2(T[1, 0], T[0, 0]) * 180 / np.pi

        return pos, ori

    def inverse_kinematics(self, pos, ori):
        """
        Computes the inverse kinematics for the given position and orientation.

        Parameters:
        pos (list): the position of the end-effector [x, y, z]
        ori (list): the orientation of the end-effector [roll, pitch, yaw] in degrees

        Returns:
        joint_angles (list): the joint angles [theta1, theta2, ..., theta6] in degrees
        """
        # Convert the orientation from degrees to radians
        ori = np.deg2rad(ori)

        # Compute the rotation matrices for each axis
        R_x = rotate_x(ori[0])
        R_y = rotate_y(ori[1])
        R_z = rotate_z(ori[2])

        # Compute the rotation matrix for the end-effector
        R = np.dot(np.dot(R_z, R_y), R_x)

        # Compute the position of the wrist center
        wc = np.array(pos) - np.dot(R, np.array([0, 0, self.lengths[5]]))

        # Compute the joint angles for the first three joints
        joint_angles = [0, 0, 0]
        joint_angles[0] = np.arctan2(wc[1], wc[0]) * 180 / np.pi
        d = np.sqrt(wc[0]**2 + wc[1]**2) - self.lengths[0]
        a = wc[2] - self.lengths[1]
        beta = np.arctan2(-a, d)
        gamma = np.arccos((self.lengths[2]**2 - self.lengths[3]**2 +
                          d**2 + a**2) / (2 * self.lengths[2] * np.sqrt(d**2 + a**2)))
        joint_angles[1] = (beta - gamma) * 180 / np.pi
        alpha = np.arccos((self.lengths[2]**2 + self.lengths[3] **
                          2 - d**2 - a**2) / (2 * self.lengths[2] * self.lengths[3]))
        joint_angles[2] = (alpha - np.pi) * 180 / np.pi

        # Compute the rotation matrix for the first three joints
        R_0_3 = np.dot(np.dot(rotate_z(joint_angles[0]), rotate_y(
            joint_angles[1])), rotate_y(joint_angles[2]))

        # Compute the rotation matrix for the last three joints
        R_3_6 = np.dot(np.dot(np.linalg.inv(R_0_3), R), rotate_z(
            -joint_angles[3]) @ rotate_y(-joint_angles[4]) @ rotate_z(-joint_angles[5]))

        # Compute the joint angles for the last three joints
        joint_angles[3] = np.arctan2(R_3_6[1, 2], R_3_6[0, 2]) * 180 / np.pi
        joint_angles[4] = np.arctan2(
            np.sqrt(R_3_6[0, 2]**2 + R_3_6[1, 2]**2), R_3_6[2, 2]) * 180 / np.pi
        joint_angles[5] = np.arctan2(R_3_6[2, 1], -R_3_6[2, 0]) * 180 / np.pi

        return joint_angles

    def solve(self, pos, ori):
        """
        Computes the joint angles for the given position and orientation.

        Parameters:
        pos (list): the position of the end-effector [x, y, z]
        ori (list): the orientation of the end-effector [roll, pitch, yaw] in degrees

        Returns:
        joint_angles (list): the joint angles [theta1, theta2, ..., theta6] in degrees
        """
        # Compute the inverse kinematics
        joint_angles = self.inverse_kinematics(pos, ori)

        # Check if the joint angles are within the limits
        for i in range(6):
            if joint_angles[i] < self.limits[i][0]:
                joint_angles[i] = self.limits[i][0]
            elif joint_angles[i] > self.limits[i][1]:
                joint_angles[i] = self.limits[i][1]

        return joint_angles
