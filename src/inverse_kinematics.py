from .forward_kinematics import *

import itertools as it
import numpy as np
from numpy.linalg import inv

# Inverse kinematics solver function
def IK_solve(base_frame, ee_frame):
    # Get the transformation from the base frame to the end-effector frame
    frame = inv(base_frame) @ ee_frame

    # Compute the coordinates of the wrist center (WC)
    Tw = frame @ inv(Tz(l[5])) @ inv(Tz(l[4]))
    x, y, z = Tw[0, 3], Tw[1, 3], Tw[2, 3]

    # Solve for q1
    q1_sols = [-np.arctan2(x, y), -np.arctan2(x, y) + np.pi]

    for q1 in q1_sols:
        # Solve for q2
        d = np.sqrt(x ** 2 + y ** 2)
        e = z - l[0]
        h = np.sqrt(d ** 2 + e ** 2)

        theta_1_sols = [np.arccos((l[1] ** 2 + h ** 2 - (l[2] + l[3]) ** 2) / (2 * l[1] * h))]
        theta_1_sols.append(-theta_1_sols[0])

        theta_2_sols = [np.arccos((l[0] ** 2 + h ** 2 - (x ** 2 + y ** 2 + z ** 2)) / (2 * l[0] * h))]
        theta_2_sols.append(-theta_2_sols[0])

        q2_sols = [np.pi - (theta_1 + theta_2) for theta_1, theta_2 in it.product(theta_1_sols, theta_2_sols)]

        for q2 in q2_sols:
            # Solve for q3
            q3_sols = [np.arccos(
                -(l[1] ** 2 + (l[2] + l[3]) ** 2 - ((z - l[0]) ** 2 + x ** 2 + y ** 2)) / (2 * l[1] * (l[2] + l[3])))]
            q3_sols.append(-q3_sols[0])

            for q3 in q3_sols:
                # Solve for q5
                T123 = T1(q1) @ T2(q2) @ T3(q3)
                T456 = inv(Tz(l[3])) @ inv(T123) @ Tw
                q5_sols = [np.arccos(T456[2, 2])]
                q5_sols.append(-q5_sols[0])
                for q5 in q5_sols:
                    # Solve for q4
                    if np.abs(q5) > 1e-3:
                        q4_sols = [np.arcsin(T456[1, 2] / np.sin(q5))]
                        q4_sols.append(np.pi - q4_sols[0])
                    else:
                        q4_sols = [0]
                    for q4 in q4_sols:
                        # Solve for q6
                        if np.abs(q5) > 1e-3:
                            q6_sols = [np.arccos(-T456[2, 0] / np.sin(q5))]
                            q6_sols.append(-q6_sols[0])
                        else:
                            q6_sols = [np.arccos(T456[0, 0])]
                            q6_sols.append(-q6_sols[0])
                        for q6 in q6_sols:
                            q = [q1, q2, q3, q4, q5, q6]

                            if np.sum(np.abs(transform_base(base_frame, q) - ee_frame)) < 1e-5:
                                return q
