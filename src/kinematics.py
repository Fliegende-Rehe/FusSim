from .matrix_utils import *
import itertools as it
from numpy.linalg import inv
from scipy.spatial.transform import Rotation


class Kinematics:
    def __init__(self, links):
        self.links_origin = [lnk.origin for lnk in links]
        self.links_length = [lnk.length for lnk in links]
        self.base_frame = np.eye(4)
        self.transformation = None

    def get_transformation(self, q):
        q1, q2, q3, q4, q5, q6 = q
        l1, l2, l3, l4, l5, l6 = self.links_length
        return [Rz(q1) @ Tz(l1),
                Ry(q2) @ Tx(l2),
                Ry(q3) @ Tz(l3),
                Rx(q4) @ Tz(l4),
                Ry(q5) @ Tx(l5),
                Rx(q6) @ Tx(l6)
                ]

    def ik_solver(self, f, q, init_angle, des_pos, J, dt=0.001, Kp=10, max_dist=0.1):
        dist_arr, vectors_arr = [], []
        qi = init_angle
        dist, unit_dist, error = compute_error(des_pos, f, q, qi)
        dist_arr.append(dist)
        vectors_arr.append(unit_dist)
        while dist > max_dist or unit_dist > 10 ** (-5):
            Jinv = inverse_Jacobian(J, qi)
            qi = (qi + dt * Jinv * Kp * (error)).evalf()
            dist, unit_dist, error = compute_error(des_pos, f, q, qi)
            dist_arr.append(dist)
            vectors_arr.append(unit_dist)

        return qi, dist_arr, vectors_arr

    def get_Td(self, desire_position, desire_orientation):
        Td = inv(self.base_frame)
        Td[:3, :3] = Rotation.from_euler('xyz', desire_orientation).as_matrix()
        Td[:3, 3] = np.array(desire_position)
        return Td

    def inverse_kinematics(self, desire_position, desire_orientation, ee_frame):
        Td = self.get_Td(desire_position, desire_orientation)
        Tw = Td @ inv(ee_frame)
        x, y, z = Tw[0, 3], Tw[1, 3], Tw[2, 3]
        l = self.links_length
        q1_sols = [-np.arctan2(x, y)]
        q1_sols.append(q1_sols[0] + np.pi)

        for q1 in q1_sols:
            d = np.sqrt(x ** 2 + y ** 2)
            e = z - l[0]
            h = np.sqrt(d ** 2 + e ** 2)
            theta_1_sols = [np.arccos((l[1] ** 2 + h ** 2 - (l[2] + l[3]) ** 2) / (2 * l[1] * h))]
            theta_1_sols.append(-theta_1_sols[0])

            theta_2_sols = [np.arccos((l[0] ** 2 + h ** 2 - (x ** 2 + y ** 2 + z ** 2)) / (2 * l[0] * h))]
            theta_2_sols.append(-theta_2_sols[0])

            q2_sols = [np.pi - (theta_1 + theta_2) for theta_1, theta_2 in it.product(theta_1_sols, theta_2_sols)]

            for q2 in q2_sols:
                q3_sols = [np.arccos(-(l[1] ** 2 + (l[2] + l[3]) ** 2 - ((z - l[0]) ** 2 + x ** 2 + y ** 2)) / (
                        2 * l[1] * (l[2] + l[3])))]
                q3_sols.append(-q3_sols[0])

                for q3 in q3_sols:
                    T1 = Rz(q1) @ Tz(self.links_length[0])
                    T2 = Ry(q2) @ Tz(self.links_length[1])
                    T3 = Ry(q3) @ Tz(self.links_length[2])

                    T123 = T1 @ T2 @ T3
                    T456 = inv(Tz(l[3])) @ inv(T123) @ Tw
                    q5_sols = [np.arccos(T456[2, 2])]
                    q5_sols.append(-q5_sols[0])
                    for q5 in q5_sols:
                        if np.abs(q5) > 1e-3:
                            q4_sols = [np.arcsin(T456[1, 2] / np.sin(q5))]
                            q4_sols.append(np.pi - q4_sols[0])
                        else:
                            q4_sols = [0]
                        for q4 in q4_sols:
                            if np.abs(q5) > 1e-3:
                                q6_sols = [np.arccos(-T456[2, 0] / np.sin(q5))]
                                q6_sols.append(-q6_sols[0])
                            else:
                                q6_sols = [np.arccos(T456[0, 0])]
                                q6_sols.append(-q6_sols[0])
                            for q6 in q6_sols:
                                q = [q1, q2, q3, q4, q5, q6]

                                return q

    def transform_base(self, trans, q):
        return trans @ self.forward_kinematics(q)

    def forward_kinematics(self, q):
        self.transformation = self.get_transformation(q)
        A = self.transformation

        def f(A):
            ret = self.base_frame
            for a in A:
                ret = ret @ a
            return ret

        return f(A)
