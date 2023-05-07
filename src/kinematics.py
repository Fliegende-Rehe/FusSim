import numpy as np
from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link], dh_table) -> None:
        thetas = list(dynamicsymbols('theta0:6'))
        ee_frame = ee_transformation(thetas, dh_table)
        self.forward = sp.lambdify(thetas, ee_frame[:3, 3])
        self.euler = sp.lambdify(thetas, rot2eul(ee_frame[:3, :3]))

        transformations = list(dynamicsymbols('px py pz alpha beta gamma'))
        target_frame = target_transformation(transformations)
        self.target = sp.lambdify(transformations, rot2eul(ee_frame[:3, :3]))

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, target=None):
        if target is None:
            target = self.get_links_position()

        target = np.array([sp.rad(theta + tar)
                           for tar, theta in zip(target, self.dh_table['theta'])],
                          dtype=float)

        position = self.forward(*target).tolist()
        orientation = self.euler(*target).tolist()

        return [element[0] for element in (position + orientation)]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def inverse_kinematics(self, target, dt=0.001, kp=10, max_dist=0.1):
        qi = np.array(self.get_links_position(), dtype=float)
        ee_frame = self.forward_kinematics(qi)

        target_frame = self.target(target)
        desire_position = target_frame[:3, 3]
        desire_orientation = target_frame[:3, 0]

        q = 1 - desire_orientation.dot(ee_frame[:3, 0])

        j_position = jacobian_matrix(ee_frame[:3, 3], thetas)
        j_orientation = desire_orientation.T * (jacobian_matrix(ee_frame[:3, 0], thetas))
        J = sp.lambdify([thetas, target], j_position.row_insert(4, j_orientation))

        dist_arr, vectors_arr = [], []

        dist, unit_dist, error = self.compute_error(qi, target)
        dist_arr.append(dist)
        vectors_arr.append(unit_dist)
        while dist > max_dist or unit_dist > 10 ** (-5):
            Jinv = inverse_jacobian(self.j_matrix, qi)
            qi = (qi + dt * Jinv * kp * error).evalf()
            dist, unit_dist, error = self.compute_error(qi, target)

            print(f"{dist = }, {unit_dist = }, {len(vectors_arr) = }", end='\r')
            dist_arr.append(dist)
            vectors_arr.append(unit_dist)

        qi = [q for q in qi]

        print()
        new_q_sol_arr = []
        for a in qi:
            a = a % (2 * sp.pi)
            if a > sp.pi:
                a -= 2 * sp.pi
            new_q_sol_arr.append(sp.deg(a).evalf())
        return new_q_sol_arr

    def compute_error(self, thetas, target):
        vect = sp.Matrix(target[:3]) - sp.Matrix(self.forward_kinematics(thetas)[:3])
        angle = sp.Matrix([self.orientation(thetas, target)])
        error = vect.row_insert(4, angle)
        dist = sp.sqrt(vect.dot(vect)).evalf()
        unit_dist = np.abs(angle[0])
        return dist, unit_dist, error
