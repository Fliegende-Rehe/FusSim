import numpy as np
from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links, dh_table):
        thetas = dynamicsymbols('theta0:{}'.format(len(links)))
        ee_frame = ee_transformation(thetas, dh_table)

        position = get_position(ee_frame)
        orientation = get_orientation(ee_frame)
        distance = position.jacobian(thetas)
        angle = orientation.jacobian(thetas)

        self.forward = sp.lambdify(thetas, position.col_join(orientation))
        self.jacobian = sp.lambdify(thetas, distance.col_join(angle))

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, theta=None):
        if theta is None:
            theta = self.get_links_position()

        return self.forward(*theta).T[0]

    def inverse_kinematics(self, target, dt=0.1, difference=0.1):
        target = np.array(target, dtype=float)
        thetas = self.get_links_position()
        error = target - self.forward_kinematics()
        while any(abs(element) > difference for element in error):
            inverse_jacobian = np.linalg.pinv(self.jacobian(*thetas))
            thetas = thetas + inverse_jacobian @ error * dt
            error = target - self.forward_kinematics(thetas)

        ik_solution = (thetas % (2 * np.pi))
        ik_solution = np.where(ik_solution > np.pi, ik_solution - (2 * np.pi), ik_solution)
        return ik_solution.tolist()

    def get_links_position(self):
        return np.array([link.get_position() for link in self.links], dtype=float)
