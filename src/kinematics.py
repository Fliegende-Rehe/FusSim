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
        jacobian = distance.col_join(angle)

        self.position = sp.lambdify(thetas, position)
        self.orientation = sp.lambdify(thetas, orientation)
        self.jacobian = sp.lambdify(thetas, jacobian)

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, theta=None, deg=True) -> np.ndarray:
        if theta is None:
            theta = self.get_links_position()

        theta = np.array([(np.deg2rad(tar) if deg else tar) for tar in theta], dtype=float)

        return np.concatenate([self.position(*theta), self.orientation(*theta)])

    def compute_error(self, qi, target):
        position = sp.Matrix(target[:3] - self.position(*qi))
        orientation = sp.Matrix(target[3:] - self.orientation(*qi))
        error = position.col_join(orientation)

        distance = sp.sqrt(error.dot(error)).evalf()
        return distance, error

    def inverse_kinematics(self, target, dt=0.01, min_distance=0.1):
        target = np.array(target, dtype=float)
        qi = np.array(self.get_links_position(False), dtype=float)
        distance, error = self.compute_error(qi, target)
        while distance > min_distance:
            Jinv = sp.Matrix(np.linalg.pinv(self.jacobian(*qi)))
            qi += np.array((dt * Jinv * error).evalf().T.tolist()[0], dtype=float)
            distance, error = self.compute_error(qi, target)
        return np.rad2deg(qi)

    def get_links_position(self, deg=True) -> list[float]:
        if deg:
            return [link.get_position() for link in self.links]
        else:
            return [np.deg2rad(link.get_position()) for link in self.links]
