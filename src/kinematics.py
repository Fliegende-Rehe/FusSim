import numpy as np
from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link], dh_table) -> None:
        thetas = list(dynamicsymbols('theta0:6'))
        ee_frame = ee_transformation(thetas, dh_table)
        self.ee_frame = sp.lambdify(thetas, ee_frame)

        target = list(dynamicsymbols('px py pz alpha beta gamma'))
        target_frame = target_transformation(target)
        self.target_frame = sp.lambdify(target, target_frame)

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, target=None):
        if target is None:
            target = self.get_links_position()

        target = np.array([sp.rad(tar) for tar in target], dtype=float)

        ee_frame = self.ee_frame(*target)
        position = ee_frame[:3, 3].tolist()
        orientation = rot2eul(ee_frame[:3, :3]).tolist()

        return [element[0] for element in (position + orientation)]

    def inverse_kinematics(self, target, dt=0.001, kp=10, max_dist=0.1):


        return []

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]
