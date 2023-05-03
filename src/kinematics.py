from numpy import arctan2, arccos, sqrt
from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_param = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def forward_kinematics(self, thetas=None):
        if not thetas:
            thetas = self.get_links_position()
        transformation = get_transformation_matrix(self.dh_param, thetas)
        position = extract_position(transformation)
        orientation = extract_orientation(transformation)
        return np.array(position + orientation)

    def inverse_kinematics(self, position, orientation):
        thetas = [None] * 7

        initial_position = self.get_links_position()
        tm = get_transformation_matrix(self.dh_param, initial_position)
        # print(tm)
        #
        # P06 = extract_position(tm)
        # print(self.forward_kinematics(initial_position))
        # print(self.dh_param)
        # P04 = P06 - P46

        # thetas.insert(1, arctan2(Pyi - D6i * Ayi, Pxi - D6i * Axi))
        #
        # thetas.insert(3, arccos((l1 ** 2 + A2i ** 2 - P14l) / 2 * l1 * a2) - arctan2(D4i, A3i))
        #
        # thetas.insert(2, arctan2(z14, sqrt(x14 ** 2 + y14 ** 2))
        #               + arccos((A2i ** 2 - P14l ** 2 - l1 ** 2) / 2 * a2 * P14l))
        #
        # thetas.insert(5, arccos(R6zl @ R3zl))
        #
        # thetas.insert(4, arctan2(Ly, Lx))
        #
        # thetas.insert(6, arctan2(Jz, Iz))

        return thetas
