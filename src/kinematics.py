from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links

    def get_transformation_matrix(self, links_angle: list[float]) -> np.array(list[list[float]]):
        transformation = dh_table(*self.links[0].dh)
        for link, ang in zip(self.links[1:], links_angle):
            dh_param = link.dh
            # dh_param[0] = ang
            transformation = transformation @ dh_table(*dh_param)
        return transformation

    def forward_kinematics(self, links_angle):
        tm = self.get_transformation_matrix(links_angle)
        ee_cord = tm[:3, 3]
        euler_ang = euler_angles(tm[:3, :3])
        return np.concatenate((ee_cord, euler_ang))

    def get_links_position(self, position: list[float], orientation: list[float]) -> list[float]:
        return []
