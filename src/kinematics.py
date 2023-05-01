from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links

    def get_transformation_matrix(self, links_angle: list[float]) -> np.array(list[list[float]]):
        self.links[0].dh['z'] += links_angle[0]
        transformation = dh_table(*self.links[0].dh.values())
        for link, ang in zip(self.links[1:], links_angle[1:]):
            dh_param = link.dh
            dh_param['z'] += ang
            transformation = transformation @ dh_table(*dh_param.values())
        return transformation

    def forward_kinematics(self, links_angle):
        tm = self.get_transformation_matrix(links_angle)
        ee_cord = tm[:3, 3]
        euler_ang = euler_angles(tm[:3, :3])
        return np.concatenate((ee_cord, euler_ang))

    def get_links_position(self, position: list[float], orientation: list[float]) -> list[float]:
        return []
