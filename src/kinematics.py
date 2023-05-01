from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links

    def get_transformation_matrix(self, links_angle: list[float]) -> np.array(list[list[float]]):
        transformation = None
        for link, ang in zip(self.links, links_angle):
            dh_param = link.dh
            dh_param['z'] -= ang
            if self.links.index(link) == 0:
                transformation = dh_table(dh_param)
            else:
                transformation = transformation @ dh_table(dh_param)
        return transformation

    def forward_kinematics(self, links_angle):
        tm = self.get_transformation_matrix(links_angle)
        ee_cord = tm[:3, 3]
        euler_ang = euler_angles(tm[:3, :3])
        return np.concatenate((ee_cord, euler_ang))
