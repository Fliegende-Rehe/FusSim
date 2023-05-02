from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_param = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def get_ee_position(self):
        return [float("{:f}".format(float(row))) for row in self.get_transformation()[:3, 3]]

    def get_transformation(self) -> np.array(list[list[float]]):
        angles = self.get_links_position()
        transformation = []
        for dh, ang in zip(self.dh_param, angles):
            dh['z'] += ang
            transformation = transformation @ dh_table(dh) if self.dh_param.index(dh) != 0 else dh_table(dh)
        return transformation
