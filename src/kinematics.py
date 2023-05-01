from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.dh_param = [lnk.dh for lnk in links]

    def get_ee_xyz(self, links_angle: list[float]) -> np.array(list[list[float]]):
        transformation = None
        for dh, ang in zip(self.dh_param, links_angle):
            dh['z'] += ang
            if self.dh_param.index(dh) == 0:
                transformation = dh_table(dh)
            else:
                transformation = transformation @ dh_table(dh)
        return transformation[:3, 3]
