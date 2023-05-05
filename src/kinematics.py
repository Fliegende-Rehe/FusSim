from .matrix_utils import *
from .link import *


class Kinematics:

    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_table = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> np.ndarray:
        return np.array([link.get_position() for link in self.links])

    def forward_kinematics(self, thetas=None, deg=True) -> np.ndarray:
        if thetas is None:
            thetas = self.get_links_position()
        transformation = full_transformation(self.dh_table, thetas)
        position = transformation[:3, 3]
        orientation = rot2eul(transformation[:3, :3], deg)
        return np.concatenate([position, orientation])

    def inverse_kinematics(self, position, orientation) -> list:

        return []
