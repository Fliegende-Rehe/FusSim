from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links_origin = [lnk.origin for lnk in links]
        self.links_length = [lnk.length for lnk in links]
        self.base_frame = np.eye(4)

    def get_transformations(self, initial_position: list[float]):
        q1, q2, q3, q4, q5, q6 = initial_position
        l1, l2, l3, l4, l5, l6 = self.links_length
        return [Rz(q1) @ Tz(l1),
                Ry(q2) @ Tx(l2),
                Ry(q3) @ Tz(l3),
                Rx(q4) @ Tz(l4),
                Ry(q5) @ Tx(l5),
                Rx(q6) @ Tx(l6)
                ]

    def get_links_position(self, position: list[float], orientation: list[float]):
        return []

    def forward_kinematics(self, initial_position: list[float]):
        fk = self.base_frame
        for a in self.get_transformations(initial_position):
            fk = fk @ a
        return fk
