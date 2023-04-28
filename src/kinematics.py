import itertools as it
from numpy.linalg import inv
from scipy.spatial.transform import Rotation
from math import degrees, radians

from .matrix_utils import *


class Kinematics:
    def __init__(self, links):
        self.links_origin = [lnk.origin for lnk in links]
        self.links_length = [lnk.length for lnk in links]
        self.base_frame = np.eye(4)
        self.transformation = None

    def get_transformation(self):
        pass

    def set_transformation(self, q):
        q1, q2, q3, q4, q5, q6 = q
        l1, l2, l3, l4, l5, l6 = self.links_length
        return [Rz(q1) @ Tz(l1),
                Ry(q2) @ Tx(l2),
                Ry(q3) @ Tz(l3),
                Rx(q4) @ Tz(l4),
                Ry(q5) @ Tx(l5),
                Rx(q6) @ Tx(l6)
                ]

    def forward_kinematics(self, q):
        self.transformation = self.set_transformation(q)

        def f(transformation):
            ret = self.base_frame
            for a in transformation:
                ret = ret @ a
            return ret

        return f(self.transformation)
