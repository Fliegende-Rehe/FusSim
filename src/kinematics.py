from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_param = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def get_transformation_matrix(self, thetas):
        transformation = np.eye(4)
        for dh, ang in zip(self.dh_param, thetas):
            dh['z'] += ang
            transformation = transformation @ transformation_matrix(dh) \
                if self.dh_param.index(dh) != 0 else transformation_matrix(dh)
        return transformation

    def forward_kinematics(self, thetas=None):
        if not thetas:
            thetas = self.get_links_position()
        transformation = self.get_transformation_matrix(thetas)
        position = extract_position(transformation)
        orientation = extract_orientation(transformation)
        return np.array(position + orientation)

    def inverse_kinematics(self, position, orientation):
        x_desired, y_desired, z_desired = position
        roll_desired, pitch_desired, yaw_desired = orientation

        result = []
        return result
