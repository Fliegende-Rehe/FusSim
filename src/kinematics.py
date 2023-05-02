from scipy.optimize import minimize

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_param = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def forward_kinematics(self, thetas):
        transformation = np.eye(4)
        for dh, ang in zip(self.dh_param, thetas):
            dh['z'] += ang
            transformation = transformation @ transformation_matrix(dh) \
                if self.dh_param.index(dh) != 0 else transformation_matrix(dh)
        position = extract_position(transformation)
        orientation = extract_orientation(transformation)
        return np.array(position + orientation)

    def inverse_kinematics(self, position, orientation):
        x, y, z = position
        roll, pitch, yaw = orientation

        def objective(thetas):
            pose = self.forward_kinematics(thetas)
            error = pose - np.array([x, y, z, roll, pitch, yaw])
            return np.sum(error ** 2)

        # Set the initial guess for theta based on link lengths and target position
        link_lengths = [link.length for link in self.links]
        target_distance = np.sqrt(x ** 2 + y ** 2 + (z - link_lengths[0]) ** 2)
        target_angle = np.arctan2(y, x)
        theta_initial = [target_angle, 0, target_distance - link_lengths[1], roll, pitch, yaw]

        result = minimize(objective, theta_initial, method='BFGS')
        theta = result.x

        return np.round(theta, decimals=6).tolist()
