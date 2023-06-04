from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links, dh_table):
        thetas = dynamicsymbols('theta0:{}'.format(len(links)))
        ee_frame = ee_transformation(thetas, dh_table)

        position = get_position(ee_frame)
        orientation = get_orientation(ee_frame)
        distance = position.jacobian(thetas)
        angle = orientation.jacobian(thetas)

        self.forward = sp.lambdify(thetas, position.col_join(orientation))
        self.jacobian = sp.lambdify(thetas, distance.col_join(angle))

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, theta=None):
        if theta is None:
            theta = self.get_links_position()

        return self.forward(*theta).T[0]

    def inverse_kinematics(self, target_position: np.ndarray, time_step=0.1, tolerance=0.1, damping_factor=0.0075):
        n_links = len(self.links)
        current_positions = self.get_links_position()
        error_vector = target_position - self.forward_kinematics()
        delta_theta = np.zeros_like(current_positions)

        while np.linalg.norm(error_vector) > tolerance:
            J = self.jacobian(*current_positions)
            JT = J.T
            damping_matrix = np.eye(n_links) * damping_factor ** 2
            JInv = np.linalg.inv(J @ JT + damping_matrix)
            pseudo_inverse_jacobian = JT @ JInv
            delta_theta[:] = pseudo_inverse_jacobian @ error_vector * time_step

            for i, link in enumerate(self.links):
                proposed_theta = current_positions[i] + delta_theta[i]
                if link.fit_limits(proposed_theta):
                    continue
                delta_theta[i] = link.min if proposed_theta < link.min else link.max
                delta_theta[i] -= current_positions[i]

            current_positions += delta_theta
            error_vector = target_position - self.forward_kinematics(current_positions)

        return self.angles_postprocessing(current_positions)

    def angles_postprocessing(self, thetas):
        thetas = np.remainder(thetas, 2 * np.pi)
        over_pi = np.where(abs(thetas) > np.pi)
        thetas[over_pi] -= np.sign(thetas[over_pi]) * 2 * np.pi
        return thetas

    def get_links_position(self):
        return np.array([link.get_position() for link in self.links], dtype=float)
