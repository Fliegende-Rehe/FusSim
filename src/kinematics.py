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

    def forward_kinematics(self, theta):
        return self.forward(*theta).T[0]

    def inverse_kinematics(self, thetas, target, dt=0.1, difference=0.1):
        error = target - self.forward_kinematics(thetas)
        while any(abs(element) > difference for element in error):
            inverse_jacobian = np.linalg.pinv(self.jacobian(*thetas))
            thetas = thetas + inverse_jacobian @ error * dt
            for i, link in enumerate(self.links):
                if not link.fit_limits(thetas[i]):
                    thetas[i] = link.min if thetas[i] * link.direction < link.min else link.max
            error = target - self.forward_kinematics(thetas)
        return angles_postprocessing(thetas)

    # def inverse_kinematics(self, theta, target, time_step=0.1, tolerance=0.05, damping_factor=0.0075):
    #     error_vector = target - self.forward_kinematics(theta)
    #     delta_theta = np.zeros_like(theta)
    #
    #     while np.linalg.norm(error_vector) > tolerance:
    #         J = self.jacobian(*theta)
    #         JT = J.T
    #         damping_matrix = np.eye(len(self.links)) * damping_factor ** 2
    #         JInv = np.linalg.inv(J @ JT + damping_matrix)
    #         pseudo_inverse_jacobian = JT @ JInv
    #         delta_theta[:] = pseudo_inverse_jacobian @ error_vector * time_step
    #
    #         proposed_theta = theta + delta_theta
    #         for i, link in enumerate(self.links):
    #             if link.fit_limits(proposed_theta[i]):
    #                 continue
    #             delta_theta[i] = link.min if proposed_theta[i] < link.min else link.max
    #             delta_theta[i] -= theta[i]
    #
    #         theta += delta_theta
    #         error_vector = target - self.forward_kinematics(theta)
    #
    #     return self.angles_postprocessing(theta)


def angles_postprocessing(thetas):
    thetas = np.remainder(thetas, 2 * np.pi)
    over_pi = np.where(abs(thetas) > np.pi)
    thetas[over_pi] -= np.sign(thetas[over_pi]) * 2 * np.pi
    return thetas
