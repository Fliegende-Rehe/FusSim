from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links, dh_table) -> None:
        thetas = list(dynamicsymbols('theta0:6'))

        ee_frame = full_transformation(thetas, dh_table)

        self.fk_position = sp.lambdify(thetas, ee_frame[:3, 3])
        self.fk_orientation = sp.lambdify(thetas, rot2eul(ee_frame[:3, :3]))

        J_dist = jacobian_matrix(f, joints)
        J_angl = des_or.T * (jacobian_matrix(T05[:3, 0], joints))
        self.j_matrix = sp.lambdify(thetas, J_dist.row_insert(4, J_angl))

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, target=None) -> np.ndarray:
        if target is None:
            target = self.get_links_position()

        target = np.array([sp.rad(theta + tar) for tar, theta in zip(target, self.dh_table['theta'])], dtype=float)

        position = np.array(self.fk_position(*target))
        orientation = np.array(self.fk_orientation(*target))

        return np.concatenate([position, orientation])

    def inverse_kinematics(self, init_angle, des_pos, J, dt=0.001, Kp=10, max_dist=0.1):
        dist_arr, vectors_arr = [], []
        qi = init_angle
        dist, unit_dist, error = compute_error(des_pos, qi)
        dist_arr.append(dist)
        vectors_arr.append(unit_dist)
        while dist > max_dist or unit_dist > 10 ** (-5):
            Jinv = inverse_Jacobian(J, qi)
            qi = (qi + dt * Jinv * Kp * (error)).evalf()
            dist, unit_dist, error = compute_error(des_pos, qi)

            print(f"{dist = }, {unit_dist = }, {len(vectors_arr) = }", end='\r')
            dist_arr.append(dist)
            vectors_arr.append(unit_dist)

        return qi, dist_arr, vectors_arr

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]
