from sympy.physics.mechanics import dynamicsymbols

from .matrix_utils import *
from .link import *


class Kinematics:
    def __init__(self, links, dh_table):
        thetas = dynamicsymbols('theta0:{}'.format(len(links)))
        ee_frame = ee_transformation(thetas, dh_table)

        target = sp.Matrix(dynamicsymbols('px py pz phi nu psi'))
        Jv = jacobian_matrix(ee_frame[:3, 0], thetas)
        Jw = target[3:, 0].T * jacobian_matrix(ee_frame[3:, 0], thetas)
        jacobian = Jv.row_insert(3, Jw)

        self.ee_frame = sp.lambdify(thetas, ee_frame)
        self.jacobian = sp.lambdify([thetas, target], jacobian)

        q = 1 - (target[3:, 0].dot(ee_frame[:3, 0]))
        self.q = sp.lambdify([thetas, target], q)

        self.dh_table = dh_table
        self.links = links

    def forward_kinematics(self, theta=None, deg=True):
        if theta is None:
            theta = self.get_links_position()

        theta = np.array([(np.deg2rad(tar) if deg else tar) for tar in theta], dtype=float)

        return self.ee_frame(*theta).T.tolist()[0]

    def compute_error(self, qi, target):
        vect = target[:3, 0] - sp.Matrix(self.ee_frame(*qi)[:3, 0])
        angle = sp.Matrix([self.q(qi, target)])
        error = vect.row_insert(4, angle)
        dist = sp.sqrt(vect.dot(vect)).evalf()
        unit_dist = np.abs(angle[0])
        return dist, unit_dist, error

    def inverse_kinematics(self, target, dt=0.001, kp=10, max_dist=0.1):
        target = sp.Matrix(target)
        qi = self.get_links_position(False)
        dist, unit_dist, error = self.compute_error(qi, target)
        while dist > max_dist or unit_dist > 10 ** (-5):
            Jinv = np.linalg.pinv(self.jacobian(qi, target))
            qi = (qi + dt * Jinv * kp * error).evalf()
            dist, unit_dist, error = self.compute_error(qi, target)

        return [(a % (2 * sp.pi) - 2 * sp.pi) if (a % (2 * sp.pi)) > sp.pi else sp.deg(a) for a in qi]

    def get_links_position(self, deg=True):
        if deg:
            return [link.get_position() for link in self.links]
        else:
            return [np.deg2rad(link.get_position()) for link in self.links]
