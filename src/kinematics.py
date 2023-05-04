from numpy.linalg import linalg
from sympy import symbols, zeros, diff


from .matrix_utils import *
from .link import *


class Kinematics:

    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.dh_param = [lnk.dh for lnk in self.links]

    def get_links_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def forward_kinematics(self, thetas=None, deg=True) -> list[float]:
        if thetas is None:
            thetas = self.get_links_position()
        transformation = full_transformation(self.dh_param, thetas)
        px, py, pz = sp.symbols('px py pz')
        px, py, pz = normalize_row(transformation[:3, 3])
        ox, oy, oz = sp.symbols('ox oy oz')
        ox, oy, oz = rot2eul(transformation[:3, :3], deg)
        return px, py, pz, ox, oy, oz

    def inverse_kinematics(self, position, orientation, max_iter=100, tol=0.01) -> list[float]:
        px, py, pz = position
        ox, oy, oz = orientation
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        self.forward_kinematics()

        def jacobian(theta):
            # Calculate the Jacobian matrix using the current joint angles
            J = zeros(6, 6)
            for i in range(6):
                for j in range(6):
                    J[i, j] = diff(self.forward_kinematics(theta, False)[i], theta[j])
            return J

        # Define the initial joint angles
        theta = array(self.get_links_position())
        # J = jacobian(theta)
        # print(J)
        # # Calculate the error between the current end-effector pose and the target pose
        # for i in range(max_iter):
        #     ee = self.forward_kinematics(theta, False)
        #     error = array([px, py, pz, ox, oy, oz]) - array(ee)
        #
        #     # Check if the error is below the tolerance
        #     if linalg.norm(error) < tol:
        #         print("Inverse kinematics converged after %d iterations" % (i + 1))
        #         break
        #
        #     # Calculate the Jacobian matrix using the current joint angles
        #     J = jacobian(theta)
        #
        #     # Calculate the joint angle increments using the pseudoinverse of the Jacobian matrix
        #     dtheta = linalg.pinv(J) @ error
        #
        #     # Update the joint angles
        #     theta += dtheta
        return []
