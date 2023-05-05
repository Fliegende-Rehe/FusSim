from resources.ik.hr_matrix import *


class RoboticArm:
    def __init__(self, k_arm, t_arm):
        '''
        Creates a robotic arm class for computing position and velocity.

        Input
        :param k_arm: A 2D array that lists the different axes of rotation (rows) for each joint.
        :param t_arm: A 2D array that lists the translations from the previous joint to the current joint
                      The first translation is from the global (base) frame to joint 1 (which is often equal to the global frame)
                      The second translation is from joint 1 to joint 2, etc.
        '''
        self.k = np.array(k_arm)
        self.t = np.array(t_arm)
        assert k_arm.shape == t_arm.shape, 'Warning! Improper definition of rotation axes and translations'
        self.N_joints = k_arm.shape[0]

    def position(self, Q, index=-1, p_i=[0, 0, 0]):
        '''
        Compute the position in the global (base) frame of a point given in a joint frame
        (default values will assume the input position vector is in the frame of the last joint)
        Input
        :param p_i: A 3 element vector containing a position in the frame of the index joint
        :param index: The index of the joint frame being converted from (first joint is 0, the last joint is N_joints - 1)

        Output
        :return: A 3 element vector containing the new position with respect to the global (base) frame
        '''
        # The position of this joint described by the index
        p_i_x = p_i[0]
        p_i_y = p_i[1]
        p_i_z = p_i[2]
        this_joint_position = np.array([[p_i_x],
                                        [p_i_y],
                                        [p_i_z],
                                        [1]])

        # End effector joint
        if (index == -1):
            index = self.N_joints - 1

        # Store the original index of this joint
        orig_joint_index = index

        # Store the result of matrix multiplication
        running_multiplication = None

        # Start from the index of this joint and work backwards to index 0
        while (index >= 0):

            # If we are at the original joint index
            if (index == orig_joint_index):
                running_multiplication = hr_matrix(self.k[index], self.t[index], Q[index]) @ this_joint_position
            # If we are not at the original joint index
            else:
                running_multiplication = hr_matrix(self.k[index], self.t[index], Q[index]) @ running_multiplication

            index = index - 1

        # extract the points
        px = running_multiplication[0][0]
        py = running_multiplication[1][0]
        pz = running_multiplication[2][0]

        position_global_frame = np.array([px, py, pz])

        return position_global_frame

    def pseudo_inverse(self, theta_start, p_eff_N, goal_position, max_steps=np.inf):
        '''
        Performs the inverse kinematics using the pseudoinverse of the Jacobian

        :param theta_start: An N element array containing the current joint angles in radians (e.g. np.array([np.pi/8,np.pi/4,np.pi/6]))
        :param p_eff_N: A 3 element vector containing translation from the last joint to the end effector in the last joints frame of reference
        :param goal_position: A 3 element vector containing the desired end position for the end effector in the global (base) frame
        :param max_steps: (Optional) Maximum number of iterations to compute

        Output
        :return: An N element vector containing the joint angles that result in the end effector reaching xend (i.e. the goal)
        '''
        v_step_size = 0.05
        theta_max_step = 0.2
        Q_j = theta_start  # Array containing the starting joint angles
        p_end = np.array([goal_position[0], goal_position[1],
                          goal_position[2]])  # desired x, y, z coordinate of the end effector in the base frame
        p_j = self.position(Q_j,
                            p_i=p_eff_N)  # x, y, z coordinate of the position of the end effector in the global reference frame
        delta_p = p_end - p_j  # delta_x, delta_y, delta_z between start position and desired final position of end effector
        j = 0  # Initialize the counter variable

        # While the magnitude of the delta_p vector is greater than 0.01,
        # and we are less than the max number of steps
        while np.linalg.norm(delta_p) > 0.01 and j < max_steps:

            # Reduce the delta_p 3-element delta_p vector by some scaling factor
            # delta_p represents the distance between where the end effector is now and our goal position.
            v_p = delta_p * v_step_size / np.linalg.norm(delta_p)

            # Get the jacobian matrix given the current joint angles
            J_j = self.jacobian(Q_j, p_eff_N)

            # Calculate the pseudo-inverse of the Jacobian matrix
            J_invj = np.linalg.pinv(J_j)

            # Multiply the two matrices together
            v_Q = np.matmul(J_invj, v_p)

            # Move the joints to new angles
            # We use the np.clip method here so that the joint doesn't move too much. We
            # just want the joints to move a tiny amount at each time step because
            # the full motion of the end effector is nonlinear, and we're approximating the
            # big nonlinear motion of the end effector as a bunch of tiny linear motions.
            Q_j = Q_j + np.clip(v_Q, -1 * theta_max_step, theta_max_step)  # [:self.N_joints]

            # Get the current position of the end-effector in the global frame
            p_j = self.position(Q_j, p_i=p_eff_N)

            # Increment the time step
            j = j + 1

            # Determine the difference between the new position and the desired end position
            delta_p = p_end - p_j

        # Return the final angles for each joint
        return Q_j

    def jacobian(self, Q, p_eff_N=[0, 0, 0]):
        '''
        Computes the Jacobian (just the position, not the orientation)

        :param Q: An N element array containing the current joint angles in radians
        :param p_eff_N: A 3 element vector containing translation from the last joint to the end effector in the last joints frame of reference

        Output
        :return: A 3xN 2D matrix containing the Jacobian matrix
        '''
        # Position of the end effector in global frame
        p_eff = self.position(Q, -1, p_eff_N)

        first_iter = True

        jacobian_matrix = None

        for i in range(0, self.N_joints):
            if (first_iter == True):

                # Difference in the position of the end effector in the global frame
                # and this joint in the global frame
                p_eff_minus_this_p = p_eff - self.position(Q, index=i)

                # Axes
                kx = self.k[i][0]
                ky = self.k[i][1]
                kz = self.k[i][2]
                k = np.array([kx, ky, kz])

                px = p_eff_minus_this_p[0]
                py = p_eff_minus_this_p[1]
                pz = p_eff_minus_this_p[2]
                p_eff_minus_this_p = np.array([px, py, pz])

                this_jacobian = np.cross(k, p_eff_minus_this_p)

                # Convert to a 2D matrix
                j0 = this_jacobian[0]
                j1 = this_jacobian[1]
                j2 = this_jacobian[2]
                this_jacobian = np.array([[j0],
                                          [j1],
                                          [j2]])
                jacobian_matrix = this_jacobian
                first_iter = False
            else:
                p_eff_minus_this_p = p_eff - self.position(Q, index=i)

                # Axes
                kx = self.k[i][0]
                ky = self.k[i][1]
                kz = self.k[i][2]
                k = np.array([kx, ky, kz])

                # Difference between this joint's position and end effector's position
                px = p_eff_minus_this_p[0]
                py = p_eff_minus_this_p[1]
                pz = p_eff_minus_this_p[2]
                p_eff_minus_this_p = np.array([px, py, pz])

                this_jacobian = np.cross(k, p_eff_minus_this_p)

                # Convert to a 2D matrix
                j0 = this_jacobian[0]
                j1 = this_jacobian[1]
                j2 = this_jacobian[2]
                this_jacobian = np.array([[j0],
                                          [j1],
                                          [j2]])
                jacobian_matrix = np.concatenate((jacobian_matrix, this_jacobian), axis=1)  # side by side

        return jacobian_matrix
