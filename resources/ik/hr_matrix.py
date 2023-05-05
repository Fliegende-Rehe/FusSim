from resources.ik.axis_angle_rot_matrix import *


def hr_matrix(k, t, q):
    '''
    Create the Homogenous Representation matrix that transforms a point from Frame B to Frame A.
    Using the axis-angle representation
    Input
    :param k: A 3 element array containing the unit axis to rotate around (kx,ky,kz)
    :param t: The translation from the current frame (e.g. Frame A) to the next frame (e.g. Frame B)
    :param q: The rotation angle (i.e. joint angle)

    Output
    :return: A 4x4 Homogenous representation matrix
    '''
    # Calculate the rotation matrix (angle-axis representation)
    rot_matrix_A_B = axis_angle_rot_matrix(k, q)

    # Store the translation vector t
    translation_vec_A_B = t

    # Convert to a 2D matrix
    t0 = translation_vec_A_B[0]
    t1 = translation_vec_A_B[1]
    t2 = translation_vec_A_B[2]
    translation_vec_A_B = np.array([[t0],
                                    [t1],
                                    [t2]])

    # Create the homogeneous transformation matrix
    homgen_mat = np.concatenate((rot_matrix_A_B, translation_vec_A_B), axis=1)  # side by side

    # Row vector for bottom of homogeneous transformation matrix
    extra_row_homgen = np.array([[0, 0, 0, 1]])

    # Add extra row to homogeneous transformation matrix
    homgen_mat = np.concatenate((homgen_mat, extra_row_homgen), axis=0)  # one above the other

    return homgen_mat
