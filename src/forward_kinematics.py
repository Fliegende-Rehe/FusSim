from .transformation_matrix import *


# Forward kinematics solver function
def FK_solve(q):
    # Calculate the transformation matrices for each joint
    A = [T1(q[0]), T2(q[1]), T3(q[2]), T4(q[3]), T5(q[4]), T6(q[5])]

    # Function to multiply all the transformation matrices to get the final end effector position
    def f(A):
        ret = np.eye(4)
        for a in A:
            ret = ret @ a
        return ret

    return f(A)


# Function to transform the end effector position from base frame to a new frame
def transform_base(trans, q):
    # Calculate the end effector transformation matrix in base frame
    ee_base = FK_solve(q)
    # Transform the end effector position to the new frame
    return trans @ ee_base
