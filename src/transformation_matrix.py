from .matrix_utils import *

l = [1, 1, 1, 1, 1, 1]  # Define the link lengths of the robot arm


# Define transformation matrices for each joint
def T1(q1):
    """
    Calculates the transformation matrix for joint 1 based on the joint angle q1 and link length l[0]
    """
    return Rz(q1) @ Tz(l[0])


def T2(q2):
    """
    Calculates the transformation matrix for joint 2 based on the joint angle q2 and link length l[1]
    """
    return Rx(q2) @ Tz(l[1])


def T3(q3):
    """
    Calculates the transformation matrix for joint 3 based on the joint angle q3 and link length l[2]
    """
    return Rx(q3) @ Tz(l[2])


def T4(q4):
    """
    Calculates the transformation matrix for joint 4 based on the joint angle q4 and link length l[3]
    """
    return Tz(l[3]) @ Rz(q4)


def T5(q5):
    """
    Calculates the transformation matrix for joint 5 based on the joint angle q5
    """
    return Ry(q5)


def T6(q6):
    """
    Calculates the transformation matrix for joint 6 based on the joint angle q6 and link lengths l[4] and l[5]
    """
    return Rz(q6) @ Tz(l[4]) @ Tz(l[5])
