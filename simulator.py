import traceback
from itertools import combinations_with_replacement

from .src.robotic_cell import *

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

'''
    θ: Angle about the previous Z, from old X to new X.
    alpha: Angle about new X, from old Z-axis to new Z-axis.
    d: Offset distance along the previous Z, from old joint to new joint center.
    a: Length along new X, from old joint to new joint center.
    In addition to that while placing the local coordinates in Figure 2 following rules apply;
    — Z-axis is always the joint axis on which the joint rotates about if it is a rotational joint or moves along if it is a
    translational joint.
    — X-axis must be perpendicular and intersect both new Z & old Z
    — Y-axis’s placement follows the right-hand rule based on the X & Z axes
'''

ABB_IRB2600 = [
    {'limits': [-180, 180], 'dh': [0, 90, 445, 150], 'home': 0},
    {'limits': [-95, 155], 'dh': [90, 0, 0, 700], 'home': 0},
    {'limits': [-180, 65], 'dh': [0, -90, 0, 61.389], 'home': 90},
    {'limits': [-180, 180], 'dh': [0, 90, 795, 0], 'home': 0},
    {'limits': [-120, 120], 'dh': [90, 0, 0, 0], 'home': 0},
    {'limits': [-180, 180], 'dh': [0, -90, 0, 592.144], 'home': 90}
]

KP3_V2H500_2 = [
    {'limits': [-180, 180], 'dh': [0, 90, 962.582, 1272.045], 'home': 0},
    {'limits': [-180, 180], 'dh': [0, 0, 0, 0], 'home': 0},
    {'limits': [-180, 180], 'dh': [0, 0, 0, 0], 'home': 0},
]

TOLERANCE = 1
SPEED = 10


def run(context) -> None:
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)

        assembly = fusion.get_assembly()

        robot_cell = RoboticCell(assembly, ABB_IRB2600, KP3_V2H500_2)

        # robot_cell.set_random_position(SPEED)

        robot = robot_cell.arm
        robot_cell.home()
        ang = robot.get_position()
        print(ang)

        print([float("{:f}".format(float(row))) for row in robot.kinematics.forward_kinematics(ang)[:3]])

        fusion_exit(kill=False)

    except:
        messenger(f'Runtime error\n{traceback.format_exc()}')
