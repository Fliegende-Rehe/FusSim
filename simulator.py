# fusion setup
from sys import path

EXTERNAL_MODULES_PATH = 'C:\\fusion\\py39_fusion\\Lib\\site-packages'

if EXTERNAL_MODULES_PATH not in path:
    path.append(EXTERNAL_MODULES_PATH)

# actual code
import traceback

from .src.robotic_cell import *

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

ABB_IRB2600 = {
    'dh_table': {
        'theta': [0, 90, 0, 0, 0, 0],
        'length': [150, 700, 115, 0, 0, -53.611],
        'alpha': [90, 0, 90, -90, 90, 0],
        'offset': [445, 0, 0, 795, 0, 595.144]
    },
    'links_param': [
        [-165, 165, 90],
        [-90, 90],
        [-75, 180, 90, -1],
        [-180, 180],
        [-120, 120],
        [-180, 180, -90]
    ],
}

TOLERANCE = 1
SPEED = 10


def run(context) -> None:
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)
        assembly = fusion.get_assembly()
        robot_cell = RoboticCell(assembly, ABB_IRB2600)

        kinematics = robot_cell.robots[0].kinematics

        ang = [-50] * 6
        forward = kinematics.forward_kinematics(ang)
        logger(f'Forward kinematics: \n'
              f'input {ang} deg \n'
              f'output {rounded(forward)} \n')

        inverse = kinematics.inverse_kinematics(forward)
        logger(f'Inverse kinematics: \n'
              f'input {rounded(forward)} \n'
              f'output {rounded(inverse)} deg \n')

        logger(f'Check: \n'
              f'IK = {rounded(inverse)} deg \n'
              f'FK = {rounded(kinematics.forward_kinematics(inverse))} \n')

        robot_cell.drive([inverse], SPEED)

    except:
        messenger(f'Runtime error\n{traceback.format_exc()}')
