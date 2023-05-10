# fusion setup
from sys import path

EXTERNAL_MODULES_PATH = 'C:\\fusion\\py39_fusion\\Lib\\site-packages'

if EXTERNAL_MODULES_PATH not in path:
    path.append(EXTERNAL_MODULES_PATH)

# actual code
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
SPEED = 0.25


def run(context) -> None:
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)
        assembly = fusion.get_assembly()
        robot_cell = RoboticCell(assembly, ABB_IRB2600)

        robot = robot_cell.robots[0]
        kinematics = robot.kinematics

        for _ in range(5):
            ang =  robot.get_random_angles()
            forward = kinematics.forward_kinematics(ang)
            inverse = kinematics.inverse_kinematics(forward)
            logger(f'Check: \n'
                   f'target is {rounded(forward)} \n'
                   f'result is {rounded(kinematics.forward_kinematics(inverse))} \n'
                   f'with {rounded(np.rad2deg(inverse))} rad \n'
                   )

            robot_cell.drive([inverse], SPEED)

        fusion_exit()

    except Exception as e:
        messenger(f'Runtime error\n{e}')
