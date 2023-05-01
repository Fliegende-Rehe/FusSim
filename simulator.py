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

ABB_IRB2600 = [
    {'dh': {'z': 0, 'along_z': 445, 'x': 90, 'along_x': 150}, 'limits': [-165, 165], 'home': 90},

    {'dh': {'z': 90, 'along_z': 0, 'x': 0, 'along_x': 700}, 'limits': [-90, 90], 'home': 0},

    {'dh': {'z': 0, 'along_z': 0, 'x': -90, 'along_x': 61.389}, 'limits': [-75, 180], 'home': -90},

    {'dh': {'z': 0, 'along_z': 795, 'x': 90, 'along_x': 0}, 'limits': [-180, 180], 'home': 0},

    {'dh': {'z': 90, 'along_z': 0, 'x': 0, 'along_x': 0}, 'limits': [-120, 120], 'home': 0},

    {'dh': {'z': 0, 'along_z': 0, 'x': -90, 'along_x': 592.144}, 'limits': [-180, 180], 'home': -90},
]

TOLERANCE = 1
SPEED = 10


def run(context) -> None:
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)

        assembly = fusion.get_assembly()

        robot_cell = RoboticCell(assembly, ABB_IRB2600)

        robot_cell.set_random_position(SPEED)

        robot = robot_cell.robots[0]

        ang = robot.get_positions()

        print(ang)

        print([float("{:f}".format(float(row))) for row in robot.kinematics.forward_kinematics(ang)])

        fusion_exit(kill=False)

    except:
        messenger(f'Runtime error\n{traceback.format_exc()}')
