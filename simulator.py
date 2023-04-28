import traceback

from .src.robotic_cell import *
from .src.part import *
from .src.fusion import *

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

ABB_IRB2600 = [
    {'limits': [-168, 168], 'origin': [0, 0, 445], 'rotation_axis': [0, 0, 1], 'length': 455, 'home': 0},
    {'limits': [-120, 120], 'origin': [150, 0, 0], 'rotation_axis': [0, 1, 0], 'length': 150, 'home': 0},
    {'limits': [-180, 75], 'origin': [0, 0, 700], 'rotation_axis': [0, 1, 0], 'length': 700, 'home': 0},
    {'limits': [-180, 180], 'origin': [0, 0, 115], 'rotation_axis': [1, 0, 0], 'length': 115, 'home': 0},
    {'limits': [-125, 125], 'origin': [795, 0, 0], 'rotation_axis': [0, 1, 0], 'length': 795, 'home': -90},
    {'limits': [-135, 135], 'origin': [558, 0, 0], 'rotation_axis': [1, 0, 0], 'length': 558, 'home': 0}
]

KP3_V2H500_2 = [
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'rotation_axis': [0, 0, 1], 'length': 455, 'home': 0},
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'rotation_axis': [0, 0, 1], 'length': 455, 'home': 0},
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'rotation_axis': [0, 0, 1], 'length': 455, 'home': 0},
]

TOLERANCE = 0.05
SPEED = 20.0


def run(context):
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)

        assembly = fusion.get_assembly()

        robot_cell = RoboticCell(assembly, ABB_IRB2600, KP3_V2H500_2)

        part = Part(assembly)
        trajectory = part.get_trajectory(TOLERANCE)

        robot_cell.set_random_position(SPEED)

        robot_cell.home(SPEED)

        fusion_exit(kill=False)

    except:
        print(f'Error in run\n{traceback.format_exc()}')
