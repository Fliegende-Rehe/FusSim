from .src.robotic_cell import *
from .src.part import *
from .src.fusion import *

import traceback

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

ABB_IRB2600 = [
    {'limits': [-168, 168], 'origin': [0, 0, 445], 'length': 455, 'home_position': 0},
    {'limits': [-120, 120], 'origin': [150, 0, 0], 'length': 700,  'home_position': 0},
    {'limits': [-180, 75], 'origin':  [0, 0, 700], 'length': 115,  'home_position': 0},
    {'limits': [-180, 180], 'origin': [0, 0, 115], 'length': 795,  'home_position': 0},
    {'limits': [-125, 125], 'origin': [795, 0, 0], 'length': 0,  'home_position': 0},
    {'limits': [-180, 180], 'origin': [558, 0, 0], 'length': 558,  'home_position': 0}
]

KP3_V2H500_2 = [
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'length': 455,  'home_position': 0},
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'length': 455,  'home_position': -90},
    {'limits': [-180, 180], 'origin': [0, 0, 0], 'length': 455,  'home_position': 0},
]

TOLERANCE = 1
SPEED = 10.0


def run(context):
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)

        assembly = fusion.get_assembly()

        robot_cell = RoboticCell(assembly, ABB_IRB2600, KP3_V2H500_2)
        robot_cell.launch()

        part = Part(assembly)
        trajectory = part.get_trajectory(TOLERANCE)

        robot_cell.process_trajectory(trajectory, SPEED)

        kill()

    except:
        print(f'Error in run\n{traceback.format_exc()}')
