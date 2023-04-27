from .src.robotic_cell import *
from .src.part import *
from .src.fusion import *

import traceback

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

ABB_IRB2600 = [
    {'limits': [-168, 168], 'length': 445, 'home_position': 0},
    {'limits': [-120, 120], 'length': 115, 'home_position': 0},
    {'limits': [-180, 75], 'length': 700, 'home_position': 0},
    {'limits': [-180, 180], 'length': 115, 'home_position': 0},
    {'limits': [-125, 125], 'length': 795, 'home_position': 0},
    {'limits': [-180, 180], 'length': 540, 'home_position': -90}
]

KP3_V2H500_2 = [
    {'limits': [-180, 180], 'length': 1500, 'home_position': 0},
    {'limits': [-180, 180], 'length': 800, 'home_position': -90},
    {'limits': [-180, 180], 'length': 800, 'home_position': 0},
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
        messenger(f'Error in run\n{traceback.format_exc()}')
