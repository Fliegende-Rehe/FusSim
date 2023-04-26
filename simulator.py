from .src.robotic_cell import *
from .src.part import *
from .src.fusion import *

import traceback

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'

ABB_IRB2600 = [
    {'limits': [-168, 168], 'length': 215, 'home_position': 0, 'rotation_axis': 'x'},
    {'limits': [-120, 120], 'length': 700, 'home_position': 0, 'rotation_axis': 'x'},
    {'limits': [-180, 65], 'length': 287, 'home_position': 0, 'rotation_axis': 'x'},
    {'limits': [-180, 180], 'length': 508, 'home_position': 0, 'rotation_axis': 'x'},
    {'limits': [-125, 125], 'length': 0, 'home_position': 0, 'rotation_axis': 'x'},
    {'limits': [-180, 180], 'length': 600, 'home_position': -90, 'rotation_axis': 'x'}
]

KP3_V2H500_2 = [
    {'limits': [-180, 180], 'length': 1500, 'home_position': 0, 'rotation_axis': 'z'},
    {'limits': [-180, 180], 'length': 800, 'home_position': -90, 'rotation_axis': 'y'},
    {'limits': [-180, 180], 'length': 800, 'home_position': 0, 'rotation_axis': 'y'},
]

TOLERANCE = 0.05
SPEED = 12.0


def run(context):
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)

        assembly = fusion.get_assembly()

        robot_cell = RoboticCell(assembly, ABB_IRB2600, KP3_V2H500_2)
        robot_cell.launch()

        part = Part(assembly)
        trajectories = part.get_trajectories(TOLERANCE)

        # robot_cell.process_trajectories(trajectories, SPEED)

        kill()

    except:
        messenger(f'Error in run\n{traceback.format_exc()}')
