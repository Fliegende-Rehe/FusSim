import traceback

from .utilities.add_external_modules import *

from model.robotic_cell import *

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
SPEED = 0.1


def run(context) -> None:
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME)
        assembly = fusion.get_assembly()
        robot_cell = RoboticCell(assembly, ABB_IRB2600)

        part = Part(assembly, TOLERANCE)
        trajectory = part.trajectories[0].points

        for target in trajectory:
            thetas = robot_cell.robots[0].kinematics.inverse_kinematics(target + [0.0, np.pi / 2, np.pi])
            robot_cell.drive([thetas], SPEED)

        robot_cell.launch(SPEED)

        fusion_exit()
    except:
        print(f'Error\n{traceback.format_exc()}')
