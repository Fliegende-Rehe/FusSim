import traceback
from random import uniform

from .src.robot import Robot
from .src.fusion import *

PROJECT_NAME = 'gets'
FILE_NAME = 'simulation'
EXTERNAL_MODULES_PATH = 'C:/fusion/py39_fusion/Lib/site-packages'

ABB_IRB2600_CONSTRAINS = [
    {'limits': [-168.0, 168.0], 'length': 215.0, 'home': 0.0},
    {'limits': [-120.0, 120.0], 'length': 700.0, 'home': 0.0},
    {'limits': [-180.0, 65.0], 'length': 287.0, 'home': 0.0},
    {'limits': [-180.0, 180.0], 'length': 508.0, 'home': 0.0},
    {'limits': [-125.0, 125.0], 'length': 0.0, 'home': 0.0},
    {'limits': [-180.0, 180.0], 'length': 600, 'home': -90.0},
]
KP3_V2H_CONSTRAINS = [
    {'limits': [0.0, 0.0], 'length': 0.0, 'home': 0.0},
    {'limits': [0.0, 0.0], 'length': 0.0, 'home': 0.0},
    {'limits': [0.0, 0.0], 'length': 0.0, 'home': 0.0},
]


def run(context):
    try:
        fusion = Fusion(PROJECT_NAME, FILE_NAME, EXTERNAL_MODULES_PATH)

        assembly = fusion.get_assembly()

        abb_irb2600 = Robot(
            assembly.get_component('ABB_IRB2600'), ABB_IRB2600_CONSTRAINS
        )

        abb_irb2600.launch()

        abb_irb2600.drive([uniform(lnk.min, lnk.max) for lnk in abb_irb2600.links])

        kill()
    except:
        messenger(f'Error in run\n{traceback.format_exc()}')
