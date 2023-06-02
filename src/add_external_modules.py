import sys, subprocess

EXTERNAL_MODULES = r'C:\fusion\py39_fusion\Lib\site-packages'
BAT = r'C:\Workspace\simulator\external_modules.bat'

if EXTERNAL_MODULES not in sys.path:
    subprocess.call(BAT, shell=True)
    sys.path.append(EXTERNAL_MODULES)

from .robotic_cell import *