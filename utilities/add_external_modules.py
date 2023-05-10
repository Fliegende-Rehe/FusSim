import sys

EXTERNAL_MODULES_PATH = 'C:\\fusion\\py39_fusion\\Lib\\site-packages'

ROOT_PATH = 'C:\\Workspace\\simulator'

if EXTERNAL_MODULES_PATH not in sys.path:
    sys.path.append(EXTERNAL_MODULES_PATH)

if ROOT_PATH not in sys.path:
    sys.path.append(ROOT_PATH)
