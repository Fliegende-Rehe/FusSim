import sys
import traceback
from datetime import datetime
from time import sleep

import adsk.core
import adsk.fusion

EXTERNAL_MODULES_PATH = 'C:\\fusion\\py39_fusion\\Lib\\site-packages'

APP = adsk.core.Application.get()
DESIGN = adsk.fusion.Design
UI = APP.userInterface

DELAY_TO_OPEN_FILE = 2

class Fusion:
    def __init__(self, project_name, file_name):
        self.project_name = project_name
        self.file_name = file_name
        self.assembly = self.set_assembly()
        get_external_modules()

    def get_assembly(self):
        return self.assembly

    def set_assembly(self):
        if self.file_name not in APP.activeDocument.name:
            self.open_assembly()
        logger('\n\nNew session logs:', False)
        root = DESIGN.cast(APP.activeProduct).rootComponent
        return [occ.component for occ in root.occurrences.asList]

    def open_assembly(self):
        try:
            target_proj = next((project for project in APP.data.dataProjects
                                if project.name == self.project_name)
                               )

            target_file = next((file for file in target_proj.rootFolder.dataFiles
                                if file.name == self.file_name)
                               )

            APP.documents.open(target_file, True)
        except:
            kill(self.project_name, self.file_name)

        sleep(DELAY_TO_OPEN_FILE)
        logger('Script is ready', False)


def get_external_modules():
    if EXTERNAL_MODULES_PATH not in sys.path:
        sys.path.append(EXTERNAL_MODULES_PATH)


def kill(*objs_to_check):
    if objs_to_check:
        messenger(f"Check {objs_to_check}")
    logger('Terminate session')
    try:
        sys.exit('Session is terminated by kill')
    except:
        traceback.print_exc(file=sys.stdout)


def refresh():
    adsk.doEvents()
    adsk.doEvents()
    APP.activeViewport.refresh()


def messenger(msg, error=True):
    msg = msg if isinstance(msg, str) else str(msg)
    title = 'Error' if error else 'Log'
    UI.messageBox(f'{title}: {msg}')


def logger(msg, time_tag=True):
    msg = msg if isinstance(msg, str) else str(msg)
    msg = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}' if time_tag else msg
    APP.log(msg)
