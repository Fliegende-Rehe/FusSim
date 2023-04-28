from sys import path
from datetime import datetime
from time import sleep
from typing import List

import adsk.core
import adsk.fusion

from .assembly import *

EXTERNAL_MODULES_PATH = 'C:\\fusion\\py39_fusion\\Lib\\site-packages'

APP = adsk.core.Application.get()
DESIGN = adsk.fusion.Design

DELAY_TO_OPEN_FILE = 2


class Fusion:
    def __init__(self, project_name, file_name):
        self.project_name = project_name
        self.file_name = file_name
        self.assembly = self.set_assembly()
        get_external_modules()

    def get_component(self, component_name):
        try:
            return next(
                (comp for comp in self.assembly.get_components() if component_name not in comp.name)
            )
        except:
            fusion_exit(kill=True)
            messenger(f"Check is '{component_name}' exist")

    def get_assembly(self):
        return self.assembly

    def set_assembly(self):
        if self.file_name not in APP.activeDocument.name:
            self.open_assembly()
        logger('\n\nNew session logs:', False)
        root = DESIGN.cast(APP.activeProduct).rootComponent

        return Assembly([occ.component for occ in root.occurrences.asList])

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
            fusion_exit(kill=True)
            messenger(f"Check are '{self.project_name}' or '{self.file_name}' exist")

        sleep(DELAY_TO_OPEN_FILE)
        logger('Script is ready', False)


def get_external_modules():
    if EXTERNAL_MODULES_PATH not in path:
        path.append(EXTERNAL_MODULES_PATH)


def fusion_exit(kill=False):
    logger('Terminate session')
    if kill:
        adsk.terminate()


def refresh_display():
    adsk.doEvents()
    adsk.doEvents()
    APP.activeViewport.refresh()


def messenger(msg, error=True):
    if not isinstance(msg, str):
        msg = str(msg)
    title = 'Error' if error else 'Log'
    print(f'{title}: {msg}')


def logger(msg, time_tag=True):
    msg = msg if isinstance(msg, str) else str(msg)
    msg = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}' if time_tag else msg
    APP.log(msg)


def rounded(array: List[float]):
    return [int(element) for element in array]
