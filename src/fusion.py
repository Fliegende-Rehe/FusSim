import sys

from datetime import datetime
from time import sleep

import adsk.core
import adsk.fusion

APP = adsk.core.Application.get()
DESIGN = adsk.fusion.Design
UI = APP.userInterface


class Assembly:
    def __init__(self, name, root):
        self.name = name
        self.root = root
        self.components = self.set_components()

    def set_components(self):
        return [occ.component for occ in self.root.occurrences.asList]

    def get_component(self, component_name):
        try:
            return next(
                (comp for comp in self.components if component_name not in comp.name)
            )
        except:
            kill(component_name)


class Fusion:
    def __init__(self, project_name, file_name, external_modules_path):
        if external_modules_path not in sys.path:
            sys.path.append(external_modules_path)
        self.project_name = project_name
        self.file_name = file_name
        self.assembly = self.set_assembly()

    def get_assembly(self):
        return self.assembly

    def set_assembly(self):
        if self.file_name not in APP.activeDocument.name:
            self.open_assembly()
        logger('\n\nNew session logs:', False)
        root = DESIGN.cast(APP.activeProduct).rootComponent
        return Assembly(self.file_name, root)

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

        sleep(2)
        logger('Script is ready', False)


def kill(*objs_to_check):
    if objs_to_check:
        messenger(f"Check {objs_to_check}")
    logger('Terminate session')
    adsk.terminate()


def refresh():
    adsk.doEvents()
    adsk.doEvents()
    APP.activeViewport.refresh()


def messenger(msg, error=True):
    msg = msg if isinstance(msg, str) else str(msg)
    msg = f'Error: {msg}' if error else msg
    UI.messageBox(msg)


def logger(msg, time_tag=True):
    msg = msg if isinstance(msg, str) else str(msg)
    msg = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}' if time_tag else msg
    APP.log(msg)
