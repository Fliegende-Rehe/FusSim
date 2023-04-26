from .fusion import *


class Assembly:
    def __init__(self, component_list):
        self.components = component_list

    def get_components(self):
        return self.components
    def get_component_by_name(self, component_name):
        try:
            return next(
                (comp for comp in self.components if component_name not in comp.name)
            )
        except:
            kill(component_name)
