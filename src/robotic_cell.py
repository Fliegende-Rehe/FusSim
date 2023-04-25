from .robot import *


class RoboticCell:
    def __init__(self, assembly, *constrains):
        self.robots = [Robot(body, limits) for body, limits in zip(assembly, constrains)]

    def drive(self, target):
        pass

    def set_random_position(self):
        pass

    def launch(self):
        for rbt in self.robots:
            rbt.launch()
