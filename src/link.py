from math import radians, degrees
from random import uniform

from .fusion import *


class Link:
    def __init__(self, joint, constraints):
        self.joint = joint
        self.joint.isLocked = True

        self.dh = constraints['dh']
        self.defined_home = self.dh[0]
        self.defined_min, self.defined_max = constraints['limits']

        self.actual_home = constraints['home']
        self.actual_min = self.actual_home - abs(self.defined_min)
        self.actual_max = self.actual_home + abs(self.defined_max)

        self.set_link(True)

    def get_actual_home_positions(self) -> float:
        return self.actual_home

    def get_defined_home_positions(self) -> float:
        return self.defined_home

    def set_link(self, enable: bool = True) -> None:
        limits = self.joint.jointMotion.rotationLimits
        limits.isMinimumValueEnabled = enable
        limits.minimumValue = radians(self.actual_min)
        limits.isMaximumValueEnabled = enable
        limits.maximumValue = radians(self.actual_max)
        self.joint.angle.expression = str(self.actual_home)

    def fit_limits(self, angle: float) -> bool:
        if self.defined_min <= angle <= self.defined_max:
            return True
        messenger(
            f'Angle in beyond limits for {self.joint.name}\n'
            f'angle: {angle}\n'
            f'limit: ({self.defined_min}, {self.defined_max})\n'
        )
        return False

    def get_position(self) -> float:
        return degrees(self.joint.jointMotion.rotationValue)

    async def set_position(self, target_angle: float) -> None:
        self.joint.isLocked = False
        while not self.joint.isLocked:
            self.joint.jointMotion.rotationValue = radians(target_angle + self.actual_home)
            self.joint.isLocked = True

    def get_random_position(self) -> float:
        return uniform(self.defined_min, self.defined_max)
