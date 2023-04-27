from math import radians, degrees
from random import uniform

from src.fusion import *


class Link:
    def __init__(self, joint, constraints):
        self.joint = joint
        self.joint.isLocked = True
        self.min, self.max = constraints['limits']
        self.length = constraints['length']
        self.home = constraints['home_position']
        self.rotation_axis = constraints['rotation_axis']

        self.set_limits()

    def set_limits(self) -> None:
        lim = self.joint.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = radians(self.min)
        lim.isMaximumValueEnabled = True
        lim.maximumValue = radians(self.max)

    def fit_limits(self, angle: float) -> bool:
        if self.min <= angle <= self.max:
            return True
        messenger(
            f'Error in beyond_bounds for {self.joint.name}\nangle: {angle}\nlimit: ({self.min}, {self.max})'
        )
        return False

    def get_position(self) -> float:
        return degrees(self.joint.jointMotion.rotationValue)

    async def set_home(self) -> None:
        await self.async_set_position(self.home)

    async def async_set_position(self, target_angle: float) -> None:
        self.joint.isLocked = False
        while not self.joint.isLocked:
            self.joint.jointMotion.rotationValue = radians(target_angle)
            self.joint.isLocked = True

    def get_random_position(self) -> float:
        return uniform(self.min, self.max)
