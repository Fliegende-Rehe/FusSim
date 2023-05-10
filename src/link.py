from random import uniform

import numpy as np

from .fusion import *


class Link:
    def __init__(self, joint, constraints):
        self.joint = joint
        self.name = joint.name

        self.min, self.max, = constraints[:2]
        self.home = constraints[2] if len(constraints) > 2 else 0
        self.direction = constraints[3] if len(constraints) > 3 else 1

        self.set_limits(True)

    def set_limits(self, enable: bool = True) -> None:
        self.joint.angle.expression = str(self.home)
        if abs(self.min) == self.max == 180:
            enable = False
        limits = self.joint.jointMotion.rotationLimits
        limits.isMinimumValueEnabled = enable
        limits.minimumValue = np.deg2rad(self.min if self.direction == 1 else -abs(self.max))
        limits.isMaximumValueEnabled = enable
        limits.maximumValue = np.deg2rad(self.max if self.direction == 1 else abs(self.min))
        self.joint.isLocked = True

    def get_limits(self, angle: float) -> bool:
        if self.min <= angle * self.direction <= self.max:
            return True
        messenger(
            f'Angle in beyond limits for {self.joint.name}\n'
            f'angle: {angle}\n'
            f'limit: ({self.min}, {self.max})\n'
        )
        return False

    async def set_position(self, target_angle: float) -> None:
        self.joint.isLocked = False
        while not self.joint.isLocked:
            self.joint.jointMotion.rotationValue = target_angle * self.direction
            self.joint.isLocked = True

    def get_position(self) -> float:
        return self.joint.jointMotion.rotationValue * self.direction

    def get_random_position(self) -> float:
        return np.deg2rad(uniform(self.min, self.max))
