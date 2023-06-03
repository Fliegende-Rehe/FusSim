import numpy as np
from scipy import stats
from asyncio import run

from .robot import *
from .part import *

SPEED = 0.1


class RoboticCell:
    def __init__(self, assembly: Assembly, *constrains) -> None:
        self.robots = [Robot(body, param) for body, param in zip(assembly.get_components(), constrains)]
        self.position_chain = []

    def launch(self, speed: float = SPEED) -> None:
        home_positions = [[0.0] * len(rbt.links) for rbt in self.robots]
        self.drive(home_positions, speed, home=True)

    def random_position(self, speed: float = SPEED) -> None:
        targets = [rbt.get_random_angles() for rbt in self.robots]
        self.drive(targets, speed)

    def drive(self, targets: list[list[float]], speed: float, home: bool = False):
        def synchronize_robots_speed() -> list[float]:
            drive_time = max(rbt.get_drive_time(tar, speed) for rbt, tar in zip(self.robots, targets))
            ranges = [max(rbt.get_drive_ranges(tar)) for rbt, tar in zip(self.robots, targets)]
            return [rng / drive_time if rng != 0 else 0 for rng in ranges]

        async def async_drive() -> None:
            tasks = [rbt.drive(tar, spd, home) for rbt, tar, spd in zip(self.robots, targets, speeds)]
            await gather(*tasks)

        speeds = synchronize_robots_speed()

        run(async_drive())

    def supress_noises(self, ratio=4):
        for col in range(len(self.position_chain[0])):
            col_diff = []
            for row in range(len(self.position_chain) - 1):
                upper_row_diff = abs(self.position_chain[row][col] - self.position_chain[row + 1][col])
                if row == 0:
                    col_diff.append(upper_row_diff)
                    continue

                lower_row_diff = abs(self.position_chain[row - 1][col] - self.position_chain[row][col])
                avg_diff = sum(col_diff) / len(col_diff)
                if upper_row_diff > avg_diff * ratio and lower_row_diff > avg_diff * ratio:
                    logger(
                        f'[{row}][{col}]'
                        f' = {np.round(np.rad2deg(self.position_chain[row][col]), decimals=2)}',
                        False
                    )
                    self.position_chain[row][col] = self.position_chain[row - 1][col] + avg_diff
                else:
                    col_diff.append(upper_row_diff)

    def calculate_position_chain(self, target, orientation):
        for point in target:
            inverse = self.robots[0].kinematics.inverse_kinematics(point + orientation)
            self.position_chain.append(inverse)

        self.supress_noises()

        self.print_position_chain()

    def process_position_chain(self, speed):
        self.drive([self.position_chain], speed)

    def print_position_chain(self):
        logger('\n', False)
        for i, position in enumerate(self.position_chain, 0):
            logger(f'{i}) {rounded(np.rad2deg(position))}', False)
        logger('\n', False)
