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

    def calculate_position_chain(self, target, orientation):
        for point in target:
            inverse = self.robots[0].kinematics.inverse_kinematics(point + orientation)
            self.position_chain.append(inverse)

        # self.print_position_chain()
        noise = [[] for _ in range(len(self.position_chain[0]))]
        for col in range(len(self.position_chain[0])):
            avg_diff = 0
            for row in range(1, len(self.position_chain)):
                diff = abs(self.position_chain[row][col] - self.position_chain[row - 1][col])
                if row == 1:
                    avg_diff = diff
                    continue
                if diff > avg_diff * 500:
                    noise[col].append(self.position_chain[row][col])
                else:
                    avg_diff += diff
                    avg_diff /= 2
            logger(np.round(np.rad2deg(avg_diff), decimals=2))

        for index in noise:
            logger(np.round(np.rad2deg(index), False))

        arr = rounded(np.rad2deg(np.array(self.position_chain).T[0]))
        logger(sum(arr)/ len(arr))
            # col_diff = []
            # for row in range(1, len(self.position_chain)):
            #     diff = abs(self.position_chain[row][col] - self.position_chain[row - 1][col])
            #     col_diff.append(diff)
            # noise_index = significantly_different_value_indexes(col_diff, std_multiplier=1.85)
            # if len(noise_index) == 0:
            #     continue
            # for i in range(0, len(noise_index), 2):
            #     row = int(noise_index[i]) + 1
            #     logger(
            #         f'chain [{row}][{col}]'
            #         f' = {np.round(np.rad2deg(np.array(self.position_chain).T[col][row]), decimals=2)}',
            #         False
            #     )
            #     self.position_chain[row][col] = (self.position_chain[row - 1][col] + self.position_chain[row + 1][
            #         col]) / 2
            # logger('\n', False)

        self.print_position_chain()

    def process_position_chain(self, speed):
        self.drive([self.position_chain], speed)

    def print_position_chain(self):
        for i, position in enumerate(self.position_chain, 0):
            logger(f'{i}) {rounded(np.rad2deg(position))}', False)
        logger('\n', False)


def significantly_different_value_indexes(arr, std_multiplier):
    data = np.array(arr)
    mean = np.mean(data)
    std_dev = np.std(data)
    return [arr.index(x) for x in data if abs(x - mean) > std_multiplier * std_dev]  # arr.index(x)
