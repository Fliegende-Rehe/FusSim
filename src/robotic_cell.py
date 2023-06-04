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

    def suppress_noises(self, ratio=3):

        def calculate_diff(row, col):
            return abs(self.position_chain[row][col] - self.position_chain[row + 1][col])

        def calculate_avg_diff(col_diff):
            return sum(col_diff) / len(col_diff)

        def update_wrong_values(wrong_index, row, col):
            st = self.position_chain[wrong_index - 1][col]
            fn = self.position_chain[row][col]
            diff = (fn - st) / (row - wrong_index + 1)
            for index in range(1, row - wrong_index + 1):
                self.position_chain[row - index][col] = fn - diff * index

        for col in range(len(self.position_chain[0])):
            col_diff = []
            wrong_index = None
            for row in range(len(self.position_chain) - 1):
                upper_row_diff = calculate_diff(row, col)
                if row == 0:
                    col_diff.append(upper_row_diff)
                    continue

                lower_row_diff = calculate_diff(row - 1, col)
                avg_diff = calculate_avg_diff(col_diff)
                if upper_row_diff > avg_diff * ratio and lower_row_diff > avg_diff * ratio:
                    if wrong_index is None:
                        wrong_index = row
                else:
                    if wrong_index is not None:
                        update_wrong_values(wrong_index, row, col)
                    col_diff.append(upper_row_diff)
                    wrong_index = None

    def calculate_position_chain(self, target, orientation):
        for point in target:
            inverse = self.robots[0].kinematics.inverse_kinematics(point + orientation)
            self.position_chain.append(inverse)

        self.suppress_noises()

    def process_position_chain(self, speed):
        for position in self.position_chain:
            self.drive([position], speed)
        logger(f'The trajectory is done')
