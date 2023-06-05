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
            speeds = synchronize_robots_speed()
            tasks = [rbt.drive(tar, spd, home) for rbt, tar, spd in zip(self.robots, targets, speeds)]
            await gather(*[create_task(task) for task in tasks])

        run(async_drive())

    def suppress_noises(self, ratio=4):
        def calculate_diff(row, col):
            return abs(self.position_chain[row][col] - self.position_chain[row + 1][col])

        def update_wrong_values(wrong_index, row, col):
            st = self.position_chain[wrong_index - 1][col]
            fn = self.position_chain[row][col]
            diff = (fn - st) / (row - wrong_index + 1)
            for index in range(1, row - wrong_index + 1):
                suppressed_value = self.position_chain[row - index][col]
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
                avg_diff = sum(col_diff) / len(col_diff)
                if upper_row_diff > avg_diff * ratio and lower_row_diff > avg_diff * ratio:
                    if wrong_index is None:
                        wrong_index = row
                else:
                    if wrong_index is not None:
                        update_wrong_values(wrong_index, row, col)
                    col_diff.append(upper_row_diff)
                    wrong_index = None

    def calculate_position_chain(self, target, orientation):
        robot = self.robots[0]

        theta = robot.get_links_position()
        for tar in target:
            position = robot.kinematics.inverse_kinematics(theta, tar + orientation)
            self.position_chain.append(position)
            theta = position

        # self.position_chain = [
        #     robot.kinematics.inverse_kinematics(theta, target[index] + orientation)
        #     for index in range(len(target))
        # ]

        # self.suppress_noises()

        for p in self.position_chain:
            logger(rounded(np.rad2deg(p)), False)

    def process_position_chain(self, speed):
        for position in self.position_chain:
            self.drive([position], speed)
        logger(f'The trajectory is done')
