from typing import List
from asyncio import gather, run

from .robot import Robot
from .kinematics import *

SPEED = 10.0


class RoboticCell:
    def __init__(self, assembly, *constrains):
        self.robots: List[Robot] = [Robot(assembly.get_components()[index], constrains[index])
                                    for index in range(len(constrains))]

    def home(self, speed: float = SPEED) -> None:
        home_positions = [rbt.get_home_positions() for rbt in self.robots]
        self.drive(home_positions, speed, home=True)

    def drive(self, targets: List[List[float]], speed: float, home: bool = False):
        def synchronize_robots_speed() -> List[float]:
            drive_time = max(rbt.get_drive_time(tar, speed) for rbt, tar in zip(self.robots, targets))
            ranges = [max(rbt.get_drive_ranges(tar)) for rbt, tar in zip(self.robots, targets)]
            return [rng / drive_time if rng != 0 else 0 for rng in ranges]

        async def async_drive():
            tasks = [rbt.drive(tar, spd, home) for rbt, tar, spd in zip(self.robots, targets, speeds)]
            await gather(*tasks)

        speeds = synchronize_robots_speed()
        run(async_drive())

    def get_position(self):
        return [rbt.get_positions() for rbt in self.robots]

    def set_random_position(self, speed=SPEED):
        targets = [rbt.get_random_positions() for rbt in self.robots]
        self.drive(targets, speed)

    def process_trajectory(self, trajectories, orientation, speed=SPEED):
        pass
        # for tr in trajectories:
        #     print(f'{tr}\n')
