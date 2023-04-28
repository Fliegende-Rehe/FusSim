from .robot import *
from .part import *

SPEED = 10.0


class RoboticCell:
    def __init__(self, assembly: Assembly, *constrains) -> None:
        self.robots: list[Robot] = [Robot(assembly.get_components()[index], constrains[index])
                                    for index in range(len(constrains))]
        self.arm, self.positioner = self.robots

    def home(self, speed: float = SPEED) -> None:
        home_positions = [rbt.get_home_position() for rbt in self.robots]
        self.drive(home_positions, speed, home=True)

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

    def get_position(self) -> list[list[float]]:
        return [rbt.get_position() for rbt in self.robots]

    def set_random_position(self, speed: float = SPEED) -> None:
        targets = [rbt.get_random_position() for rbt in self.robots]
        self.drive(targets, speed)

    def weld_part(self, part: Part, speed: float = SPEED) -> None:
        orientation = [0.2, 0.2, 0.2]
        trajectories = part.trajectories
        start_point = trajectories[0]
        # self.arm.move_to(start_point, orientation, speed)
