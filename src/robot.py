from asyncio import create_task, gather, run

from .fusion import *
from .link import Link
from .kinematics import *


class Robot:
    def __init__(self, body, constraints: list[dict]) -> None:
        self.links: list[Link] = [Link(joint, constraint) for joint, constraint in zip(body.joints, constraints)]
        self.name: str = body.name
        self.kinematics = Kinematics(self.links)

    def get_home_position(self) -> list[float]:
        return [lnk.get_home_positions() for lnk in self.links]

    def get_name(self) -> str:
        return self.name

    def get_drive_time(self, target: list[float], speed: float) -> float:
        max_range = max(self.get_drive_ranges(target))
        return max_range / speed if max_range != 0 else 0

    def get_drive_ranges(self, target: list[float]) -> list[float]:
        current = self.get_position()
        return [abs(tar - cur) for tar, cur in zip(target, current)]

    async def drive(self, target: list[float], speed: float, home) -> None:
        def synchronize_links_speed() -> list[float]:
            ranges = self.get_drive_ranges(target)
            drive_time = self.get_drive_time(target, speed)
            return [rng / drive_time if rng != 0 else 0 for rng in ranges]

        async def async_drive() -> None:
            tasks = []
            for index, (link, tar) in enumerate(zip(self.links, target)):
                rng = tar - current[index]
                link_is_ready[index] = abs(rng) == 0
                if not link_is_ready[index]:
                    if abs(rng) > speeds[index]:
                        rotation_direction = 1 if rng > 0 else -1
                        current[index] += rotation_direction * speeds[index]
                    else:
                        current[index] = tar
                    if not link.fit_limits(current[index]):
                        fusion_exit(kill=True)
                    tasks.append(create_task(link.set_position(current[index])))
            await gather(*tasks)

        initial = rounded(self.get_position())
        current = self.get_position()
        speeds = synchronize_links_speed()
        if all(spd == 0 for spd in speeds):
            return
        link_is_ready = [False] * len(self.links)
        while not all(link_is_ready):
            await async_drive()
            refresh_display()

        logger(f'|{self.name}| at home position' if home
               else f'|{self.name}| move link from {initial} to {rounded(target)}')

    def move_to(self, position: list[float], orientation: list[float], speed: float) -> None:
        position = self.kinematics.get_links_position(position, orientation)

        async def async_move_to() -> None:
            await gather(self.drive(position, speed, False))

        run(async_move_to())

    def get_position(self) -> list[float]:
        return [link.get_position() for link in self.links]

    def get_random_position(self) -> list[float]:
        return [link.get_random_position() for link in self.links]
