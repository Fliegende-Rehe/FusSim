from .fusion import *
from .link import Link
from .kinematics import *

from asyncio import create_task, gather, run
from typing import List
from math import radians, degrees

LOG_PRECISION = 3


class Robot:
    def __init__(self, body, constraints: List[dict]):
        self.links: List[Link] = [Link(joint, constraint) for joint, constraint in zip(body.joints, constraints)]
        self.name: str = body.name

    def get_name(self) -> str:
        return self.name

    def get_max_drive_time(self, target: List[float], speed: float) -> float:
        return max(self.get_ranges(target)) / speed

    def get_ranges(self, target: List[float]) -> List[float]:
        current = self.get_links_positions()
        return [abs(tar - cur) for tar, cur in zip(target, current)]

    async def launch(self) -> None:
        async def async_launch():
            tasks = [lnk.set_home() for lnk in self.links]
            await gather(*tasks)

        await async_launch()
        refresh()
        logger(f'|{self.name}| at home position')

    async def drive(self, target: List[float], speed: float) -> None:
        def synchronize_links_speed() -> List[float]:
            ranges = self.get_ranges(target)
            drive_time = max(ranges) / speed
            return [rng / drive_time for rng in ranges]

        async def async_drive() -> None:
            tasks = []
            for index, (link, tar) in enumerate(zip(self.links, target)):
                rng = tar - current[index]
                rotation_direction = 1 if rng > 0 else -1
                link_is_ready[index] = abs(rng) < speeds[index]
                if link_is_ready[index]:
                    continue
                step = speeds[index] if abs(rng) >= speeds[index] else abs(rng)
                current[index] += rotation_direction * step
                if not link.fit_limits(current[index]):
                    kill('angles to drive')
                tasks.append(create_task(link.async_set_position(current[index])))
            await gather(*tasks)

        initial = self.get_links_positions(LOG_PRECISION)
        current = self.get_links_positions()
        speeds = synchronize_links_speed()
        link_is_ready = [False] * len(self.links)
        while not all(link_is_ready):
            await async_drive()
            refresh()
        final = self.get_links_positions(LOG_PRECISION)
        logger(f'|{self.name}| moved from {initial} to {final}')

    def get_links_positions(self, precision: int = 10) -> List[float]:
        return [round(link.get_position(), precision) for link in self.links]

    def get_random_positions(self) -> List[float]:
        return [link.get_random_position() for link in self.links]

    def set_random_position(self, speed: float = 8.0) -> None:
        self.drive(self.get_random_positions(), speed)

    def go_to_coordinates(self, desire_position, desire_orientation, speed):
        kinematics = Kinematics(self.links)
        initial_joint_angles = [radians(ang) for ang in self.get_links_positions()]
        initial_position = kinematics.forward_kinematics(initial_joint_angles)

        angles = kinematics.inverse_kinematics(desire_position, desire_orientation, initial_position)

        for ang in angles:
            print(f'{ang}\n')
        # # run(self.drive(angles, speed))
