from asyncio import create_task, gather

from .kinematics import *


class Robot:
    def __init__(self, body, constraints) -> None:
        dh_table, links_param = constraints.values()
        self.links = [Link(joint, constraint) for joint, constraint in zip(body.joints, links_param)]
        self.kinematics = Kinematics(self.links, dh_table)
        self.name = body.name
        logger(f'|{self.name}| home position is {rounded(self.kinematics.forward_kinematics().T.tolist()[0])}')

    def get_drive_time(self, target: list[float], speed: float) -> float:
        max_range = max(abs(tar - cur) for tar, cur in zip(target, self.kinematics.get_links_position()))
        return max_range / speed if max_range != 0 else 0

    def get_drive_ranges(self, target: list[float]) -> list[float]:
        current = self.kinematics.get_links_position()
        return [abs(tar - cur) for tar, cur in zip(target, current)]

    async def drive(self, target: list[float], speed: float, home) -> None:
        def _synchronize_links_speed():
            ranges = self.get_drive_ranges(target)
            drive_time = self.get_drive_time(target, speed)
            return [rng / drive_time if rng != 0 else 0 for rng in ranges]

        async def _async_drive():
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
                    if not link.get_limits(current[index]):
                        fusion_exit(kill=True)
                    tasks.append(create_task(link.set_position(current[index])))
            await gather(*tasks)

        current = self.kinematics.get_links_position()
        speeds = _synchronize_links_speed()
        if all(spd == 0 for spd in speeds):
            return
        link_is_ready = [False] * len(self.links)
        while not all(link_is_ready):
            await _async_drive()
            refresh_display()

        logger(f'|{self.name}| moved to ' + ('home' if home else str(rounded(self.kinematics.forward_kinematics()))))

    def get_random_angles(self) -> list[float]:
        return [link.get_random_position() for link in self.links]
