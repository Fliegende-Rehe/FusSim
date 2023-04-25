from asyncio import gather, create_task, run
from random import uniform

from .fusion import *
from .link import Link

SPEED = 6.0  # 0.075
LOG_PRESISION = 3
TIME_TO_CATCH_BREATH = 1


class Robot:
    def __init__(self, body, constrains):
        self.links = [Link(jnt, lim)
                      for jnt, lim in zip(body.joints, constrains)]
        self.name = body.name.split()[0]

    def launch(self):
        for lnk in self.links:
            lnk.set_home()
        refresh()
        logger(f'|{self.name}| at home position')
        sleep(TIME_TO_CATCH_BREATH)

    def drive(self, target):
        initial = self.get_links_positions(LOG_PRESISION)

        async def async_drive():
            tasks = []
            for index, (lnk, tar) in enumerate(zip(self.links, target)):
                rng = tar - current[index]
                rotation_direction = 1 if rng > 0 else -1
                link_is_ready[index] = abs(rng) < speed[index]
                if link_is_ready[index]:
                    continue
                step = speed[index] if abs(
                    rng) >= speed[index] else abs(rng)
                current[index] += rotation_direction * step
                if not lnk.fit_limits(current[index]):
                    break
                tasks.append(create_task(
                    lnk.async_set_position(current[index])))
            await gather(*tasks)

        current = self.get_links_positions()
        ranges = [abs(tar - cur) for tar, cur in zip(target, current)]
        max_time = max(ranges) / SPEED
        speed = [rng / max_time for rng in ranges]
        link_is_ready = [False] * len(self.links)
        while not all(link_is_ready):
            run(async_drive())
            refresh()

        final = self.get_links_positions(LOG_PRESISION)
        logger(f'|{self.name}| moved from {initial} to {final}')
        sleep(TIME_TO_CATCH_BREATH)

    def get_links_positions(self, presision=10):
        return [round(lnk.get_position(), presision) for lnk in self.links]

    def set_random_position(self):
        self.drive([uniform(lnk.min, lnk.max) for lnk in self.links])
