from .fusion import *

from typing import List


class Part:
    def __init__(self, assembly):
        self.body = assembly.get_component_by_name('part')


    def get_trajectories(self) -> List[List]:
        sketch = self.body.sketches[0]

        # trajectory = adsk.core.ObjectCollection.create()
        # trajectory.add(adsk.core.Point3D.create(0, 0, 0))
        # trajectory.add(adsk.core.Point3D.create(5, 1, 0))
        #
        # for point in trajectory:
        #     print(point.)
        return self.body.sketches
