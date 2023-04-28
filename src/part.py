from .trajectory import *
from .assembly import *

TOLERANCE = 0.5


class Part:
    def __init__(self, assembly: Assembly, tolerance: float = TOLERANCE):
        body = assembly.get_component_by_name('part')
        trajectories_sketch = body.sketches[0]
        trajectories_spline = trajectories_sketch.sketchCurves.sketchFittedSplines
        self.trajectories = [Trajectory(spline, tolerance) for spline in trajectories_spline]

    def update_trajectories(self) -> list[Trajectory]:
        origin = [0, 0, 0]
        for tr in self.trajectories:
            tr.update_points(origin)
        return self.trajectories
