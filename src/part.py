from .trajectory import *

TOLERANCE = 0.5


class Part:
    def __init__(self, assembly: Assembly, tolerance: float = TOLERANCE):
        self.body = assembly.get_component_by_name('part')
        trajectories_sketch = self.body.sketches[0]
        trajectories_spline = trajectories_sketch.sketchCurves.sketchFittedSplines
        self.trajectories = [Trajectory(spline, tolerance) for spline in trajectories_spline]
        self.assembly = assembly
        self.sey_to_world()

    def get_origin(self):
        return self.assembly.get_component_origin(self.body)

    def sey_to_world(self) -> list[Trajectory]:
        origin = self.get_origin()
        for tr in self.trajectories:
            tr.offset_points(origin)
        return self.trajectories
