from .trajectory import *

TOLERANCE = 0.5


class Part:
    def __init__(self, assembly: Assembly, tolerance: float = TOLERANCE):
        self.body = assembly.get_component_by_name('part')
        sketch = self.body.sketches[0]
        curves = sketch.sketchCurves
        self.trajectory = Trajectory(curves, tolerance)
        self.assembly = assembly
        self.set_to_world()

    def get_origin(self):
        return self.assembly.get_component_origin(self.body)

    def set_to_world(self):
        origin = self.get_origin()
        self.trajectory.offset_points(origin)
