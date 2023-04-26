from .fusion import *

ORIGIN_OFFSET = [1387.161, 0.00, 971.549]


class Part:
    def __init__(self, assembly):
        self.component = assembly.get_component_by_name('part')
        self.trajectory = None

    def get_trajectory(self, tolerance=1.0):
        self.set_trajectory(tolerance)
        return self.trajectory

    def set_trajectory(self, tolerance):
        sketch = self.component.sketches[0]
        spline = sketch.sketchCurves.sketchFittedSplines[0]
        strokes = spline.geometry.evaluator.getStrokes(0.0, 1.0, tolerance)[1]
        points = [list(s.asArray()) for s in strokes]
        logger(f'The trajectory consists of {len(points)} points')
        self.trajectory = [list(map(sum, zip(ORIGIN_OFFSET, p))) for p in points]
