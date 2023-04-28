from .fusion import *


class Trajectory:
    def __init__(self, part, tolerance: float) -> None:
        self.spline = part.sketches[0].sketchCurves.sketchFittedSplines[0]
        strokes = self.spline.geometry.evaluator.getStrokes(0.0, 1.0, tolerance)[1]
        self.points = [list(s.asArray()) for s in strokes]
        logger(f'The trajectory consists of {len(self.points)} points')

    def get_points(self) -> list[list[float]]:
        self.update_trajectory()
        return self.points

    def update_trajectory(self) -> None:
        origin = [0, 0, 0]
        self.points = [list(map(sum, zip(origin, p))) for p in self.points]
