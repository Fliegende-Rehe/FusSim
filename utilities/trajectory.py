from utilities.fusion import *


class Trajectory:
    def __init__(self, spline, tolerance: float) -> None:
        start, end =  spline.geometry.evaluator.getParameterExtents()[1:3]
        strokes = spline.geometry.evaluator.getStrokes(start, end, tolerance)[1]
        self.points = [list(s.asArray()) for s in strokes]
        logger(f'The trajectory consists of {len(self.points)} points')

    def update_points(self, origin: list[float]) -> list[list[float]]:
        self.points = [list(map(sum, zip(origin, p))) for p in self.points]
        return self.points
