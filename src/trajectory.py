from .fusion import *


class Trajectory:
    def __init__(self, spline, tolerance: float) -> None:
        start, end = spline.geometry.evaluator.getParameterExtents()[1:3]
        strokes = spline.geometry.evaluator.getStrokes(start, end, tolerance)[1]
        self.points = [[point * 10 for point in s.asArray()] for s in strokes]
        logger(f'The trajectory consists of {len(self.points)} points')

    def offset_points(self, offset: list[float]) -> list[list[float]]:
        self.points = [list(map(sum, zip(offset, p))) for p in self.points]
        return self.points
