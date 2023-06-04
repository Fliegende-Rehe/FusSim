import numpy as np

from .fusion import *


class Trajectory:
    def __init__(self, curves, tolerance, scale=10) -> None:
        self.points = []

        for curve in curves:
            length = curve.length * scale
            start, end = curve.geometry.evaluator.getParameterExtents()[1:3]
            # maximum distance tolerance between the curve and the linear interpolation.
            strokes = curve.geometry.evaluator.getStrokes(start, end, tolerance / 1000)[1]
            edges = []
            for point2D in strokes:
                x, y, z = point2D.asArray()
                point3D = [-x * scale, -y * scale, z * scale]
                edges.append(np.array(point3D))

            if isinstance(curve, adsk.fusion.SketchLine):
                edges = interpolate_3D_points(edges, int(length / tolerance))

            self.points.extend(edges[:-1])

        self.points.sort(key=lambda col: col[1])
        logger(f'The trajectory is made up of {len(self.points)} points')

    def offset_points(self, offset: list[float]) -> list[list[float]]:
        self.points = [list(map(sum, zip(offset, p))) for p in self.points]
        return self.points


def interpolate_3D_points(points, N):
    start_point, end_point = points
    ratios = np.linspace(0, 1, N)
    points = [start_point * (1 - ratio) + end_point * ratio for ratio in ratios]
    return points
