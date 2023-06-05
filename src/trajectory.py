from .fusion import *


class Trajectory:
    def __init__(self, curves, tolerance, scale=10) -> None:
        self.points = []
        self.scale = scale

        success = True
        length = 0
        NurbsCurve3D.merge
        for curve in curves:
            length += curve.length * self.scale
            success, start, end = curve.geometry.evaluator.getParameterExtents()
            success, strokes = curve.geometry.evaluator.getStrokes(start, end, tolerance)
            interpolation = [self.point_transformation(point) for point in strokes]
            if success:
                self.points.extend(interpolation)
            else:
                fusion_exit(True)

        self.points.sort(key=lambda col: col[1])

        logger(f'The trajectory length is {int(length)} mm ({len(self.points)} points)')

    def offset_points(self, offset: list[float]):
        self.points = [list(map(sum, zip(offset, p))) for p in self.points]

    def point_transformation(self, point3d):
        x, y, z = point3d.asArray()
        return [-x * self.scale, -y * self.scale, z * self.scale]
