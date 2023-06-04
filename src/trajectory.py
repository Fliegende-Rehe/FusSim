import numpy as np

from .fusion import *


class Trajectory:
    def __init__(self, curves, tolerance, scale=10) -> None:
        self.points = []
        self.scale = scale

        for curve in curves:
            length = curve.length * self.scale
            point_num = int(length / tolerance)
            edges = []

            if isinstance(curve, adsk.fusion.SketchLine):
                start_point, end_point = [self.point3d2vector(point3d) for point3d in curve.geometry.getData()[1:3]]
                edges = interpolate_line(start_point, end_point, point_num)

            if isinstance(curve, adsk.fusion.SketchArc):
                _, center, normal, reference_vector, radius, start_angle, end_angle = curve.geometry.getData()
                center = self.point3d2vector(center)
                normal = self.point3d2vector(normal.asPoint())
                reference_vector = self.point3d2vector(reference_vector.asPoint())
                radius *= scale
                edges = interpolate_arc(point_num, center, normal, reference_vector, radius, start_angle, end_angle)
            self.points.extend(edges[:-1])

        self.points.sort(key=lambda col: col[1])
        logger(f'The trajectory is made up of {len(self.points)} points')

    def offset_points(self, offset: list[float]):
        self.points = [list(map(sum, zip(offset, p))) for p in self.points]

    def point3d2vector(self, point3d):
        x, y, z = point3d.asArray()
        return np.array([-x * self.scale, -y * self.scale, z * self.scale])


def interpolate_line(start_point, end_point, point_num):
    ratios = np.linspace(0, 1, point_num)
    return [start_point * (1 - ratio) + end_point * ratio for ratio in ratios]


def interpolate_arc(N, center, normal, reference_vector, radius, start_angle, end_angle):
    normal = normal / np.linalg.norm(normal)
    reference_vector = reference_vector / np.linalg.norm(reference_vector)

    reference_vector -= reference_vector.dot(normal) * normal
    reference_vector /= np.linalg.norm(reference_vector)

    binormal = np.cross(normal, reference_vector)
    binormal /= np.linalg.norm(binormal)

    angles = np.linspace(start_angle, end_angle, N)

    points = np.zeros((N, 3))
    for i in range(N):
        points[i] = center + radius * (np.cos(angles[i]) * reference_vector + np.sin(angles[i]) * binormal)

    return points
