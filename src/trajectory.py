from geomdl import NURBS
from geomdl.visualization import VisMPL
import numpy as np

from .fusion import *


class Trajectory:
    def __init__(self, curves, tolerance, scale=10) -> None:
        self.points = []
        self.scale = scale

        # for curve in curves:
        curve = curves[0]

        length = curve.length * self.scale
        curve = curve.geometry.asNurbsCurve
        quantity = int(length / tolerance)
        _, control_points, degree, knots, rational, weights, _ = curve.getData()

        nurbs_curve = NURBS.Curve()
        nurbs_curve.degree = degree
        nurbs_curve.knotvector = [knot for knot in knots]
        nurbs_curve.ctrlptsw = [self.point3d2vector(point3d) for point3d in control_points]
        t_vals = np.linspace(curve.knotvector[curve.degree], curve.knotvector[-curve.degree - 1], quantity)

        interpolation = [nurbs_curve.evaluate_single(t) for t in t_vals]

        self.points.extend(interpolation)

        # self.points.sort(key=lambda col: col[1])
        # logger(f'The trajectory is made up of {len(self.points)} points')

    def offset_points(self, offset: list[float]):
        self.points = [list(map(sum, zip(offset, p))) for p in self.points]

    def point3d2vector(self, point3d):
        x, y, z = point3d.asArray()
        return np.array([-x * self.scale, -y * self.scale, z * self.scale, 1])

# def calculate_basis_function(u, i, p, knots):
#     if p == 0:
#         return 1.0 if knots[i] <= u < knots[i + 1] else 0.0
#     else:
#         coeff1 = ((u - knots[i]) / (knots[i + p] - knots[i])) if knots[i] != knots[i + p] else 0
#         coeff2 = ((knots[i + p + 1] - u) / (knots[i + p + 1] - knots[i + 1])) if knots[i + 1] != knots[i + p + 1] else 0
#
#         a = coeff1 * calculate_basis_function(u, i, p - 1, knots)
#         b = coeff2 * calculate_basis_function(u, i + 1, p - 1, knots)
#         return a + b
#
#
# def interpolate_arc_NURBS(N, control_points, weights, knots, degree, isRational):
#     points = []
#     for i in range(N):
#         u = i / (N - 1)  # normalize u to range from 0 to 1
#         point = [0, 0, 0]
#         rational_weight = 0
#         for j, Pj in enumerate(control_points):
#             Rj = calculate_basis_function(u, j, degree, knots)  # you need to implement this
#             if isRational:
#                 Rj *= weights[j]
#                 rational_weight += Rj
#             point[0] += Rj * Pj[0]
#             point[1] += Rj * Pj[1]
#             point[2] += Rj * Pj[2]
#         if isRational:
#             point[0] /= rational_weight
#             point[1] /= rational_weight
#             point[2] /= rational_weight
#         points.append(point)
#     return points
