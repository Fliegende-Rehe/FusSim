from .fusion import *

import adsk.core
import adsk.fusion

from typing import List

ORIGIN_OFFSET = [1387.161, 0.00, 971.549]


class Part:
    def __init__(self, assembly):
        self.component = assembly.get_component_by_name('part')
        self.trajectories = None

    def get_trajectories(self, tolerance=1.0) -> List[List]:
        self.set_trajectories(tolerance)
        return self.trajectories

    def draw_spline(self, pnt):
        sketch = self.component.sketches.add(self.component.yZConstructionPlane)

        points = adsk.core.ObjectCollection.create()
        for p in pnt:
            points.add(p)

        spline = sketch.sketchCurves.sketchFittedSplines.add(points)

        fitPoints = spline.fitPoints

        fitPoint = fitPoints.item(1)

        line = spline.getTangentHandle(fitPoint)
        if line is None:
            line = spline.activateTangentHandle(fitPoint)

        gottenLine = spline.getTangentHandle(fitPoint)

        gottenLine.deleteMe()

        activatedArc = spline.activateCurvatureHandle(fitPoint)

        gottenArc = spline.getCurvatureHandle(fitPoint)
        gottenLine = spline.getTangentHandle(fitPoint)

        gottenArc.deleteMe()

    def set_trajectories(self, tolerance):
        points = self.get_spline_points(tolerance)
        self.draw_spline(points)
        self.trajectories = [list(p.asArray()) for p in points]
        for point in self.trajectories:
            print(f'{point}\n')

    def get_spline_points(self, tolerance):
        sketch = self.component.sketches[0]
        spline = sketch.sketchCurves.sketchFittedSplines[0]

        evaluator = spline.geometry.evaluator
        start, end = evaluator.getParameterExtents()[1:]

        return evaluator.getStrokes(start, end, tolerance)[1]
