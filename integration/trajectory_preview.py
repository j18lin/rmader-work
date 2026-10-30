"""Small, ROS-independent preview of an R-MADER PieceWisePolTraj."""
import math


def _value(coeff, u):
    return ((coeff.a*u + coeff.b)*u + coeff.c)*u + coeff.d


def sample_pwp(pwp, max_points=80):
    """Return world-frame [x, y] points from each normalized cubic interval."""
    times = list(pwp.times)
    n = len(times) - 1
    if n < 1 or len(pwp.coeff_x) != n or len(pwp.coeff_y) != n:
        raise ValueError("invalid PWP interval counts")
    if not all(math.isfinite(t) for t in times):
        raise ValueError("non-finite PWP time")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("PWP times must increase")
    points = []
    for i in range(n):
        cx, cy = pwp.coeff_x[i], pwp.coeff_y[i]
        values = (cx.a, cx.b, cx.c, cx.d, cy.a, cy.b, cy.c, cy.d)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite PWP coefficient")
        samples = max(2, min(8, int(math.ceil((times[i+1]-times[i]) / 0.15))))
        for j in range(samples):
            u = j / float(samples)
            x, y = _value(cx, u), _value(cy, u)
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("non-finite PWP point")
            points.append([round(x, 3), round(y, 3)])
    x, y = _value(pwp.coeff_x[-1], 1.0), _value(pwp.coeff_y[-1], 1.0)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("non-finite PWP endpoint")
    points.append([round(x, 3), round(y, 3)])
    if len(points) > max_points:
        points = [points[round(i*(len(points)-1)/(max_points-1))]
                  for i in range(max_points)]
    return points

