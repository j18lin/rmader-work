import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as Obj

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "integration"))
from trajectory_preview import sample_pwp


def coeff(a=0, b=0, c=0, d=0):
    return Obj(a=a, b=b, c=c, d=d)


class PreviewTest(unittest.TestCase):
    def test_normalized_cubic_intervals(self):
        pwp = Obj(times=[10.0, 11.0, 12.0],
                  coeff_x=[coeff(c=1), coeff(c=1, d=1)],
                  coeff_y=[coeff(), coeff(c=1)])
        points = sample_pwp(pwp)
        self.assertEqual(points[0], [0.0, 0.0])
        self.assertIn([1.0, 0.0], points)
        self.assertEqual(points[-1], [2.0, 1.0])
        self.assertGreater(len(points), 4)

    def test_bad_polynomial_is_not_visualized(self):
        for pwp in (
            Obj(times=[0, 1], coeff_x=[], coeff_y=[]),
            Obj(times=[1, 0], coeff_x=[coeff()], coeff_y=[coeff()]),
            Obj(times=[0, 1], coeff_x=[coeff(d=math.nan)], coeff_y=[coeff()]),
        ):
            with self.assertRaises(ValueError):
                sample_pwp(pwp)


if __name__ == "__main__":
    unittest.main()

