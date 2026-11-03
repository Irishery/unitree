"""Pure-numpy checks for the bottle RGB-D geometry helpers."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "g1_mujoco"))

from g1_mujoco.bottle_geometry import (  # noqa: E402
    fit_bottle_pose, red_bottle_mask)


class BottleGeometryTests(unittest.TestCase):
    def test_red_mask_separates_bottle_from_brown_table(self):
        image = np.zeros((40, 40, 3), dtype=np.uint8)
        image[:, :] = (137, 76, 30)       # table
        image[10:30, 15:25] = (111, 14, 6)  # shaded bottle side
        image[8:12, 17:23] = (220, 30, 20)  # bright top
        mask = red_bottle_mask(image)
        self.assertGreater(mask.sum(), 150)
        self.assertFalse(mask[35, 5])       # table only
        self.assertTrue(mask[20, 20])       # bottle side

    def test_cylinder_fit_recovers_centre_and_diameter(self):
        radius, height = 0.031, 0.235
        centre = np.array([0.45, 0.0, 0.8725])
        angles = np.linspace(0.0, np.pi, 200)
        z = np.linspace(centre[2] - height / 2, centre[2] + height / 2, 100)
        side = np.array([
            [centre[0] + radius * np.cos(a), centre[1] + radius * np.sin(a), zi]
            for a in angles for zi in z
        ])
        top = np.array([
            [centre[0] + r * np.cos(a), centre[1] + r * np.sin(a),
             centre[2] + height / 2]
            for r in np.linspace(0.0, radius, 20) for a in angles
        ])
        estimate = fit_bottle_pose(np.vstack((side, top)))
        self.assertIsNotNone(estimate)
        self.assertLess(np.linalg.norm(estimate["centre"] - centre), 3.0e-3)
        self.assertAlmostEqual(estimate["diameter"], 2.0 * radius, delta=5.0e-3)
        self.assertAlmostEqual(estimate["height"], height, delta=6.0e-3)


if __name__ == "__main__":
    unittest.main()
