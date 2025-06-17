import unittest

import numpy as np
import pinocchio as pin


class TestModelGraphBindings(unittest.TestCase):
    def test_build_model(self):
        g = pin.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())

        g.addJoint(
            "b1_b2",
            pin.JointPrismaticGraph(np.array([1, 0, 0])),
            "body1",
            pin.SE3.Random(),
            "body2",
            pin.SE3.Random(),
        )

        m = pin.buildModel(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

        m, bi = pin.buildModelWithBuildInfo(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")


if __name__ == "__main__":
    unittest.main()
