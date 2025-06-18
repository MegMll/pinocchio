import unittest

import numpy as np
import pinocchio as pin


class TestModelGraphBindings(unittest.TestCase):
    def test_build_model(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())

        g.addJoint(
            "b1_b2",
            pin.graph.JointPrismaticGraph(np.array([1, 0, 0])),
            "body1",
            pin.SE3.Random(),
            "body2",
            pin.SE3.Random(),
        )

        m = pin.graph.buildModel(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

        m, bi = pin.graph.buildModelWithBuildInfo(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

    def test_converter(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())
        g.addBody("body3", pin.Inertia.Identity())
        g.addJoint(
            "b1_b2",
            pin.graph.JointPrismaticGraph(np.array([1, 0, 0])),
            "body1",
            pin.SE3.Random(),
            "body2",
            pin.SE3.Random(),
        )
        g.addJoint(
            "b2_b3",
            pin.graph.JointPrismaticGraph(np.array([0, 1, 0])),
            "body2",
            pin.SE3.Random(),
            "body3",
            pin.SE3.Random(),
        )

        m_body1, build_info_body1 = pin.graph.buildModelWithBuildInfo(
            g, "body1", pin.SE3.Identity()
        )
        q_body1 = pin.randomConfiguration(
            m_body1, np.array([-1.0] * 2), np.array([1.0] * 2)
        )
        v_body1 = np.random.random(m_body1.nv)

        m_body3, build_info_body3 = pin.graph.buildModelWithBuildInfo(
            g, "body3", pin.SE3.Identity()
        )
        converter = pin.graph.createConverter(
            m_body1, m_body3, build_info_body1, build_info_body3
        )
        q_body3 = converter.convertConfigurationVector(q_body1)
        self.assertTrue(q_body1[0] == -q_body3[-1])
        self.assertTrue(q_body1[1] == -q_body3[-2])

        v_body3 = converter.convertTangentVector(q_body1, v_body1)
        self.assertTrue(v_body1[0] == -v_body3[-1])
        self.assertTrue(v_body1[1] == -v_body3[-2])


if __name__ == "__main__":
    unittest.main()
