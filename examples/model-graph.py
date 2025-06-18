import numpy as np
import pinocchio as pin

# Create an empty model graph
g = pin.graph.ModelGraph()

###### Add bodies, sensors...
g.addBody("body1", pin.Inertia.Identity())
g.addBody("body2", pin.Inertia.Identity())

###### Add joints
g.addJoint(
    "b1_b2",
    pin.graph.JointPrismaticGraph(np.array([1, 0, 0])),
    "body1",
    pin.SE3.Random(),
    "body2",
    pin.SE3.Random(),
)

# Create model based on graph
m = pin.graph.buildModel(g, "body1", pin.SE3.Identity())

print(m)
