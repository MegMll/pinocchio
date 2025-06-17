import numpy as np

import pinocchio as pin

# This example show how to:
#  - Construct a kinematics chain with ModelGraph API
#  - Build a Model with b1 as root body with a fixed base
#  - Build a Model with b3 as root body with a free flyer as root joint
#  - Use ModelConfigurationConverter API to convert configuration and velocity
#    vector from the first model to the second

# Construct kinematics chain with ModelGraph API.
g = pin.ModelGraph()

I_I = pin.Inertia.Identity()
X_I = pin.SE3.Identity()

g.addBody("b1", I_I)
g.addBody("b2", I_I)
g.addBody("b3", I_I)
g.addJoint(
    "j1",
    pin.JointRevoluteGraph(np.array([0.0, 0.0, 1.0])),
    "b1",
    pin.SE3.Random(),
    "b2",
    pin.SE3.Random(),
)
g.addJoint(
    "j2",
    pin.JointPrismaticGraph(np.array([1.0, 0.0, 0.0])),
    "b2",
    pin.SE3.Random(),
    "b3",
    pin.SE3.Random(),
)

# Create a Model with b1 as root body and a fixed base.
forward_model, forward_build_info = pin.buildModelWithBuildInfo(g, "b1", X_I)
forward_data = forward_model.createData()

# Compute b3 placement and velocity with forward model.
qmax = np.ones(forward_model.nq)
forward_q = pin.randomConfiguration(forward_model, -qmax, qmax)
forward_v = np.random.rand(forward_model.nv)
pin.forwardKinematics(forward_model, forward_data, forward_q, forward_v)
pin.updateFramePlacements(forward_model, forward_data)

b3_index = forward_model.getFrameId("b3", pin.BODY)
X_b3 = forward_data.oMf[b3_index]
V_b3 = pin.getFrameVelocity(forward_model, forward_data, b3_index)

# Create the backward model with b3 as root body and a free flyer as root joint.
# b3 is placed at the same position and same velocity than b3 in the forward model.
backward_model, backward_build_info = pin.buildModelWithBuildInfo(
    g, "b3", X_b3, pin.JointFreeFlyerGraph()
)
backward_data = backward_model.createData()

# Create the converter and convert the configuration and velocity vector.
f_to_b_converter = pin.createConverter(
    forward_model, backward_model, forward_build_info, backward_build_info
)
backward_q = f_to_b_converter.convertConfigurationVector(forward_q)
backward_v = f_to_b_converter.convertTangentVector(forward_q, forward_v)
backward_v[:6] = V_b3

pin.forwardKinematics(backward_model, backward_data, backward_q, backward_v)
pin.updateFramePlacements(backward_model, backward_data)

# Show that frame configuration and velocities are the same.
for i, frame in enumerate(forward_model.frames):
    if frame.type == pin.BODY:
        i_b = backward_model.getFrameId(frame.name, frame.type)
        motion_f = pin.getFrameVelocity(forward_model, forward_data, i)
        motion_b = pin.getFrameVelocity(backward_model, backward_data, i_b)

        print(f"Frame {frame.name} configuration and velocity in forward_model:")
        print(forward_data.oMf[i], motion_f)
        print(f"Frame {frame.name} configuration and velocity in backward_model:")
        print(backward_data.oMf[i_b], motion_b)
        print()
