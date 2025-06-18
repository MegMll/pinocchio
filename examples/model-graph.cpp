#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>
///
/// This example show how to:
///  - Construct a kinematics chain with ModelGraph API
///  - Bias a joint
///  - Merge two kinematics chain into a kinematics tree
///  - Lock a joint in a particular configuration
///  - Create a Model with a custom root joint
int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  graph::ModelGraph g;

  // Adding bodies to the graph.
  g.addBody("body1", Inertia::Identity());
  g.addFrame("body2", graph::BodyFrameGraph(Inertia::Identity()));
  g.addFrame("body3", graph::BodyFrameGraph(Inertia::Identity()));

  // Adding a sensor to the graph.
  g.addFrame("sensor1", graph::SensorFrameGraph());

  // Now we add the joints between every body/sensor we have in the graph.
  SE3 pose_b1_to_j1 = SE3::Random(); // pose of joint j1 wrt body1
  SE3 pose_j1_to_b2 = SE3::Random(); // pose of body2 wrt joint j1
  g.addJoint(
    "j1", graph::JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1", pose_b1_to_j1, "body2",
    pose_j1_to_b2);

  // j2 will be biased by 50cm.
  SE3 pose_b2_to_j2 = SE3::Random(); // pose of joint j2 wrt body2
  SE3 pose_j2_to_b3 = SE3::Random(); // pose of body3 wrt joint j2
  Eigen::VectorXd j2_bias(1);
  j2_bias << 0.5;
  g.addJoint(
    "j2", graph::JointPrismaticGraph(Eigen::Vector3d::UnitX()), "body2", pose_b2_to_j2, "body3",
    pose_j2_to_b3, j2_bias);

  // sensor1 is a sensor frame so it can only be linked to the others body via a fixed joint
  SE3 pose_b1_s1 = SE3::Random();
  SE3 pose_s1 = SE3::Random();
  g.addJoint("b1_s1", graph::JointFixedGraph(), "body1", pose_b1_s1, "sensor1", pose_s1);

  // Now we can choose which body will be our root its position, and build the model
  Model kinematics_chain_from_body1 = graph::buildModel(g, "body1", SE3::Identity());
  std::cout << kinematics_chain_from_body1 << std::endl;

  // To merge two model, we can create a new ModelGraph and merge it to the first one.
  // To simplify the process, we will append g to g.
  // Since all joints and frames should have an unique name, we will use graph::prefixNames
  // function.
  graph::ModelGraph g1 = graph::prefixNames(g, "g1/");
  graph::ModelGraph g2 = graph::prefixNames(g, "g2/");

  // Then we will attach g2/body3 to g1/body2 with a spherical joint.
  graph::ModelGraph g1_g2_merged =
    graph::merge(g1, g2, "g1/body2", "g2/body3", SE3::Random(), graph::JointSphericalGraph());

  // We can then create our model with any body as a root.
  Model kinematics_tree_from_g1_body1 =
    graph::buildModel(g1_g2_merged, "g1/body1", SE3::Identity());
  std::cout << kinematics_tree_from_g1_body1 << std::endl;

  // To lock a joints we only have to provide his name and his reference configuration.
  // We will lock g1/j1 at 0.3 rad.
  Eigen::VectorXd j1_lock(1);
  j1_lock << 0.3;
  graph::ModelGraph g1_g2_merged_locked = graph::lockJoints(g1_g2_merged, {"g1/j1"}, {j1_lock});

  // We can then create the locked model.
  // We will use g2/body2 as root with a FreeFyier joint.
  Model kinematics_tree_from_g2_body2 = graph::buildModel(
    g1_g2_merged_locked, "g2/body2", SE3::Identity(), graph::JointFreeFlyerGraph());
  std::cout << kinematics_tree_from_g2_body2 << std::endl;

  return 0;
}
