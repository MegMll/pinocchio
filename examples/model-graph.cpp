#include "pinocchio/parsers/graph/model-graph.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  graph::ModelGraph g;

  // Adding bodies to the graph
  g.addBody("body1", Inertia::Identity());
  g.addFrame("body2", graph::BodyFrameGraph(Inertia::Identity()));

  //Adding a sensor to the graph
  g.addFrame("sensor1", graph::SensorFrameGraph());

  //Now we add the joints between every body/sensor we have in the graph
  SE3 pose_b1 = SE3::Random(); // pose of joint b1_b2 wrt body2
  SE3 pose_b2 = SE3::Random(); // pose of body2 wrt joint b1_b2
  g.addJoint("b1_b2", graph::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "body1", pose_b1, "body2", pose_b2);

  // sensor1 is a sensor frame so it can only be linked to the others body via a fixed joint
  SE3 pose_b1_s1 = SE3::Random();
  SE3 pose_s1 = SE3::Random();
  g.addJoint("b1_s1", graph::JointFixedGraph(), "body1", pose_b1_s1, "sensor1", pose_s1);

  // Now we can choose which body will be our root its position, and build the model
  Model m = g.buildModel("body1", SE3::Identity());

  std::cout << m << std::endl;

  return 0;
}