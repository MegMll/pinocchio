//
// Copyright (c) 2024-2025 INRIA
//
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"

#include <boost/test/unit_test.hpp>

pinocchio::graph::ModelGraph
buildReversableModelGraph(const pinocchio::graph::JointGraphVariant & joint)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrameGraph(pinocchio::Inertia::Identity()));
  g.addFrame(
    "body2", BodyFrameGraph(
               pinocchio::Inertia(
                 4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Zero())));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  g.addJoint("body1_to_body2", joint, "body1", poseBody1, "body2", poseBody2);

  return g;
}

/// function isApprox better to avoid problem with zero precision
bool SE3isApprox(
  const pinocchio::SE3 & s1,
  const pinocchio::SE3 & s2,
  const double prec = Eigen::NumTraits<double>::dummy_precision())
{
  return s1.rotation().isApprox(s2.rotation())
         && (s1.translation() - s2.translation()).isZero(prec);
}

BOOST_AUTO_TEST_SUITE(ModelGraphTest)

/// @brief test if vertex are added to the graph
BOOST_AUTO_TEST_CASE(test_add_vertex)
{
  using namespace pinocchio::graph;

  ModelGraph g;

  g.addFrame("body1", BodyFrameGraph(pinocchio::Inertia::Identity()));
  BOOST_CHECK(g.name_to_vertex.find("body1") != g.name_to_vertex.end());
}

/// @brief Test if edges and their reverse are added correctly to the graph
BOOST_AUTO_TEST_CASE(test_add_edges)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  ////////////////////////////////////// Bodies
  g.addFrame("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));

  auto v_out = g.name_to_vertex["body1"];
  auto v_in = g.name_to_vertex["body2"];
  BOOST_CHECK(boost::edge(v_out, v_in, g.g).second);
  BOOST_CHECK(boost::edge(v_in, v_out, g.g).second);
}

/// @brief Test of simple 2R robot to try out kinematics and what happens when we use different body
/// as root body body1 --- body2
BOOST_AUTO_TEST_CASE(test_linear_2D_robot)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevoluteGraph(Eigen::Vector3d::UnitY()));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());
  BOOST_CHECK(m.jointPlacements[m.getJointId("body1_to_body2")].isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2, 0, 0))));

  // Forward kinematics
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[0] = M_PI / 2;

  // Compute forward kinematics
  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  pinocchio::Model m1 = g.buildModel("body2", pinocchio::SE3::Identity());
  // Compute forward kinematics
  pinocchio::Data d1(m1);
  pinocchio::framesForwardKinematics(m1, d1, -q);

  // World to Body1 (Identity)
  pinocchio::SE3 X1 = pinocchio::SE3::Identity();
  // Body1 to Joint1 (translation of 2 along X)
  pinocchio::SE3 X2 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0));
  // Rotation around Y by q = pi/2
  Eigen::AngleAxisd R_y(q[0], Eigen::Vector3d::UnitY());
  pinocchio::SE3 X3 = pinocchio::SE3(R_y.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body2 (translation of 2 along Y)
  pinocchio::SE3 X4 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 3.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose = X1 * X2 * X3 * X4;
  BOOST_CHECK(d.oMf[m.getFrameId("body2", pinocchio::BODY)].isApprox(bodyPose));

  // World to Body2 (Identity)
  pinocchio::SE3 X1_ = pinocchio::SE3::Identity();
  // Body2 to Joint1
  pinocchio::SE3 X2_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, -3.0, 0.0));
  // Rotation around Y by q = -pi/2 (reverse joint)
  Eigen::AngleAxisd R_y_(-q[0], Eigen::Vector3d::UnitY());
  pinocchio::SE3 X3_ = pinocchio::SE3(R_y_.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Joint1 to Body1
  pinocchio::SE3 X4_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2.0, 0.0, 0.0));
  // Final transformation
  pinocchio::SE3 bodyPose1 = X1_ * X2_ * X3_ * X4_;
  BOOST_CHECK(d1.oMf[m1.getFrameId("body1", pinocchio::BODY)].isApprox(bodyPose1));
}

/// @brief Test out the fixed joint.
BOOST_AUTO_TEST_CASE(test_fixed_joint)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", pinocchio::Inertia::Identity());
  g.addFrame("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", JointFixedGraph(), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -3., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -5., 0.)));

  ///////////////// Model
  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity(), JointFreeFlyerGraph());

  BOOST_CHECK(m.njoints == 3);
  BOOST_CHECK(m.frames[m.getFrameId("body2_to_body3", pinocchio::FIXED_JOINT)].placement.isApprox(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4., -1., 0.))));
}

/// @brief test construction of model with a mimic
BOOST_AUTO_TEST_CASE(test_mimic_joint)
{
  using namespace pinocchio::graph;
  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", BodyFrameGraph(pinocchio::Inertia::Identity()));
  g.addFrame("body3", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 pose_body1_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 pose_body2_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 4., 0.));
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "body1", pose_body1_joint1,
    "body2", pose_body2_joint1);

  pinocchio::SE3 pose_body2_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5., 0., 0.));
  pinocchio::SE3 pose_body3_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.));
  double scaling = 2.0;
  double offset = 0.5;
  g.addJoint(
    "body2_to_body3",
    JointMimicGraph(
      JointRevoluteGraph(Eigen::Vector3d::UnitY()), "body1_to_body2", scaling, offset),
    "body2", pose_body2_joint2, "body3", pose_body3_joint2);

  ///////////////// Model
  pinocchio::SE3 pose_body1_universe = pinocchio::SE3::Identity();
  pinocchio::Model m = g.buildModel("body1", pose_body1_universe);

  Eigen::VectorXd q(m.nq);
  q << M_PI / 2;

  pinocchio::Data d(m);
  pinocchio::framesForwardKinematics(m, d, q);

  // First revolute around X
  Eigen::AngleAxisd R_x(q[0], Eigen::Vector3d::UnitX());
  pinocchio::SE3 X_joint1 = pinocchio::SE3(R_x.toRotationMatrix(), Eigen::Vector3d::Zero());
  Eigen::AngleAxisd R_y(scaling * q[0] + offset, Eigen::Vector3d::UnitY());
  pinocchio::SE3 X_joint2 = pinocchio::SE3(R_y.toRotationMatrix(), Eigen::Vector3d::Zero());
  // Final transformation
  pinocchio::SE3 bodyPose = pose_body1_universe * pose_body1_joint1 * X_joint1 * pose_body2_joint1
                            * pose_body2_joint2 * X_joint2 * pose_body3_joint2;

  BOOST_CHECK(d.oMf[m.getFrameId("body3", pinocchio::BODY)].isApprox(bodyPose));
}

/// @brief test out reverse joint for revolute
BOOST_AUTO_TEST_CASE(test_reverse_revolute)
{
  using namespace pinocchio::graph;
  ModelGraph g = buildReversableModelGraph(JointRevoluteGraph(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief Test for a reverse revolute unbounded
BOOST_AUTO_TEST_CASE(test_reverse_revolute_unbounded)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevoluteUnboundedGraph(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  double ca, sa;
  pinocchio::SINCOS(M_PI / 4, &ca, &sa);
  q[0] = ca;
  q[1] = sa;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_reverse.nq);
  q_reverse[0] = q[0];  // cos(-a) = cos(a)
  q_reverse[1] = -q[1]; // sin(-a) = -sin(a)
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test out reverse joint for prismatic
BOOST_AUTO_TEST_CASE(test_reverse_prismatic)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointPrismaticGraph(Eigen::Vector3d::UnitY()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = 0.2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test out reverse joint for helical
BOOST_AUTO_TEST_CASE(test_reverse_helical)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointHelicalGraph(Eigen::Vector3d::UnitY(), 2.3));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 3;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test reversing helical joint on a simple linear robot
/// body1 --- body2
BOOST_AUTO_TEST_CASE(test_reverse_universal)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(
    JointUniversalGraph(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX()));

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  q[1] = M_PI / 4;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_forward.nq);
  q_reverse[0] = q[1];
  q_reverse[1] = q[0];

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief compare reverse model with spherical
BOOST_AUTO_TEST_CASE(test_reverse_spherical)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointSphericalGraph());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(0.4, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;

  Eigen::VectorXd q(m_forward.nq);
  q << q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w();

  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << q_sph.inverse().x(), q_sph.inverse().y(), q_sph.inverse().z(), q_sph.inverse().w();

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_spherical_zyx)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointSphericalZYXGraph());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  // config vector forward model ZYX
  Eigen::Vector3d q(m_forward.nq);
  q << M_PI / 4, M_PI, M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  // rotation matrix for spherique xyz for inverting spherical zyx
  Eigen::AngleAxisd Rx(-q[2], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(-q[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(-q[0], Eigen::Vector3d::UnitZ());
  // Eigen convention is right multiply
  Eigen::Matrix3d R = Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();

  Eigen::Vector3d q_reverse = R.eulerAngles(2, 1, 0);

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_composite)
{
  using namespace pinocchio::graph;

  JointCompositeGraph jmodel;
  pinocchio::SE3 jPose1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.)); // from body to j1
  pinocchio::SE3 jPose2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)); // from j1 to j2
  pinocchio::SE3 jPose3 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)); // from j2 to j3
  jmodel.addJoint(JointRevoluteGraph(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(JointRevoluteGraph(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(JointPrismaticGraph(Eigen::Vector3d::UnitY()), jPose3);

  ModelGraph g = buildReversableModelGraph(jmodel);

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q << M_PI / 2, M_PI / 2, 0.4;

  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << -q[2], -q[1], -q[0];
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);

  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_planar)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointPlanarGraph());

  //////////////////////////////////// Forward model
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  double ca, sa;
  pinocchio::SINCOS(M_PI / 3, &ca, &sa);
  q << 2, 1, ca, sa;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);
  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  // Compute reverse coordinate
  Eigen::Vector3d trans;
  trans << q[0], q[1], 0;
  Eigen::VectorXd q_reverse = Eigen::VectorXd::Zero(m_reverse.nq);
  Eigen::Matrix3d R;
  R << ca, sa, 0, -sa, ca, 0, 0, 0, 1;
  Eigen::Vector3d trans_rev = -R * trans;
  q_reverse << trans_rev[0], trans_rev[1], ca, -sa;

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  //////////////////////////////////// All bodies should be at the same configuration
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
  ///////////////// Model
  // BOOST_CHECK_THROW(g.buildModel("body2", pinocchio::SE3::Identity()), std::runtime_error);
}

/// @brief test if reversing of a composite joint is correct.
BOOST_AUTO_TEST_CASE(test_reverse_mimic)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrameGraph(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrameGraph(pinocchio::Inertia::Identity()));
  g.addFrame("body3", BodyFrameGraph(pinocchio::Inertia::Identity()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 pose_body1_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 pose_body2_joint1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 4., 0.));
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "body1", pose_body1_joint1,
    "body2", pose_body2_joint1);

  pinocchio::SE3 pose_body2_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5., 0., 0.));
  pinocchio::SE3 pose_body3_joint2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.));
  double scaling = 2.0;
  double offset = 0.5;
  g.addJoint(
    "body2_to_body3",
    JointMimicGraph(
      JointRevoluteGraph(Eigen::Vector3d::UnitY()), "body1_to_body2", scaling, offset),
    "body2", pose_body2_joint2, "body3", pose_body3_joint2);

  ///////////////// Model
  BOOST_CHECK_THROW(g.buildModel("body3", pinocchio::SE3::Identity()), std::runtime_error);
}

/// @brief Test out if inertias are well placed on the model
BOOST_AUTO_TEST_CASE(test_inertia)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  pinocchio::Inertia inert = pinocchio::Inertia(
    1., pinocchio::Inertia::Vector3(0., -2., 1.), pinocchio::Symmetric3::Random());
  g.addFrame("body1", inert);
  g.addFrame(
    "body2", pinocchio::Inertia(
               4., pinocchio::Inertia::Vector3(0., 2., 0.), pinocchio::Symmetric3::Random()));
  g.addFrame("body3", inert);

  /////////////////////////////////////// Joints
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 2., 0.)));
  g.addJoint(
    "body2_to_body3", JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., -2., 0.)), "body3",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-2., 0., 0.)));

  pinocchio::Model m = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Model m1 = g.buildModel("body3", pinocchio::SE3::Identity());

  Eigen::VectorXd q = Eigen::VectorXd::Random(m.nq);

  pinocchio::Data d(m);
  pinocchio::Data d1(m1);

  pinocchio::crba(m, d, q);
  pinocchio::crba(m1, d1, -q);

  BOOST_CHECK(d.M.isApprox(d1.M));
}

/// @brief test out a tree robot
///          /---- left leg
/// torso ---
///          \--- right leg
/// @param
BOOST_AUTO_TEST_CASE(test_tree_robot)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  g.addFrame("torso", pinocchio::Inertia::Identity());
  g.addFrame("left_leg", pinocchio::Inertia::Identity());
  g.addFrame("right_leg", pinocchio::Inertia::Identity());

  g.addJoint(
    "torso_to_left_leg", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
  g.addJoint(
    "torso_to_right_leg", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

  pinocchio::Model m = g.buildModel("torso", pinocchio::SE3::Identity(), JointFreeFlyerGraph());

  BOOST_CHECK(m.parents[m.getJointId("torso_to_left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso_to_right_leg")] == m.getJointId("root_joint"));

  pinocchio::Model m1 = g.buildModel("left_leg", pinocchio::SE3::Identity(), JointFreeFlyerGraph());
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_left_leg")] == m1.getJointId("root_joint"));
  BOOST_CHECK(m1.parents[m.getJointId("torso_to_right_leg")] == m1.getJointId("torso_to_left_leg"));
}

/// @brief Test to make sure that we can't chain sensor or OpFrame, that not fixed joint is created
/// when adding a sensor frame
/// @param
BOOST_AUTO_TEST_CASE(test_other_frame)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevoluteGraph(Eigen::Vector3d::UnitX()));

  g.addFrame("sensor1", SensorFrameGraph());

  /////////////////////////////////////// Joints
  g.addJoint(
    "body2_to_sensor1", JointFixedGraph(), "body2",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "sensor1",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)));

  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());

  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 2;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("sensor1", d_f.oMf[m_forward.getFrameId("sensor1", pinocchio::SENSOR)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  BOOST_CHECK(m_forward.frames.size() == 5); // All the bodies (2), joints (2) and one sensor
  BOOST_CHECK(m_reverse.frames.size() == 5); // same

  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));

  g.addFrame("sensor2", SensorFrameGraph());
  BOOST_CHECK_THROW(
    g.addJoint(
      "sensor2_sensor1", JointFixedGraph(), "sensor1", pinocchio::SE3::Identity(), "sensor2",
      pinocchio::SE3::Identity()),
    std::runtime_error);
}

/// @brief Test q_ref positioning
BOOST_AUTO_TEST_CASE(test_q_ref_revolute)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", pinocchio::Inertia::Identity());
  g.addFrame("body2", pinocchio::Inertia::Identity());

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(1);
  q_ref[0] = M_PI / 4;
  g.addJoint(
    "body1_to_body2", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "body1", poseBody1, "body2",
    poseBody2, q_ref);

  /////////////////////////////////////// Joints
  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());

  pinocchio::Data d_f(m_forward);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q[0] = M_PI / 4;
  pinocchio::framesForwardKinematics(m_forward, d_f, q);

  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);
  pinocchio::framesForwardKinematics(m_reverse, d_reverse, -q);

  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief Test q_ref positioning for joint composite
BOOST_AUTO_TEST_CASE(test_q_ref_composite)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  //////////////////////////////////////// Bodies
  g.addFrame("body1", BodyFrameGraph(pinocchio::Inertia::Identity()));
  g.addFrame("body2", BodyFrameGraph(pinocchio::Inertia::Identity()));

  /////////////////////////////////////// Joints
  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));
  pinocchio::SE3 poseBody2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.));

  JointCompositeGraph jmodel;
  pinocchio::SE3 jPose1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 3., 0.)); // from body to j1
  pinocchio::SE3 jPose2 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)); // from j1 to j2
  pinocchio::SE3 jPose3 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)); // from j2 to j3
  jmodel.addJoint(JointRevoluteGraph(Eigen::Vector3d::UnitX()), jPose1);
  jmodel.addJoint(JointRevoluteGraph(Eigen::Vector3d::UnitZ()), jPose2);
  jmodel.addJoint(JointPrismaticGraph(Eigen::Vector3d::UnitY()), jPose3);

  Eigen::VectorXd q_ref = Eigen::Vector3d::Zero();
  q_ref[0] = M_PI / 4;
  q_ref[1] = -M_PI / 3;
  q_ref[2] = 1.5;

  g.addJoint("body1_to_body2", jmodel, "body1", poseBody1, "body2", poseBody2, q_ref);

  pinocchio::Model m_forward = g.buildModel("body1", pinocchio::SE3::Identity());
  pinocchio::Data d_f(m_forward);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(m_forward.nq);
  q << M_PI / 2, M_PI / 5, 0.4;

  pinocchio::framesForwardKinematics(m_forward, d_f, q);
  //////////////////////////////////// Reverse model
  pinocchio::Model m_reverse =
    g.buildModel("body2", d_f.oMf[m_forward.getFrameId("body2", pinocchio::BODY)]);
  pinocchio::Data d_reverse(m_reverse);

  Eigen::VectorXd q_reverse(m_reverse.nq);
  q_reverse << -q[2], -q[1], -q[0];

  pinocchio::framesForwardKinematics(m_reverse, d_reverse, q_reverse);
  BOOST_CHECK(SE3isApprox(
    d_reverse.oMf[m_reverse.getFrameId("body1", pinocchio::BODY)],
    d_f.oMf[m_forward.getFrameId("body1", pinocchio::BODY)]));
}

/// @brief  Test the algorithm to merge 2 graphs
BOOST_AUTO_TEST_CASE(test_merge_graphs)
{
  using namespace pinocchio::graph;

  ModelGraph g;
  g.addFrame("torso", pinocchio::Inertia::Identity());
  g.addFrame("left_leg", pinocchio::Inertia::Identity());
  g.addFrame("right_leg", pinocchio::Inertia::Identity());
  g.addJoint(
    "torso2left_leg", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "left_leg", pinocchio::SE3::Identity());
  g.addJoint(
    "torso2right_leg", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "torso",
    pinocchio::SE3::Identity(), "right_leg", pinocchio::SE3::Identity());

  ModelGraph g1;
  g1.addFrame("upper_arm", pinocchio::Inertia::Identity());
  g1.addFrame("lower_arm", pinocchio::Inertia::Identity());
  g1.addFrame("hand", pinocchio::Inertia::Identity());
  g1.addJoint(
    "upper2lower", JointRevoluteGraph(Eigen::Vector3d::UnitZ()), "upper_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)), "lower_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.)));
  g1.addJoint(
    "lower2hand", JointRevoluteGraph(Eigen::Vector3d::UnitX()), "lower_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 1., 0.)), "hand",
    pinocchio::SE3::Identity());

  ModelGraph g_full = mergeGraphs(
    g, g1, "torso", "upper_arm",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)),
    JointRevoluteGraph(Eigen::Vector3d::UnitY()));

  pinocchio::Model m =
    g_full.buildModel("torso", pinocchio::SE3::Identity(), JointFreeFlyerGraph());

  BOOST_CHECK(m.parents[m.getJointId("torso2left_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("torso2right_leg")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("merging_joint")] == m.getJointId("root_joint"));
  BOOST_CHECK(m.parents[m.getJointId("g2/upper2lower")] == m.getJointId("merging_joint"));
  BOOST_CHECK(m.parents[m.getJointId("g2/lower2hand")] == m.getJointId("g2/upper2lower"));

  BOOST_CHECK(
    m.frames[m.getFrameId("g2/upper_arm", pinocchio::BODY)].placement
    == pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 4)));
}

BOOST_AUTO_TEST_CASE(test_lock_joint)
{
  using namespace pinocchio::graph;

  ModelGraph g = buildReversableModelGraph(JointRevoluteGraph(Eigen::Vector3d::UnitZ()));

  std::vector<std::string> joints_to_lock;
  joints_to_lock.push_back("body1_to_body2");
  std::vector<Eigen::VectorXd> ref_configs;
  Eigen::VectorXd q_ref(1);
  q_ref[0] = M_PI / 6;
  ref_configs.push_back(q_ref);
  ModelGraph g_locked = fixJointsGraph(g, joints_to_lock, ref_configs);

  pinocchio::Model m = g_locked.buildModel("body1", pinocchio::SE3::Identity());

  BOOST_CHECK(m.njoints == 1);

  pinocchio::SE3 poseBody1 =
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(2., 0., 0.));

  Eigen::AngleAxisd Rz(q_ref[0], Eigen::Vector3d::UnitZ());
  std::cout << Rz.toRotationMatrix() << std::endl;
  pinocchio::SE3 pose_fixed_joint = pinocchio::SE3(Rz.toRotationMatrix(), Eigen::Vector3d::Zero());

  BOOST_CHECK(m.frames[m.getFrameId("body1_to_body2", pinocchio::FIXED_JOINT)].placement.isApprox(
    poseBody1 * pose_fixed_joint));
}

BOOST_AUTO_TEST_SUITE_END()
