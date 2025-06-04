//
// Copyright (c) 2025 INRIA
//

#include <boost/test/unit_test.hpp>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/graph/model-configuration-converter.hpp"
#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"

BOOST_AUTO_TEST_SUITE(ModelConfigurationConverter)

// TODO: factorize
/// function isApprox better to avoid problem with zero precision
bool SE3isApprox(
  const pinocchio::SE3 & s1,
  const pinocchio::SE3 & s2,
  const double prec = Eigen::NumTraits<double>::dummy_precision())
{
  return s1.rotation().isApprox(s2.rotation())
         && (s1.translation() - s2.translation()).isZero(prec);
}

BOOST_AUTO_TEST_CASE(test_create_converter)
{
  // Create the following model:
  //      j3    j4
  //   b4----b3----b5
  //        | j2
  //        b2
  //        | j1 (unbounded)
  //        b1
  //
  // The model will be created from b1, b4 and b5 and will have the following joint order:
  // - b1:    j1,  j2, j3, j4
  // - b4:    j3,  j2, j1, j4
  // - b5:    j4,  j2, j1, j3
  // - b1_ff: jff, j1, j2, j3, j4
  //
  // We should have the following configuration and tangent vector:
  // - configuration:
  //   - b1:    [j1[0], j1[2], j2, j3, j4]
  //   - b4:    [j3, j2, j1[0], j1[2], j4]
  //   - b5:    [j4, j2, j1[0], j1[2], j3]
  //   - b1_ff: [jff[0..7], j1[0], j1[2], j2, j3, j4]
  // - tangent:
  //   - b1:    [j1, j2, j3, j4]
  //   - b4:    [j3, j2, j1, j4]
  //   - b5:    [j4, j2, j1, j3]
  //   - b1_ff: [jff[0..8], j1[0], j1[2], j2, j3, j4]

  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addBody("b4", I_I);
  g.addBody("b5", I_I);

  pinocchio::graph::JointRevoluteGraph joint(Eigen::Vector3d::UnitX());
  g.addJoint(
    "j1", pinocchio::graph::JointRevoluteUnboundedGraph(Eigen::Vector3d::UnitX()), "b1", X_I, "b2",
    X_I);
  g.addJoint("j2", joint, "b2", X_I, "b3", X_I);
  g.addJoint("j3", joint, "b3", X_I, "b4", X_I);
  g.addJoint("j4", joint, "b3", X_I, "b5", X_I);

  auto b1_model = g.buildModel("b1", X_I);
  auto b4_model = g.buildModel("b4", X_I);
  auto b5_model = g.buildModel("b5", X_I);
  auto b1_ff_model = g.buildModel(
    "b1", X_I, pinocchio::graph::JointGraphVariant(pinocchio::graph::JointFreeFlyerGraph()), "ff");

  auto b1_to_b4_converter = pinocchio::graph::createConverter(b1_model, b4_model, g);
  auto b1_to_b5_converter = pinocchio::graph::createConverter(b1_model, b5_model, g);
  auto b4_to_b5_converter = pinocchio::graph::createConverter(b4_model, b5_model, g);
  auto b1_ff_to_b1_converter = pinocchio::graph::createConverter(b1_ff_model, b1_model, g);

  // Test b1 to b4
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter.configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter.tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b4_converter.joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[0].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[0].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[1].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.joint_mapping[1].same_direction, false);

  //   j3
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[2].idx_qs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[2].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[2].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.joint_mapping[2].same_direction, false);

  //   j4
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.configuration_mapping[3].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b4_converter.tangent_mapping[3].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b4_converter.joint_mapping[3].same_direction, true);

  // Test b1 to b5
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter.configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter.tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_to_b5_converter.joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[0].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[0].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[1].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.joint_mapping[1].same_direction, false);

  //   j3
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[2].idx_qs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[2].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[2].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.configuration_mapping[3].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b1_to_b5_converter.tangent_mapping[3].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_to_b5_converter.joint_mapping[3].same_direction, false);

  // Test b4 to b5
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter.configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter.tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b4_to_b5_converter.joint_mapping.size(), 4);

  //   j3
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[0].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[0].idx_qs_source, 0);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[0].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[0].idx_vs_source, 0);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[0].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.joint_mapping[0].same_direction, false);

  //   j2
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[1].idx_qs_source, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[1].idx_qs_target, 1);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[1].idx_vs_source, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.joint_mapping[1].same_direction, true);

  //   j1
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[2].nq, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[2].idx_qs_source, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[2].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[2].idx_vs_source, 2);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[2].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[3].idx_qs_source, 4);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.configuration_mapping[3].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[3].idx_vs_source, 3);
  BOOST_CHECK_EQUAL(b4_to_b5_converter.tangent_mapping[3].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b4_to_b5_converter.joint_mapping[3].same_direction, false);

  // Test b1_ff to b1
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter.configuration_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter.tangent_mapping.size(), 4);
  BOOST_REQUIRE_EQUAL(b1_ff_to_b1_converter.joint_mapping.size(), 4);

  //   j1
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[0].nq, 2);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[0].idx_qs_source, 7);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[0].idx_qs_target, 0);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[0].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[0].idx_vs_source, 6);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[0].idx_vs_target, 0);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.joint_mapping[0].same_direction, true);

  //   j2
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[1].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[1].idx_qs_source, 9);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[1].idx_qs_target, 2);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[1].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[1].idx_vs_source, 7);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[1].idx_vs_target, 1);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.joint_mapping[1].same_direction, true);

  //   j3
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[2].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[2].idx_qs_source, 10);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[2].idx_qs_target, 3);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[2].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[2].idx_vs_source, 8);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[2].idx_vs_target, 2);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.joint_mapping[2].same_direction, true);

  //   j4
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[3].nq, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[3].idx_qs_source, 11);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.configuration_mapping[3].idx_qs_target, 4);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[3].nv, 1);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[3].idx_vs_source, 9);
  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.tangent_mapping[3].idx_vs_target, 3);

  BOOST_CHECK_EQUAL(b1_ff_to_b1_converter.joint_mapping[3].same_direction, true);
}

BOOST_AUTO_TEST_CASE(test_convert_configuration)
{
  pinocchio::graph::ModelGraph g;
  pinocchio::Inertia I_I(pinocchio::Inertia::Identity());
  pinocchio::SE3 X_I(pinocchio::SE3::Identity());

  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addBody("b4", I_I);
  g.addBody("b5", I_I);
  g.addBody("b6", I_I);
  g.addBody("b7", I_I);
  g.addBody("b8", I_I);
  g.addBody("b9", I_I);
  g.addJoint(
    "j1", pinocchio::graph::JointRevoluteGraph(Eigen::Vector3d::UnitX()), "b1",
    pinocchio::SE3::Random(), "b2", pinocchio::SE3::Random());
  g.addJoint(
    "j2", pinocchio::graph::JointFreeFlyerGraph(), "b2", pinocchio::SE3::Random(), "b3",
    pinocchio::SE3::Random());
  g.addJoint(
    "j3", pinocchio::graph::JointSphericalGraph(), "b3", pinocchio::SE3::Random(), "b4",
    pinocchio::SE3::Random());
  g.addJoint(
    "j4", pinocchio::graph::JointUniversalGraph(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()),
    "b4", pinocchio::SE3::Random(), "b5", pinocchio::SE3::Random());
  g.addJoint(
    "j5", pinocchio::graph::JointRevoluteUnboundedGraph(Eigen::Vector3d::UnitX()), "b5",
    pinocchio::SE3::Random(), "b6", pinocchio::SE3::Random());
  g.addJoint(
    "j6", pinocchio::graph::JointPrismaticGraph(Eigen::Vector3d::UnitX()), "b6",
    pinocchio::SE3::Random(), "b7", pinocchio::SE3::Random());
  g.addJoint(
    "j7", pinocchio::graph::JointHelicalGraph(Eigen::Vector3d::UnitX(), 0.1), "b7",
    pinocchio::SE3::Random(), "b8", pinocchio::SE3::Random());
  g.addJoint(
    "j8", pinocchio::graph::JointTranslationGraph(), "b8", pinocchio::SE3::Random(), "b9",
    pinocchio::SE3::Random());

  const auto model_a = g.buildModel("b1", X_I);
  pinocchio::Data data_a(model_a);
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model_a.nq);
  const Eigen::VectorXd q_a = pinocchio::randomConfiguration(model_a, -qmax, qmax);
  pinocchio::framesForwardKinematics(model_a, data_a, q_a);

  // Check joint mapping and backward conversion
  {
    auto model_b = g.buildModel("b9", data_a.oMf[model_a.getFrameId("b9", pinocchio::BODY)]);
    pinocchio::Data data_b(model_b);
    Eigen::VectorXd q_b = pinocchio::neutral(model_b);
    auto a_to_b_converter = pinocchio::graph::createConverter(model_a, model_b, g);
    a_to_b_converter.convertConfiguration(q_a, q_b);
    pinocchio::framesForwardKinematics(model_b, data_b, q_b);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(
          SE3isApprox(data_a.oMf[i], data_b.oMf[model_b.getFrameId(frame.name, frame.type)]));
      }
    }
  }

  // Check forward conversion
  {
    pinocchio::Data data_a2(model_a);
    Eigen::VectorXd q_a2 = pinocchio::neutral(model_a);
    auto a_to_a_converter = pinocchio::graph::createConverter(model_a, model_a, g);
    a_to_a_converter.convertConfiguration(q_a, q_a2);
    pinocchio::framesForwardKinematics(model_a, data_a2, q_a2);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(SE3isApprox(data_a.oMf[i], data_a2.oMf[i]));
      }
    }
  }

  // Check forward conversion with custom root joint
  {
    auto model_a_ff = g.buildModel(
      "b1", X_I, pinocchio::graph::JointGraphVariant(pinocchio::graph::JointFreeFlyerGraph()),
      "ff");
    pinocchio::Data data_a_ff(model_a_ff);
    const Eigen::VectorXd qmax_ff = Eigen::VectorXd::Ones(model_a_ff.nq);
    Eigen::VectorXd q_a_ff = pinocchio::randomConfiguration(model_a_ff, -qmax_ff, qmax_ff);
    q_a_ff.head<7>() << 0., 0., 0., 0., 0., 0., 1.;
    pinocchio::framesForwardKinematics(model_a_ff, data_a_ff, q_a_ff);
    auto a_ff_to_a_converter = pinocchio::graph::createConverter(model_a_ff, model_a, g);
    a_ff_to_a_converter.convertConfiguration(q_a_ff, q_a);
    pinocchio::framesForwardKinematics(model_a, data_a, q_a);
    for (std::size_t i = 0; i < model_a.frames.size(); ++i)
    {
      const auto & frame = model_a.frames[i];
      if (frame.type == pinocchio::FrameType::BODY)
      {
        BOOST_CHECK(
          SE3isApprox(data_a.oMf[i], data_a_ff.oMf[model_a_ff.getFrameId(frame.name, frame.type)]));
      }
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
