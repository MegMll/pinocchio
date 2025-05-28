//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_joints_graph_hpp__
#define __pinocchio_parsers_joints_graph_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  struct JointFixedGraph
  {
    pinocchio::SE3 joint_offset = pinocchio::SE3::Identity();
    int nq = 0;

    JointFixedGraph() = default;
    JointFixedGraph(const pinocchio::SE3 & pose)
    : joint_offset(pose)
    {
    }
  };

  struct JointRevoluteGraph
  {
    // rotation axis
    Eigen::Vector3d axis;
    int nq = 1;

    explicit JointRevoluteGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointRevoluteUnboundedGraph
  {
    Eigen::Vector3d axis;
    int nq = 2;

    explicit JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointPrismaticGraph
  {
    Eigen::Vector3d axis;
    int nq = 1;

    explicit JointPrismaticGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointFreeFlyerGraph
  {
    int nq = 7;

    JointFreeFlyerGraph() = default;
  };

  struct JointSphericalGraph
  {
    int nq = 4;

    JointSphericalGraph() = default;
  };

  // Flipped whne model is reversed ?
  struct JointSphericalZYXGraph
  {
    int nq = 3;

    JointSphericalZYXGraph() = default;
  };

  struct JointTranslationGraph
  {
    int nq = 3;

    JointTranslationGraph() = default;
  };

  struct JointPlanarGraph
  {
    int nq = 4;

    JointPlanarGraph() = default;
  };

  struct JointHelicalGraph
  {
    Eigen::Vector3d axis;
    double pitch;

    int nq = 1;
    JointHelicalGraph(const Eigen::Vector3d & ax, const double p)
    : axis(ax)
    , pitch(p)
    {
    }
  };

  struct JointUniversalGraph
  {
    Eigen::Vector3d axis1;
    Eigen::Vector3d axis2;

    int nq = 2;
    JointUniversalGraph(const Eigen::Vector3d & ax1, const Eigen::Vector3d & ax2)
    : axis1(ax1)
    , axis2(ax2)
    {
    }
  };

  // Forward declare
  struct JointCompositeGraph;
  // Forward declare
  struct JointMimicGraph;

  typedef boost::variant<
    JointFixedGraph,
    JointRevoluteGraph,
    JointRevoluteUnboundedGraph,
    JointPrismaticGraph,
    JointFreeFlyerGraph,
    JointSphericalGraph,
    JointSphericalZYXGraph,
    JointTranslationGraph,
    JointPlanarGraph,
    JointHelicalGraph,
    JointUniversalGraph,
    boost::recursive_wrapper<JointCompositeGraph>,
    boost::recursive_wrapper<JointMimicGraph>>
    JointGraphVariant;

  struct JointMimicGraph
  {
    std::string primary_name;

    JointGraphVariant secondary_joint;
    double scaling;
    double offset;

    int nq = 0;

    JointMimicGraph() = default;

    JointMimicGraph(
      const JointGraphVariant & jmodel_secondary,
      const std::string & name_primary,
      const double scaling_,
      const double offset_)
    : primary_name(name_primary)
    , secondary_joint(jmodel_secondary)
    , scaling(scaling_)
    , offset(offset_)
    {
    }
  };

  struct JointCompositeGraph
  {
    std::vector<JointGraphVariant> joints;
    std::vector<SE3> jointsPlacements;

    int nq = 0;
    JointCompositeGraph() = default;

    JointCompositeGraph(const JointGraphVariant & j, const SE3 & jPose)
    {
      joints.push_back(j);
      jointsPlacements.push_back(jPose);
      nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
    }

    JointCompositeGraph(const std::vector<JointGraphVariant> & js, const std::vector<SE3> & jPoses)
    : joints(js)
    , jointsPlacements(jPoses)
    {
      for (const auto & j : js)
        nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
    }

    void addJoint(const JointGraphVariant & jm, const SE3 & pose = SE3::Identity())
    {
      joints.push_back(jm);
      jointsPlacements.push_back(pose);
      nq += boost::apply_visitor([](const auto & j) { return j.nq; }, jm);
    }
  };
} // namespace pinocchio

#endif
