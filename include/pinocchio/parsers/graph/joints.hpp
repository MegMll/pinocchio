//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_joints_graph_hpp__
#define __pinocchio_parsers_joints_graph_hpp__

#include "pinocchio/parsers/config.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  namespace graph
  {
    struct PINOCCHIO_PARSERS_DLLAPI JointLimits
    {
      // Max effort
      Eigen::VectorXd maxEffort;
      // Max velocity
      Eigen::VectorXd maxVel;
      // Max position
      Eigen::VectorXd maxConfig;
      // Min position
      Eigen::VectorXd minConfig;

      // friction applied in this joint
      Eigen::VectorXd friction;
      // Damping applied by this joint.
      Eigen::VectorXd damping;

      // Armature inertia created by this joint
      Eigen::VectorXd armature;
      // Dry friction.
      double frictionLoss = 0.;

      JointLimits() = default;

      template<int Nq, int Nv>
      void setDimensions();

      void append(const JointLimits & jlimit, const int nq, const int nv);
    };

    struct JointFixedGraph
    {
      pinocchio::SE3 joint_offset = pinocchio::SE3::Identity();
      static constexpr int nq = 0;
      static constexpr int nv = 0;

      JointFixedGraph() = default;
      JointFixedGraph(const pinocchio::SE3 & pose)
      : joint_offset(pose)
      {
      }

      bool operator==(const JointFixedGraph & other) const
      {
        return joint_offset == other.joint_offset && nq == other.nq;
      }
    };

    struct JointRevoluteGraph
    {
      // rotation axis
      Eigen::Vector3d axis;
      static constexpr int nq = 1;
      static constexpr int nv = 1;

      explicit JointRevoluteGraph(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointRevoluteGraph & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointRevoluteUnboundedGraph
    {
      Eigen::Vector3d axis;
      static constexpr int nq = 2;
      static constexpr int nv = 1;

      explicit JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointRevoluteUnboundedGraph & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointPrismaticGraph
    {
      Eigen::Vector3d axis;
      static constexpr int nq = 1;
      static constexpr int nv = 1;

      explicit JointPrismaticGraph(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointPrismaticGraph & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointFreeFlyerGraph
    {
      static constexpr int nq = 7;
      static constexpr int nv = 6;

      JointFreeFlyerGraph() = default;

      bool operator==(const JointFreeFlyerGraph &) const
      {
        return true;
      }
    };

    struct JointSphericalGraph
    {
      static constexpr int nq = 4;
      static constexpr int nv = 3;

      JointSphericalGraph() = default;

      bool operator==(const JointSphericalGraph &) const
      {
        return true;
      }
    };

    struct JointSphericalZYXGraph
    {
      static constexpr int nq = 3;
      static constexpr int nv = 3;

      JointSphericalZYXGraph() = default;

      bool operator==(const JointSphericalZYXGraph &) const
      {
        return true;
      }
    };

    struct JointTranslationGraph
    {
      static constexpr int nq = 3;
      static constexpr int nv = 3;

      JointTranslationGraph() = default;

      bool operator==(const JointTranslationGraph &) const
      {
        return true;
      }
    };

    struct JointPlanarGraph
    {
      static constexpr int nq = 4;
      static constexpr int nv = 3;

      JointPlanarGraph() = default;

      bool operator==(const JointPlanarGraph &) const
      {
        return true;
      }
    };

    struct JointHelicalGraph
    {
      Eigen::Vector3d axis;
      double pitch;

      static constexpr int nq = 1;
      static constexpr int nv = 1;

      JointHelicalGraph(const Eigen::Vector3d & ax, const double p)
      : axis(ax)
      , pitch(p)
      {
      }

      bool operator==(const JointHelicalGraph & other) const
      {
        return axis == other.axis && pitch == other.pitch;
      }
    };

    struct JointUniversalGraph
    {
      Eigen::Vector3d axis1;
      Eigen::Vector3d axis2;

      static constexpr int nq = 2;
      static constexpr int nv = 2;

      JointUniversalGraph(const Eigen::Vector3d & ax1, const Eigen::Vector3d & ax2)
      : axis1(ax1)
      , axis2(ax2)
      {
      }

      bool operator==(const JointUniversalGraph & other) const
      {
        return axis1 == other.axis1 && axis2 == other.axis2;
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

      static constexpr int nq = 0;
      static constexpr int nv = 0;

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

      bool operator==(const JointMimicGraph & other) const
      {
        return primary_name == other.primary_name && scaling == other.scaling
               && offset == other.offset && secondary_joint == other.secondary_joint;
      }
    };

    struct JointCompositeGraph
    {
      std::vector<JointGraphVariant> joints;
      std::vector<SE3> jointsPlacements;

      int nq = 0;
      int nv = 0;

      JointCompositeGraph() = default;

      JointCompositeGraph(const JointGraphVariant & j, const SE3 & jPose)
      {
        joints.push_back(j);
        jointsPlacements.push_back(jPose);
        nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
        nv += boost::apply_visitor([](const auto & j_) { return j_.nv; }, j);
      }

      JointCompositeGraph(
        const std::vector<JointGraphVariant> & js, const std::vector<SE3> & jPoses)
      : joints(js)
      , jointsPlacements(jPoses)
      {
        for (const auto & j : js)
        {
          nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
          nv += boost::apply_visitor([](const auto & j_) { return j_.nv; }, j);
        }
      }

      void addJoint(const JointGraphVariant & jm, const SE3 & pose = SE3::Identity())
      {
        joints.push_back(jm);
        jointsPlacements.push_back(pose);
        nq += boost::apply_visitor([](const auto & j) { return j.nq; }, jm);
        nv += boost::apply_visitor([](const auto & j) { return j.nv; }, jm);
      }

      bool operator==(const JointCompositeGraph & other) const
      {
        return joints == other.joints && jointsPlacements == other.jointsPlacements
               && nq == other.nq && nv == other.nv;
      }
    };
  } // namespace graph
} // namespace pinocchio

#endif
