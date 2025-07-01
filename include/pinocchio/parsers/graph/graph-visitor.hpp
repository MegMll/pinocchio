//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_graph_visitor_hpp__
#define __pinocchio_parsers_graph_graph_visitor_hpp__

#include "pinocchio/macros.hpp"

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/multibody/joint/joint-collection.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/joints.hpp"

#include <Eigen/Core>

#include <boost/graph/depth_first_search.hpp>

#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>

#include <utility>
#include <stdexcept>

namespace pinocchio
{
  namespace graph
  {
    namespace internal
    {
      struct MakeJointLimitsDefaultVisitor : public boost::static_visitor<JointLimits>
      {
        template<typename Joint>
        JointLimits operator()(const Joint &) const
        {
          JointLimits jlimit;
          jlimit.setDimensions<Joint::nq, Joint::nv>();
          return jlimit;
        }

        JointLimits operator()(const JointComposite & j) const
        {
          JointLimits jlimit = boost::apply_visitor(*this, j.joints[0]);

          for (size_t i = 1; i < j.joints.size(); i++)
          {
            int nq = boost::apply_visitor([](const auto & j) { return j.nq; }, j.joints[i]);
            int nv = boost::apply_visitor([](const auto & j) { return j.nv; }, j.joints[i]);
            jlimit.append(boost::apply_visitor(*this, j.joints[i]), nq, nv);
          }
          return jlimit;
        }
      };

      struct ReverseJointLimitsVisitor : public boost::static_visitor<JointLimits>
      {
        const JointLimits jlimit;
        ReverseJointLimitsVisitor(const JointLimits & jlimit)
        : jlimit(jlimit)
        {
        }

        // For revolute, revolute unbounded, prismatic, helical, translation, mimic, fixed
        template<typename Joint>
        JointLimits operator()(const Joint &) const
        {
          JointLimits jlimit_return = jlimit;
          jlimit_return.maxConfig = -jlimit.minConfig;
          jlimit_return.minConfig = -jlimit.maxConfig;

          return jlimit_return;
        }

        // For freeFlyer = no changes, except for translation limits.
        JointLimits operator()(const JointFreeFlyer &) const
        {

          return jlimit;
        }

        // For spherical = no changes
        JointLimits operator()(const JointSpherical &) const
        {
          return jlimit;
        }

        // universal = inverse axis config, so inverse limits
        JointLimits operator()(const JointUniversal & j) const
        {
          JointLimits jlimit_return = jlimit;
          for (int i = 0; i < j.nq; i++)
          {
            jlimit_return.maxConfig[i] = jlimit.maxConfig[j.nq - 1 - i];
            jlimit_return.minConfig[i] = jlimit.minConfig[j.nq - 1 - i];
          }

          for (int i = 0; i < j.nv; i++)
          {
            jlimit_return.maxEffort[i] = jlimit.maxEffort[j.nv - 1 - i];
            jlimit_return.maxVel[i] = jlimit.maxVel[j.nv - 1 - i];

            jlimit_return.friction[i] = jlimit.friction[j.nv - 1 - i];
            jlimit_return.damping[i] = jlimit.damping[j.nv - 1 - i];

            jlimit_return.armature[i] = jlimit.armature[j.nv - 1 - i];
          }

          return jlimit_return;
        }

        // ZYX = inverse order and max becomes min
        JointLimits operator()(const JointSphericalZYX & j) const
        {
          JointLimits jlimit_return = jlimit;
          for (int i = 0; i < j.nq; i++)
          {
            jlimit_return.maxConfig[i] = -jlimit.minConfig[j.nq - 1 - i];
            jlimit_return.minConfig[i] = -jlimit.maxConfig[j.nq - 1 - i];
          }

          for (int i = 0; i < j.nv; i++)
          {
            jlimit_return.maxEffort[i] = jlimit.maxEffort[j.nv - 1 - i];
            jlimit_return.maxVel[i] = jlimit.maxVel[j.nv - 1 - i];

            jlimit_return.friction[i] = jlimit.friction[j.nv - 1 - i];
            jlimit_return.damping[i] = jlimit.damping[j.nv - 1 - i];

            jlimit_return.armature[i] = jlimit.armature[j.nv - 1 - i];
          }

          return jlimit_return;
        }

        // Composite = inverse order inside and apply visitor on each joint
        JointLimits operator()(const JointComposite & j) const
        {
          int nq_curr =
            boost::apply_visitor([](const auto & j_) { return j_.nq; }, j.joints.back());
          int index_back_config = j.nq - nq_curr;
          int nv_curr =
            boost::apply_visitor([](const auto & j_) { return j_.nv; }, j.joints.back());
          int index_back_tangent = j.nv - nv_curr;

          int i = static_cast<int>(j.joints.size() - 1);

          auto createAndFillJointLimits = [&](
                                            int nq_curr, int nv_curr, int index_back_config,
                                            int index_back_tangent) -> JointLimits {
            // Step 1: Initialize jtemp using the visitor
            JointLimits jtemp = boost::apply_visitor(MakeJointLimitsDefaultVisitor(), j.joints[i]);

            jtemp.minConfig.conservativeResize(nq_curr);
            jtemp.maxConfig.conservativeResize(nq_curr);
            jtemp.maxEffort.conservativeResize(nv_curr);
            jtemp.maxVel.conservativeResize(nv_curr);
            jtemp.friction.conservativeResize(nv_curr);
            jtemp.damping.conservativeResize(nv_curr);
            jtemp.armature.conservativeResize(nv_curr);

            // Step 2: Copy segments from jlimit into jtemp
            jtemp.minConfig.segment(0, nq_curr) =
              jlimit.minConfig.segment(index_back_config, nq_curr);
            jtemp.maxConfig.segment(0, nq_curr) =
              jlimit.maxConfig.segment(index_back_config, nq_curr);

            jtemp.maxEffort.segment(0, nv_curr) =
              jlimit.maxEffort.segment(index_back_tangent, nv_curr);
            jtemp.maxVel.segment(0, nv_curr) = jlimit.maxVel.segment(index_back_tangent, nv_curr);

            jtemp.friction.segment(0, nv_curr) =
              jlimit.friction.segment(index_back_tangent, nv_curr);
            jtemp.damping.segment(0, nv_curr) = jlimit.damping.segment(index_back_tangent, nv_curr);

            jtemp.armature.segment(0, nv_curr) =
              jlimit.armature.segment(index_back_tangent, nv_curr);

            return jtemp;
          };

          JointLimits jtemp =
            createAndFillJointLimits(nq_curr, nv_curr, index_back_config, index_back_tangent);

          JointLimits jlimit_return =
            boost::apply_visitor(ReverseJointLimitsVisitor(jtemp), j.joints.back());
          // Do the same for the rest
          for (i = static_cast<int>(j.joints.size() - 2); i >= 0; i--)
          {
            nq_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nq; }, j.joints[static_cast<size_t>(i)]);
            index_back_config -= nq_curr;
            nv_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nv; }, j.joints[static_cast<size_t>(i)]);
            index_back_tangent -= nv_curr;

            JointLimits jtemp_ =
              createAndFillJointLimits(nq_curr, nv_curr, index_back_config, index_back_tangent);
            jlimit_return.append(
              boost::apply_visitor(
                ReverseJointLimitsVisitor(jtemp_), j.joints[static_cast<size_t>(i)]),
              nq_curr, nv_curr);
          }
          return jlimit_return;
        }
      };

      struct ReverseJointGraphVisitor : public boost::static_visitor<std::pair<JointVariant, SE3>>
      {
        using ReturnType = std::pair<JointVariant, SE3>;

        ReturnType operator()(const JointRevolute & joint) const
        {
          return {JointRevolute(joint.axis), SE3::Identity()};
        }

        ReturnType operator()(const JointRevoluteUnbounded & joint) const
        {
          return {JointRevoluteUnbounded(joint.axis), SE3::Identity()};
        }

        ReturnType operator()(const JointPrismatic & joint) const
        {
          return {JointPrismatic(joint.axis), SE3::Identity()};
        }
        ReturnType operator()(const JointFixed & joint) const
        {
          return {JointFixed(joint.joint_offset.inverse()), SE3::Identity()};
        }
        ReturnType operator()(const JointFreeFlyer &) const
        {

          return {JointFreeFlyer(), SE3::Identity()};
        }
        ReturnType operator()(const JointSpherical &) const
        {
          return {JointSpherical(), SE3::Identity()};
        }
        ReturnType operator()(const JointSphericalZYX &) const
        {
          return {JointSphericalZYX(), SE3::Identity()};
        }
        ReturnType operator()(const JointTranslation &) const
        {
          return {JointTranslation(), SE3::Identity()};
        }
        ReturnType operator()(const JointPlanar &) const
        {
          return {JointPlanar(), SE3::Identity()};
        }
        ReturnType operator()(const JointHelical & joint) const
        {
          return {JointHelical(joint.axis, joint.pitch), SE3::Identity()};
        }
        ReturnType operator()(const JointUniversal & joint) const
        {
          return {JointUniversal(-joint.axis2, -joint.axis1), SE3::Identity()};
        }
        ReturnType operator()(const JointMimic & joint) const
        {
          return {joint, SE3::Identity()};
        }
        ReturnType operator()(const JointComposite & joint) const
        {
          JointComposite jReturn;
          auto temp = boost::apply_visitor(*this, joint.joints.back());
          jReturn.addJoint(temp.first, temp.second * SE3::Identity());
          // Reverse joints
          for (int i = static_cast<int>(joint.joints.size() - 2); i >= 0; i--)
          {
            temp = boost::apply_visitor(*this, joint.joints[static_cast<size_t>(i)]);
            jReturn.addJoint(
              temp.first,
              temp.second * joint.jointsPlacements[static_cast<size_t>(i + 1)].inverse());
          }
          return {jReturn, joint.jointsPlacements[0].inverse()};
        }
      };

      struct CreateJointModelVisitor : public boost::static_visitor<JointModel>
      {
        typedef JointModelTpl<double> JointModel;
        typedef JointCollectionDefaultTpl<double> JointCollectionDefault;

        // Joint Revolute
        typedef typename JointCollectionDefault::JointModelRX JointModelRX;
        typedef typename JointCollectionDefault::JointModelRY JointModelRY;
        typedef typename JointCollectionDefault::JointModelRZ JointModelRZ;

        // Joint Revolute Unaligned
        typedef
          typename JointCollectionDefault::JointModelRevoluteUnaligned JointModelRevoluteUnaligned;

        // Joint Revolute UBounded
        typedef typename JointCollectionDefault::JointModelRUBX JointModelRUBX;
        typedef typename JointCollectionDefault::JointModelRUBY JointModelRUBY;
        typedef typename JointCollectionDefault::JointModelRUBZ JointModelRUBZ;

        // Joint Revolute Unbounded Unaligned
        typedef typename JointCollectionDefault::JointModelRevoluteUnboundedUnaligned
          JointModelRevoluteUnboundedUnaligned;

        // Joint Prismatic
        typedef typename JointCollectionDefault::JointModelPX JointModelPX;
        typedef typename JointCollectionDefault::JointModelPY JointModelPY;
        typedef typename JointCollectionDefault::JointModelPZ JointModelPZ;

        // Joint Prismatic Unaligned
        typedef typename JointCollectionDefault::JointModelPrismaticUnaligned
          JointModelPrismaticUnaligned;

        // Joint Spherical
        typedef typename JointCollectionDefault::JointModelSpherical JointModelSpherical;

        // Joint Spherical ZYX
        typedef typename JointCollectionDefault::JointModelSphericalZYX JointModelSphericalZYX;

        // Joint Translation
        typedef typename JointCollectionDefault::JointModelTranslation JointModelTranslation;

        // Joint FreeFlyer
        typedef typename JointCollectionDefault::JointModelFreeFlyer JointModelFreeFlyer;

        // Joint Planar
        typedef typename JointCollectionDefault::JointModelPlanar JointModelPlanar;

        // Joint Composite
        typedef typename JointCollectionDefault::JointModelComposite JointModelComposite;

        // Joint Mimic
        typedef typename JointCollectionDefault::JointModelMimic JointModelMimic;

        // Joint Helical
        typedef typename JointCollectionDefault::JointModelHx JointModelHx;
        typedef typename JointCollectionDefault::JointModelHy JointModelHy;
        typedef typename JointCollectionDefault::JointModelHz JointModelHz;

        // Joint Helical Unaligned
        typedef
          typename JointCollectionDefault::JointModelHelicalUnaligned JointModelHelicalUnaligned;

        // Joint Universal
        typedef typename JointCollectionDefault::JointModelUniversal JointModelUniversal;

        typedef JointModel ReturnType;

        ReturnType operator()(const JointFixed & /*joint*/) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - cannot create a fixed joint. In pinocchio, fixed joints are frame.");
        }
        ReturnType operator()(const JointRevolute & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelRX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelRY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelRZ();
          }
          else
          {
            return JointModelRevoluteUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointRevoluteUnbounded & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelRUBX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelRUBY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelRUBZ();
          }
          else
          {
            return JointModelRevoluteUnboundedUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointPrismatic & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelPX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelPY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelPZ();
          }
          else
          {
            return JointModelPrismaticUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointHelical & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelHX(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelHY(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelHZ(joint.pitch);
          }
          else
          {
            return JointModelHelicalUnaligned(joint.axis, joint.pitch);
          }
        }
        ReturnType operator()(const JointFreeFlyer & /*joint*/) const
        {
          return JointModelFreeFlyer();
        }
        ReturnType operator()(const JointTranslation & /*joint*/) const
        {
          return JointModelTranslation();
        }
        ReturnType operator()(const JointPlanar & /*joint*/) const
        {
          return JointModelPlanar();
        }
        ReturnType operator()(const JointSpherical & /*joint*/) const
        {
          return JointModelSpherical();
        }
        ReturnType operator()(const JointSphericalZYX & /*joint*/) const
        {
          return JointModelSphericalZYX();
        }
        ReturnType operator()(const JointUniversal & joint) const
        {
          return JointModelUniversal(joint.axis1, joint.axis2);
        }

        ReturnType operator()(const JointMimic & joint) const
        {
          return boost::apply_visitor(*this, joint.secondary_joint);
        }
        ReturnType operator()(const JointComposite & joint) const
        {
          JointModelComposite jmodel;
          for (size_t i = 0; i < joint.joints.size(); i++)
            jmodel.addJoint(
              boost::apply_visitor(*this, joint.joints[i]), joint.jointsPlacements[i]);

          return jmodel;
        }
      };

      struct AddJointModelVisitor : public boost::static_visitor<>
      {
        const ModelGraphVertex & source_vertex;
        const ModelGraphVertex & target_vertex;
        const ModelGraphEdge & edge;
        Model & model;
        CreateJointModelVisitor cjm;

        AddJointModelVisitor(
          const ModelGraphVertex & source,
          const ModelGraphVertex & target,
          const ModelGraphEdge & edge_,
          Model & model_)
        : source_vertex(source)
        , target_vertex(target)
        , edge(edge_)
        , model(model_)
        {
        }

        template<typename JointGraph, typename FrameGraph>
        void operator()(const JointGraph & /*joint*/, const FrameGraph & /*f_*/)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - Invalid joint between non body frames. Non body frames can "
            "only be added with Fixed joint");
        }

        template<typename JointGraph>
        void operator()(const JointGraph & joint, const BodyFrame & b_f)
        {
          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr) // body frame is index 0
                                                                      // in variant
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph -Invalid joint between a body and a non body frame.");

          const SE3 & joint_pose = edge.source_to_joint;
          const SE3 & body_pose = edge.joint_to_target;

          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint, cjm(joint), previous_body.placement * joint_pose, edge.name,
            edge.jlimit.maxEffort, edge.jlimit.maxVel, edge.jlimit.minConfig, edge.jlimit.maxConfig,
            edge.jlimit.friction, edge.jlimit.damping);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);

          // armature
          model.armature.segment(model.joints[j_id].idx_v(), model.joints[j_id].nv()) =
            edge.jlimit.armature;
        }

        template<typename FrameGraph>
        void operator()(const JointFixed & joint, const FrameGraph & f_)
        {
          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];

          model.addFrame(Frame(
            target_vertex.name, previous_body.parentJoint,
            previous_body.placement * edge.source_to_joint * joint.joint_offset
              * edge.joint_to_target,
            f_.f_type));
        }

        void operator()(const JointMimic & joint, const BodyFrame & b_f)
        {
          if (!edge.forward)
            PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - JointMimic cannot be reversed.");

          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Invalid joint between a body and a non body frame.");

          if (!model.existJointName(joint.primary_name))
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - The parent joint of the mimic node is not in the kinematic tree");

          const auto primary_joint = model.joints[model.getJointId(joint.primary_name)];

          const SE3 & joint_pose = edge.source_to_joint;
          const SE3 & body_pose = edge.joint_to_target;

          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint,
            JointModelMimic(cjm(joint), primary_joint, joint.scaling, joint.offset),
            previous_body.placement * joint_pose, edge.name);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);
        }

        void operator()(const JointFixed & joint, const BodyFrame & b_f)
        {
          // Need to check what's vertex the edge is coming from. If it's a body, then we add
          // both the fixed joint frame and a body frame. Otherwise, it's a "fake" fixed joint
          // That's only used for graph construction, so we just add the body frame.
          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr)
          {
            FrameIndex prev_f_id = model.getFrameId(source_vertex.name, OP_FRAME);
            if (prev_f_id == model.frames.size())
              prev_f_id = model.getFrameId(source_vertex.name, SENSOR);

            const Frame previous_frame = model.frames[prev_f_id];
            model.addFrame(Frame(
              target_vertex.name, previous_frame.parentJoint,
              previous_frame.placement * edge.source_to_joint * joint.joint_offset
                * edge.joint_to_target,
              BODY, b_f.inertia));
          }
          else
          {
            const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
            // Don't add a new joint in the model — create the fixed_joint frame
            const FrameIndex f_id = model.addFrame(Frame(
              edge.name, previous_body.parentJoint,
              previous_body.placement * edge.source_to_joint * joint.joint_offset, FIXED_JOINT,
              b_f.inertia));
            SE3 body_placement = previous_body.placement * edge.source_to_joint * joint.joint_offset
                                 * edge.joint_to_target;
            model.addBodyFrame(
              target_vertex.name, previous_body.parentJoint, body_placement, (int)f_id);
          }
        }
      };

      struct ReverseQVisitor : boost::static_visitor<Eigen::VectorXd>
      {
        const Eigen::VectorXd q;

        ReverseQVisitor(const Eigen::VectorXd q_)
        : q(q_)
        {
        }

        Eigen::VectorXd operator()(const JointRevolute &) const
        {
          return -q;
        }

        Eigen::VectorXd operator()(const JointRevoluteUnbounded &) const
        {
          Eigen::Vector2d q_rev;
          q_rev << q[0], q[1];

          return q_rev;
        }

        Eigen::VectorXd operator()(const JointPrismatic &) const
        {
          return -q;
        }

        Eigen::VectorXd operator()(const JointFixed &) const
        {
          return q;
        }

        Eigen::VectorXd operator()(const JointFreeFlyer &) const
        {
          // Compute the inverse rotation and extract the ZYX euler angles
          JointModelFreeFlyer jmodel;
          jmodel.setIndexes(0, 0, 0);
          JointDataFreeFlyer jdata;
          jmodel.calc(jdata, q);

          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(7);
          Eigen::Quaterniond q_temp(q[6], q[3], q[4], q[5]);
          q_rev.segment<3>(0) = -jdata.M.rotation().transpose() * q.segment(0, 3);
          q_rev.segment<4>(3) = q_temp.inverse().coeffs();

          return q_rev;
        }
        Eigen::VectorXd operator()(const JointSpherical &) const
        {
          Eigen::Quaterniond q_temp(q[3], q[0], q[1], q[2]);

          return q_temp.inverse().coeffs();
        }
        Eigen::VectorXd operator()(const JointSphericalZYX &) const
        {
          // rotation matrix for spherique xyz for inverting spherical zyx
          Eigen::AngleAxisd Rx(-q[2], Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd Ry(-q[1], Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd Rz(-q[0], Eigen::Vector3d::UnitZ());
          // Eigen convention is right multiply
          Eigen::Matrix3d R = Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();
          // Convention it back into zyx
          Eigen::Vector3d q_reverse = R.eulerAngles(2, 1, 0);

          return q_reverse;
        }
        Eigen::VectorXd operator()(const JointTranslation &) const
        {
          return -q;
        }
        Eigen::VectorXd operator()(const JointPlanar &) const
        {
          Eigen::Vector3d trans;
          trans << q[0], q[3], 0;
          Eigen::Matrix3d R;
          R << q[2], q[3], 0, -q[3], q[2], 0, 0, 0, 1;
          Eigen::Vector3d trans_rev;
          trans_rev = -R * trans;
          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(4);
          q_rev << trans_rev[0], trans_rev[1], q[2], -q[3];
          return q_rev;
        }
        Eigen::VectorXd operator()(const JointHelical &) const
        {
          return -q;
        }
        Eigen::VectorXd operator()(const JointUniversal &) const
        {
          return q; // Because reverse is done in the joint, so q stays the same
        }

        Eigen::VectorXd operator()(const JointMimic & joint) const
        {
          return boost::apply_visitor(
            *this, joint.secondary_joint); // Don't know how to handle this yet
        }

        Eigen::VectorXd operator()(const JointComposite & joint) const
        {
          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(joint.nq);
          int index_back =
            joint.nq
            - boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints.back());
          int index_front = 0;
          for (int i = static_cast<int>(joint.joints.size() - 1); i >= 0; i--)
          {
            int nq_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nq; }, joint.joints[static_cast<size_t>(i)]);
            ReverseQVisitor reverse_temp(q.segment(index_back, nq_curr));
            q_rev.segment(index_front, nq_curr) =
              boost::apply_visitor(reverse_temp, joint.joints[static_cast<size_t>(i)]);
            index_front += nq_curr;
            if (i != 0)
              index_back -= boost::apply_visitor(
                [](const auto & j_) { return j_.nq; }, joint.joints[static_cast<size_t>(i - 1)]);
          }
          return q_rev;
        }
      };

      struct UpdateJointGraphPoseVisitor : public boost::static_visitor<SE3>
      {
        typedef JointDataTpl<double> JointData;
        const Eigen::VectorXd q_ref;

        UpdateJointGraphPoseVisitor(const Eigen::VectorXd q_)
        : q_ref(q_)
        {
        }

        SE3 joint_calc(JointModel jmodel) const
        {
          jmodel.setIndexes(1, 0, 0);
          JointData jdata = jmodel.createData();
          jmodel.calc(jdata, q_ref);

          return jdata.M();
        }

        SE3 operator()(const JointFixed & joint) const
        {
          return joint.joint_offset;
        }

        SE3 operator()(const JointRevolute & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Revolute nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelRevoluteUnaligned(joint.axis));
        }

        SE3 operator()(const JointPrismatic & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Prismatic nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelPrismaticUnaligned(joint.axis));
        }

        SE3 operator()(const JointRevoluteUnbounded & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - Joint Revolute Unbounded nq is 2. q_ref is the wrong size");

          return joint_calc(JointModelRevoluteUnboundedUnaligned(joint.axis));
        }

        SE3 operator()(const JointHelical & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Helical nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelHelicalUnaligned(joint.axis, joint.pitch));
        }

        SE3 operator()(const JointUniversal & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Universal nq is 2. q_ref is the wrong size");

          return joint_calc(JointModelUniversal(joint.axis1, joint.axis2));
        }

        SE3 operator()(const JointFreeFlyer & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint FreeFlyer nq is 7. q_ref is the wrong size");

          return joint_calc(JointModelFreeFlyer());
        }

        SE3 operator()(const JointSpherical & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Spherical nq is 4. q_ref is the wrong size");

          return joint_calc(JointModelSpherical());
        }

        SE3 operator()(const JointSphericalZYX & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint SphericalZYX nq is 3. q_ref is the wrong size");

          return joint_calc(JointModelSphericalZYX());
        }

        SE3 operator()(const JointPlanar & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Planar nq is 4. q_ref is the wrong size");

          return joint_calc(JointModelPlanar());
        }

        SE3 operator()(const JointTranslation & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Translation nq is 3. q_ref is the wrong size");

          return joint_calc(JointModelTranslation());
        }

        SE3 operator()(const JointMimic & /*joint*/) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - Joint Mimic cannot have a q_ref. Please use the joint offset directly.");
        }

        SE3 operator()(const JointComposite & joint) const
        {
          if (q_ref.size() != joint.nq)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - Joint Composite and its configuration vector have different sizes");

          JointComposite * joint_ptr = const_cast<JointComposite *>(&joint);

          int index = 0;

          for (size_t i = 0; i < joint.joints.size(); i++)
          {
            int nq_curr =
              boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints[i]);
            UpdateJointGraphPoseVisitor u_temp(q_ref.segment(index, nq_curr));
            SE3 pose_temp = boost::apply_visitor(u_temp, joint.joints[i]);
            joint_ptr->jointsPlacements[i] = joint_ptr->jointsPlacements[i] * pose_temp;
            index += nq_curr;
          }
          return SE3::Identity();
        }
      };

      struct RecordTreeEdgeVisitor : public boost::default_dfs_visitor
      {
        typedef ModelGraph::Graph Graph;
        typedef ModelGraph::EdgeDesc EdgeDesc;
        typedef std::unordered_map<std::string, bool> JointNameToDirection;

        RecordTreeEdgeVisitor(std::vector<EdgeDesc> * edges, JointNameToDirection * joint_forward)
        : edges(edges)
        , joint_forward(joint_forward)
        {
        }

        void tree_edge(EdgeDesc edge_desc, const Graph & g) const
        {
          const ModelGraphEdge & edge = g[edge_desc];
          (*joint_forward)[edge.name] = edge.forward;
          edges->push_back(edge_desc);
        }

        void forward_or_cross_edge(EdgeDesc, const Graph &) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Graph - there is a cycle in the graph. It is not yet "
                                   "supported, please change graph construction.");
        }

        std::vector<EdgeDesc> * edges;
        /// Joint name to a bool that hold true if the joint is in forward direction
        JointNameToDirection * joint_forward;
      };
    } // namespace internal
  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_graph_visitor_hpp__
