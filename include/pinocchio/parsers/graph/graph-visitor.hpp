//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_visitor_hpp__
#define __pinocchio_parsers_graph_visitor_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"

#include <boost/graph/visitors.hpp>

namespace pinocchio
{
  namespace graph
  {
    namespace internal
    {
      struct ReverseJointGraphVisitor
      : public boost::static_visitor<std::pair<JointGraphVariant, pinocchio::SE3>>
      {
        using ReturnType = std::pair<JointGraphVariant, pinocchio::SE3>;

        ReturnType operator()(const JointRevoluteGraph & joint) const
        {
          return {JointRevoluteGraph(joint.axis), pinocchio::SE3::Identity()};
        }

        ReturnType operator()(const JointRevoluteUnboundedGraph & joint) const
        {
          return {JointRevoluteUnboundedGraph(joint.axis), pinocchio::SE3::Identity()};
        }

        ReturnType operator()(const JointPrismaticGraph & joint) const
        {
          return {JointPrismaticGraph(joint.axis), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointFixedGraph & joint) const
        {
          return {JointFixedGraph(joint.joint_offset.inverse()), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointFreeFlyerGraph &) const
        {

          return {JointFreeFlyerGraph(), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointSphericalGraph &) const
        {
          return {JointSphericalGraph(), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointSphericalZYXGraph &) const
        {
          return {JointSphericalZYXGraph(), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointTranslationGraph &) const
        {
          return {JointTranslationGraph(), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointPlanarGraph &) const
        {
          return {JointPlanarGraph(), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointHelicalGraph & joint) const
        {
          return {JointHelicalGraph(joint.axis, joint.pitch), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointUniversalGraph & joint) const
        {
          return {JointUniversalGraph(-joint.axis2, -joint.axis1), pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointMimicGraph & joint) const
        {
          return {joint, pinocchio::SE3::Identity()};
        }
        ReturnType operator()(const JointCompositeGraph & joint) const
        {
          JointCompositeGraph jReturn;
          auto temp = boost::apply_visitor(*this, joint.joints.back());
          jReturn.addJoint(temp.first, temp.second * pinocchio::SE3::Identity());
          // Reverse joints
          for (int i = static_cast<int>(joint.joints.size() - 2); i >= 0; i--)
          {
            temp = boost::apply_visitor(*this, joint.joints[i]);
            jReturn.addJoint(temp.first, temp.second * joint.jointsPlacements[i + 1].inverse());
          }
          return {jReturn, joint.jointsPlacements[0].inverse()};
        }
      };

      // Add const to every operator (otherwise call fails)
      struct CreateJointModelVisitor : public boost::static_visitor<JointModel>
      {
        typedef JointModel ReturnType;

        ReturnType operator()(const JointFixedGraph & /*joint*/) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - cannot create a fixed joint. In pinocchio, fixed joints are frame.");
        }
        ReturnType operator()(const JointRevoluteGraph & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return pinocchio::JointModelRX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return pinocchio::JointModelRY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return pinocchio::JointModelRZ();
          }
          else
          {
            return pinocchio::JointModelRevoluteUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointRevoluteUnboundedGraph & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return pinocchio::JointModelRUBX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return pinocchio::JointModelRUBY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return pinocchio::JointModelRUBZ();
          }
          else
          {
            return pinocchio::JointModelRevoluteUnboundedUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointPrismaticGraph & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return pinocchio::JointModelPX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return pinocchio::JointModelPY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return pinocchio::JointModelPZ();
          }
          else
          {
            return pinocchio::JointModelPrismaticUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointHelicalGraph & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return pinocchio::JointModelHX(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return pinocchio::JointModelHY(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return pinocchio::JointModelHZ(joint.pitch);
          }
          else
          {
            return pinocchio::JointModelHelicalUnaligned(joint.axis, joint.pitch);
          }
        }
        ReturnType operator()(const JointFreeFlyerGraph & /*joint*/) const
        {
          return JointModelFreeFlyer();
        }
        ReturnType operator()(const JointTranslationGraph & /*joint*/) const
        {
          return JointModelTranslation();
        }
        ReturnType operator()(const JointPlanarGraph & /*joint*/) const
        {
          return JointModelPlanar();
        }
        ReturnType operator()(const JointSphericalGraph & /*joint*/) const
        {
          return JointModelSpherical();
        }
        ReturnType operator()(const JointSphericalZYXGraph & /*joint*/) const
        {
          return JointModelSphericalZYX();
        }
        ReturnType operator()(const JointUniversalGraph & joint) const
        {
          return JointModelUniversal(joint.axis1, joint.axis2);
        }

        ReturnType operator()(const JointMimicGraph & joint) const
        {
          return boost::apply_visitor(*this, joint.secondary_joint);
        }
        ReturnType operator()(const JointCompositeGraph & joint) const
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
            std::runtime_error,
            "Graph - Invalid joint between non body frames. Non body frames can "
            "only be added with Fixed joint");
        }

        template<typename JointGraph>
        void operator()(const JointGraph & joint, const BodyFrameGraph & b_f)
        {
          if (boost::get<BodyFrameGraph>(&source_vertex.frame) == nullptr) // body frame is index 0
                                                                           // in variant
            PINOCCHIO_THROW_PRETTY(
              std::runtime_error, "Graph -Invalid joint between a body and a non body frame.");

          const pinocchio::SE3 & joint_pose = edge.out_to_joint;
          const pinocchio::SE3 & body_pose = edge.joint_to_in;

          const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint, cjm(joint), previous_body.placement * joint_pose, edge.name);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);
        }

        template<typename FrameGraph>
        void operator()(const JointFixedGraph & /*joint*/, const FrameGraph & f_)
        {
          const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];

          model.addFrame(Frame(
            target_vertex.name, previous_body.parentJoint,
            previous_body.placement * edge.out_to_joint * edge.joint_to_in, f_.f_type));
        }

        void operator()(const JointMimicGraph & joint, const BodyFrameGraph & b_f)
        {
          if (edge.reverse)
            PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - JointMimic cannot be reversed.");

          if (boost::get<BodyFrameGraph>(&source_vertex.frame) == nullptr)
            PINOCCHIO_THROW_PRETTY(
              std::runtime_error, "Graph - Invalid joint between a body and a non body frame.");

          if (!model.existJointName(joint.primary_name))
            PINOCCHIO_THROW_PRETTY(
              std::runtime_error,
              "Graph - The parent joint of the mimic node is not in the kinematic tree");

          auto primary_joint = model.joints[model.getJointId(joint.primary_name)];

          const pinocchio::SE3 & joint_pose = edge.out_to_joint;
          const pinocchio::SE3 & body_pose = edge.joint_to_in;

          const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint,
            JointModelMimic(cjm(joint), primary_joint, joint.scaling, joint.offset),
            previous_body.placement * joint_pose, edge.name);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);
        }

        void operator()(const JointFixedGraph & /*joint*/, const BodyFrameGraph & b_f)
        {
          // Need to check what's vertex the edge is coming from. If it's a body, then we add
          // both the fixed joint frame and a body frame. Otherwise, it's a "fake" fixed joint
          // That's only used for graph construction, so we just add the body frame.
          if (boost::get<BodyFrameGraph>(&source_vertex.frame) == nullptr)
          {
            FrameIndex prev_f_id = model.getFrameId(source_vertex.name, OP_FRAME);
            if (prev_f_id == model.frames.size())
              prev_f_id = model.getFrameId(source_vertex.name, SENSOR);

            const Frame & previous_frame = model.frames[prev_f_id];
            model.addFrame(Frame(
              target_vertex.name, previous_frame.parentJoint,
              previous_frame.placement * edge.out_to_joint * edge.joint_to_in, BODY, b_f.inertia));
          }
          else
          {
            const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
            // Don't add a new joint in the model — create the fixed_joint frame
            FrameIndex f_id = model.addFrame(Frame(
              edge.name, previous_body.parentJoint, previous_body.placement * edge.out_to_joint,
              FIXED_JOINT, b_f.inertia));
            model.addBodyFrame(
              target_vertex.name, previous_body.parentJoint, edge.joint_to_in, (int)f_id);
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

        Eigen::VectorXd operator()(const JointRevoluteGraph &) const
        {
          return -q;
        }

        Eigen::VectorXd operator()(const JointRevoluteUnboundedGraph &) const
        {
          Eigen::Vector2d q_rev;
          q_rev << q[0], q[1];

          return q_rev;
        }

        Eigen::VectorXd operator()(const JointPrismaticGraph &) const
        {
          return -q;
        }

        Eigen::VectorXd operator()(const JointFixedGraph &) const
        {
          return q;
        }

        Eigen::VectorXd operator()(const JointFreeFlyerGraph &) const
        {

          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(7);
          Eigen::Quaterniond q_temp(q[6], q[3], q[4], q[5]);
          q_rev << -q[0], -q[1], -q[2], q_temp.inverse().x(), q_temp.inverse().y(),
            q_temp.inverse().z(), q_temp.inverse().w();

          return q_rev;
        }
        Eigen::VectorXd operator()(const JointSphericalGraph &) const
        {
          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(4);
          Eigen::Quaterniond q_temp(q[3], q[0], q[1], q[2]);
          q_rev << q_temp.inverse().x(), q_temp.inverse().y(), q_temp.inverse().z(),
            q_temp.inverse().w();

          return q_rev;
        }
        Eigen::VectorXd operator()(const JointSphericalZYXGraph &) const
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
        Eigen::VectorXd operator()(const JointTranslationGraph &) const
        {
          return -q;
        }
        Eigen::VectorXd operator()(const JointPlanarGraph &) const
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
        Eigen::VectorXd operator()(const JointHelicalGraph &) const
        {
          return -q;
        }
        Eigen::VectorXd operator()(const JointUniversalGraph &) const
        {
          return q; // Because reverse is done in the joint, so q stays the same
        }

        Eigen::VectorXd operator()(const JointMimicGraph & joint) const
        {
          return boost::apply_visitor(
            *this, joint.secondary_joint); // Don't know how to handle this yet
        }

        Eigen::VectorXd operator()(const JointCompositeGraph & joint) const
        {
          Eigen::VectorXd q_rev = Eigen::VectorXd::Zero(joint.nq);
          int index_back =
            static_cast<int>(joint.joints.size())
            - boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints.back());
          int index_front = 0;
          for (int i = static_cast<int>(joint.joints.size() - 1); i >= 0; i--)
          {
            int nq_curr =
              boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints[i]);
            ReverseQVisitor reverse_temp(q.segment(index_back, nq_curr));
            q_rev.segment(index_front, nq_curr) =
              boost::apply_visitor(reverse_temp, joint.joints[i]);
            index_front += nq_curr;
            if (i != 0)
              index_back -=
                boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints[i - 1]);
          }
          return q_rev;
        }
      };

      struct UpdateJointGraphPoseVisitor : public boost::static_visitor<pinocchio::SE3>
      {
        const Eigen::VectorXd q_ref;

        UpdateJointGraphPoseVisitor(const Eigen::VectorXd q_)
        : q_ref(q_)
        {
        }

        pinocchio::SE3 joint_calc(JointModel jmodel) const
        {
          jmodel.setIndexes(1, 0, 0);
          pinocchio::JointData jdata = jmodel.createData();
          jmodel.calc(jdata, q_ref);

          return jdata.M();
        }

        pinocchio::SE3 operator()(const JointFixedGraph & /*joint*/) const
        {
          return pinocchio::SE3::Identity();
        }

        pinocchio::SE3 operator()(const JointRevoluteGraph & joint) const
        {
          if (q_ref.size() != 1)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Revolute nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelRevoluteUnaligned(joint.axis));
        }

        pinocchio::SE3 operator()(const JointPrismaticGraph & joint) const
        {
          if (q_ref.size() != 1)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Prismatic nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelPrismaticUnaligned(joint.axis));
        }

        pinocchio::SE3 operator()(const JointRevoluteUnboundedGraph & joint) const
        {
          if (q_ref.size() != 2)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - Joint Revolute Unbounded nq is 2. q_ref is the wrong size");

          return joint_calc(JointModelRevoluteUnboundedUnaligned(joint.axis));
        }

        pinocchio::SE3 operator()(const JointHelicalGraph & joint) const
        {
          if (q_ref.size() != 1)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Helical nq is 1. q_ref is the wrong size");

          return joint_calc(JointModelHelicalUnaligned(joint.axis, joint.pitch));
        }

        pinocchio::SE3 operator()(const JointUniversalGraph & joint) const
        {
          if (q_ref.size() != 2)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Universal nq is 2. q_ref is the wrong size");

          return joint_calc(JointModelUniversal(joint.axis1, joint.axis2));
        }

        pinocchio::SE3 operator()(const JointFreeFlyerGraph & /*joint*/) const
        {
          if (q_ref.size() != 7)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint FreeFlyer nq is 7. q_ref is the wrong size");

          return joint_calc(JointModelFreeFlyer());
        }

        pinocchio::SE3 operator()(const JointSphericalGraph & /*joint*/) const
        {
          if (q_ref.size() != 4)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Spherical nq is 4. q_ref is the wrong size");

          return joint_calc(JointModelSpherical());
        }

        pinocchio::SE3 operator()(const JointSphericalZYXGraph & /*joint*/) const
        {
          if (q_ref.size() != 3)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint SphericalZYX nq is 3. q_ref is the wrong size");

          return joint_calc(JointModelSphericalZYX());
        }

        pinocchio::SE3 operator()(const JointPlanarGraph & /*joint*/) const
        {
          if (q_ref.size() != 4)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Planar nq is 4. q_ref is the wrong size");

          return joint_calc(JointModelPlanar());
        }

        pinocchio::SE3 operator()(const JointTranslationGraph & /*joint*/) const
        {
          if (q_ref.size() != 3)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Joint Translation nq is 3. q_ref is the wrong size");

          return joint_calc(JointModelTranslation());
        }

        pinocchio::SE3 operator()(const JointMimicGraph & /*joint*/) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - Joint Mimic cannot have a q_ref. Please use the joint offset argument.");
        }

        pinocchio::SE3 operator()(const JointCompositeGraph & joint) const
        {
          JointCompositeGraph * joint_ptr = const_cast<JointCompositeGraph *>(&joint);

          int index = 0;

          for (int i = 0; i < static_cast<int>(joint.joints.size()); i++)
          {
            int nq_curr =
              boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints[i]);
            UpdateJointGraphPoseVisitor u_temp(q_ref.segment(index, nq_curr));
            pinocchio::SE3 pose_temp = boost::apply_visitor(u_temp, joint.joints[i]);
            joint_ptr->jointsPlacements[i] = joint_ptr->jointsPlacements[i] * pose_temp;
            index += nq_curr;
          }
          return pinocchio::SE3::Identity();
        }
      };

      template<typename Graph>
      struct RecordTreeEdgeVisitor : public boost::default_dfs_visitor
      {
        typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

        RecordTreeEdgeVisitor(std::vector<EdgeDesc> * edges)
        : edges(edges)
        {
        }

        void tree_edge(EdgeDesc edge_desc, const Graph &) const
        {
          edges->push_back(edge_desc);
        }

        std::vector<EdgeDesc> * edges;
      };
    } // namespace internal
  } // namespace graph
} // namespace pinocchio

#endif // __pinocchio_graph_visitor_hpp__
