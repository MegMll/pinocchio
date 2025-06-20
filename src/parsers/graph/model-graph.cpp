//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"
#include "pinocchio/parsers/graph/graph-visitor.hpp"
#include "pinocchio/parsers/graph/joints.hpp"

namespace pinocchio
{
  namespace graph
  {
    namespace
    {
      /// \return true if \p joint_name is already used in \p graph
      bool isJointNameExists(const ModelGraph::Graph & graph, const std::string & joint_name)
      {
        for (auto e_it = boost::edges(graph); e_it.first != e_it.second; ++e_it.first)
        {
          if (graph[*e_it.first].name == joint_name)
          {
            return true;
          }
        }
        return false;
      }
    } // namespace

    template<int Nq, int Nv>
    void JointLimits::setDimensions()
    {
      const double infty = std::numeric_limits<double>::infinity();

      maxEffort = Eigen::VectorXd::Constant(Nv, infty);
      maxVel = Eigen::VectorXd::Constant(Nv, infty);
      maxConfig = Eigen::VectorXd::Constant(Nq, infty);
      minConfig = Eigen::VectorXd::Constant(Nq, -infty);
      friction = Eigen::VectorXd::Constant(Nv, 0.);
      damping = Eigen::VectorXd::Constant(Nv, 0.);
      armature = Eigen::VectorXd::Constant(Nv, 0);
    }

    void JointLimits::append(const JointLimits & range, const int nq, const int nv)
    {
      assert(range.maxEffort.size() == nv);
      assert(range.minConfig.size() == nq);

      maxEffort.conservativeResize(maxEffort.size() + nv);
      maxEffort.tail(nv) = range.maxEffort;
      maxVel.conservativeResize(maxVel.size() + nv);
      maxVel.tail(nv) = range.maxVel;

      minConfig.conservativeResize(minConfig.size() + nq);
      minConfig.tail(nq) = range.minConfig;
      maxConfig.conservativeResize(maxConfig.size() + nq);
      maxConfig.tail(nq) = range.maxConfig;

      damping.conservativeResize(damping.size() + nv);
      damping.tail(nv) = range.damping;
      friction.conservativeResize(friction.size() + nv);
      friction.tail(nv) = range.friction;

      armature.conservativeResize(armature.size() + nv);
      armature.tail(nv) = range.armature;
    }

    EdgeBuilder & EdgeBuilder::withJointType(const JointGraphVariant & jtype)
    {
      param.joint = jtype;
      param.jlimit = boost::apply_visitor(internal::MakeJointLimitsDefaultVisitor(), jtype);

      return *this;
    }

    EdgeParameters::EdgeParameters(
      const std::string & jname,
      const std::string & source_name,
      const SE3 & out_to_joint,
      const std::string & target_name,
      const SE3 & joint_to_in,
      const JointGraphVariant & joint,
      const boost::optional<Eigen::VectorXd> q_ref)
    : name(jname)
    , source_vertex(source_name)
    , out_to_joint(out_to_joint)
    , target_vertex(target_name)
    , joint_to_in(joint_to_in)
    , joint(joint)
    , q_ref(q_ref)
    , jlimit(boost::apply_visitor(internal::MakeJointLimitsDefaultVisitor(), joint))
    {
    }

    void ModelGraph::addFrame(const std::string & vertex_name, const FrameGraphVariant & frame)
    {
      if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - vertex already in graph");

      auto vertex_desc = boost::add_vertex(graph);
      ModelGraphVertex & vertex = graph[vertex_desc];
      vertex.name = vertex_name;

      vertex.frame = frame;
      name_to_vertex.insert({vertex_name, vertex_desc});
    }

    void ModelGraph::addBody(const std::string & vertex_name, const Inertia & inert)
    {
      addFrame(vertex_name, BodyFrameGraph(inert));
    }

    void ModelGraph::addJoint(const EdgeParameters & params)
    {
      auto out_vertex = name_to_vertex.find(params.source_vertex);
      if (out_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - out_vertex does not exists");
      }
      auto in_vertex = name_to_vertex.find(params.target_vertex);
      if (in_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - in_vertex does not exists");
      }
      if (isJointNameExists(graph, params.name))
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - joint_name already exists");
      }
      if (boost::edge(out_vertex->second, in_vertex->second, graph).second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Joint already connect in_body to out_body");
      }
      if (boost::get<BodyFrameGraph>(&graph[out_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - sensor and op_frame can only be appended to bodies");

      auto edge_desc = boost::add_edge(out_vertex->second, in_vertex->second, graph);
      if (!edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Edge cannot be added between these two vertices");
      }

      ModelGraphEdge & edge = graph[edge_desc.first];
      edge.name = params.name;
      edge.joint = params.joint;
      if (params.q_ref)
        edge.out_to_joint =
          params.out_to_joint
          * boost::apply_visitor(internal::UpdateJointGraphPoseVisitor(*params.q_ref), edge.joint);
      else
        edge.out_to_joint = params.out_to_joint;

      edge.joint_to_in = params.joint_to_in;

      edge.jlimit = params.jlimit;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, graph);
      if (!reverse_edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Reverse edge cannot be added between these two vertices");
      }

      ModelGraphEdge & reverse_edge = graph[reverse_edge_desc.first];
      reverse_edge.name = params.name;
      auto reversed_joint =
        boost::apply_visitor(internal::ReverseJointGraphVisitor(), params.joint);
      reverse_edge.joint = reversed_joint.first;
      if (params.q_ref)
      {
        const Eigen::VectorXd q_ref_reverse =
          boost::apply_visitor(internal::ReverseQVisitor(*params.q_ref), params.joint);
        reverse_edge.out_to_joint =
          params.joint_to_in.inverse()
          * boost::apply_visitor(
            internal::UpdateJointGraphPoseVisitor(q_ref_reverse), reverse_edge.joint);
      }
      else
        reverse_edge.out_to_joint = params.joint_to_in.inverse();

      reverse_edge.joint_to_in = reversed_joint.second * params.out_to_joint.inverse();
      reverse_edge.forward = false;

      reverse_edge.jlimit =
        boost::apply_visitor(internal::ReverseJointLimitsVisitor(params.jlimit), params.joint);
    }

    void ModelGraph::addJoint(
      const std::string & joint_name,
      const JointGraphVariant & joint,
      const std::string & out_body,
      const SE3 & out_to_joint,
      const std::string & in_body,
      const SE3 & joint_to_in,
      const boost::optional<Eigen::VectorXd> & q_ref)
    {
      return addJoint(
        EdgeParameters(joint_name, out_body, out_to_joint, in_body, joint_to_in, joint, q_ref));
    }

    EdgeBuilder ModelGraph::useEdgeBuilder()
    {
      return EdgeBuilder(*this);
    }

    void ModelGraph::appendGraph(const ModelGraph & g)
    {
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        this->addFrame(name, vertex_data.frame);
      }

      // Copy all forward joints from g. Since addJoint will create the reverse edge, no need to add
      // both.
      for (auto e_it = boost::edges(g.graph); e_it.first != e_it.second; ++e_it.first)
      {
        const auto & edge = *e_it.first;
        const auto & edge_data = g.graph[edge];
        if (edge_data.forward)
        {
          auto src = boost::source(edge, g.graph);
          auto tgt = boost::target(edge, g.graph);

          const auto & src_name = g.graph[src].name;
          const auto & tgt_name = g.graph[tgt].name;

          this->useEdgeBuilder()
            .withName(edge_data.name)
            .withSourceVertex(src_name)
            .withSourcePose(edge_data.out_to_joint)
            .withTargetVertex(tgt_name)
            .withTargetPose(edge_data.joint_to_in)
            .withJointType(edge_data.joint)
            .build();
        }
      }
    }
  } // namespace graph
} // namespace pinocchio
