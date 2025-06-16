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

    void ModelGraph::addJoint(
      const std::string & joint_name,
      const JointGraphVariant & joint,
      const std::string & out_body,
      const SE3 & out_to_joint,
      const std::string & in_body,
      const SE3 & joint_to_in,
      const boost::optional<Eigen::VectorXd> & q_ref)
    {
      auto out_vertex = name_to_vertex.find(out_body);
      if (out_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - out_vertex does not exists");
      }
      auto in_vertex = name_to_vertex.find(in_body);
      if (in_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - in_vertex does not exists");
      }
      if (isJointNameExists(graph, joint_name))
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
          std::invalid_argument, "Graph - Edge cannot be added between these two vertexes");
      }

      ModelGraphEdge & edge = graph[edge_desc.first];
      edge.name = joint_name;
      edge.joint = joint;
      if (q_ref)
        edge.out_to_joint =
          out_to_joint
          * boost::apply_visitor(internal::UpdateJointGraphPoseVisitor(*q_ref), edge.joint);
      else
        edge.out_to_joint = out_to_joint;

      edge.joint_to_in = joint_to_in;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, graph);
      if (!reverse_edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Reverse edge cannot be added between these two vertexes");
      }

      ModelGraphEdge & reverse_edge = graph[reverse_edge_desc.first];
      reverse_edge.name = joint_name;
      auto reversed_joint = boost::apply_visitor(internal::ReverseJointGraphVisitor(), joint);
      reverse_edge.joint = reversed_joint.first;
      if (q_ref)
      {
        const Eigen::VectorXd q_ref_reverse =
          boost::apply_visitor(internal::ReverseQVisitor(*q_ref), joint);
        reverse_edge.out_to_joint =
          joint_to_in.inverse()
          * boost::apply_visitor(
            internal::UpdateJointGraphPoseVisitor(q_ref_reverse), reverse_edge.joint);
      }
      else
        reverse_edge.out_to_joint = joint_to_in.inverse();

      reverse_edge.joint_to_in = reversed_joint.second * out_to_joint.inverse();
      reverse_edge.forward = false;
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

          this->addJoint(
            edge_data.name, edge_data.joint, src_name, edge_data.out_to_joint, tgt_name,
            edge_data.joint_to_in);
        }
      }
    }
  } // namespace graph
} // namespace pinocchio
