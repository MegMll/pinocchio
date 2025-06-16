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
    struct AddRootFrameVisitor : public boost::static_visitor<>
    {
      const ModelGraphVertex vertex;
      const pinocchio::SE3 position;
      const JointIndex j;
      Model & model;

      AddRootFrameVisitor(
        const ModelGraphVertex & v, const JointIndex & j_id, const pinocchio::SE3 & pose, Model & m)
      : vertex(v)
      , position(pose)
      , j(j_id)
      , model(m)
      {
      }

      void operator()(const BodyFrameGraph & b_f) const
      {
        const FrameIndex f_id = model.getFrameId(model.names[j], JOINT);
        model.addFrame(Frame(vertex.name, j, f_id, position, BODY, b_f.inertia));
      }

      template<typename FrameGraph>
      void operator()(const FrameGraph & f_) const
      {
        const FrameIndex f_id = model.getFrameId(model.names[j], JOINT);
        model.addFrame(Frame(vertex.name, j, f_id, position, f_.f_type));
      }
    };

    Model buildModel(
      const ModelGraph & g,
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const JointGraphVariant & root_joint,
      const std::string & root_joint_name)
    {
      typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
        Graph;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
      typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

      auto root_vertex = g.name_to_vertex.find(root_body);
      if (root_vertex == g.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - root_body does not exist in the graph");

      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g.graph), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g.graph));
      internal::RecordTreeEdgeVisitor<Graph> tree_edge_visitor(&edges);
      boost::depth_first_search(g.graph, tree_edge_visitor, colors.data(), root_vertex->second);

      Model model;
      const ModelGraphVertex & root_vertex_data = g.graph[root_vertex->second];

      if (!boost::get<JointFixedGraph>(&root_joint)) // Root joint provided
      {
        JointIndex j_id = model.addJoint(
          0, boost::apply_visitor(internal::CreateJointModelVisitor(), root_joint), root_position,
          root_joint_name);
        model.addJointFrame(j_id);

        AddRootFrameVisitor afv(root_vertex_data, j_id, pinocchio::SE3::Identity(), model);
        boost::apply_visitor(afv, root_vertex_data.frame);
      }
      else // Fixed to world
      {
        AddRootFrameVisitor afv(root_vertex_data, (JointIndex)0, root_position, model);
        boost::apply_visitor(afv, root_vertex_data.frame);
      }

      // Go through rest of the graph
      for (const auto & edge_desc : edges)
      {
        const auto & source_vertex_desc = boost::source(edge_desc, g.graph);
        const auto & target_vertex_desc = boost::target(edge_desc, g.graph);
        const ModelGraphEdge & edge = g.graph[edge_desc];
        const ModelGraphVertex & source_vertex = g.graph[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g.graph[target_vertex_desc];

        internal::AddJointModelVisitor visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint, target_vertex.frame);
      }
      return model;
    }

    ModelGraph prefixingNameGraph(const ModelGraph & g, const std::string prefix)
    {
      ModelGraph g_return;
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        g_return.addFrame(prefix + name, vertex_data.frame);
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

          g_return.addJoint(
            edge_data.name, edge_data.joint, prefix + src_name, edge_data.out_to_joint,
            prefix + tgt_name, edge_data.joint_to_in);
        }
      }
      return g_return;
    }

    ModelGraph mergeGraphs(
      const ModelGraph & g1,
      const ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const JointGraphVariant & merging_joint,
      const std::string & merging_joint_name)
    {
      // Check bodies exists in graphs
      if (g1.name_to_vertex.find(g1_body) == g1.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "mergeGraph - g1_body not found");

      auto g1_vertex = g1.name_to_vertex.find(g1_body);
      if (boost::get<BodyFrameGraph>(&g1.graph[g1_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "mergeGraph - Merging graphes needs to be done between two bodies. "
          "Vertex in g1 is not a body");

      if (g2.name_to_vertex.find(g2_body) == g2.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "mergeGraph - g2_body not found");

      auto g2_vertex = g2.name_to_vertex.find(g2_body);
      if (boost::get<BodyFrameGraph>(&g2.graph[g2_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "mergeGraph - Merging graphes needs to be done between two bodies. "
          "Vertex in g2 is not a body");

      ModelGraph g_merged;

      g_merged.appendGraph(g1);
      g_merged.appendGraph(g2);

      const std::string g2_body_merged = g2_body;

      g_merged.addJoint(
        merging_joint_name, merging_joint, g1_body, SE3::Identity(), g2_body_merged,
        pose_g2_body_in_g1);

      return g_merged;
    }

    ModelGraph lockJoints(
      const ModelGraph & g,
      const std::vector<std::string> & joints_to_lock,
      const std::vector<Eigen::VectorXd> & reference_configurations)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        joints_to_lock.size() == reference_configurations.size(),
        "Graph - mismatch size between joints_to_lock list and reference configurations");

      ModelGraph g_locked;
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        g_locked.addFrame(name, vertex_data.frame);
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

          auto it = std::find(joints_to_lock.begin(), joints_to_lock.end(), edge_data.name);
          if (it != joints_to_lock.end())
          {
            auto index = std::distance(joints_to_lock.begin(), it);
            const Eigen::VectorXd & q_ref =
              reference_configurations[static_cast<std::size_t>(index)];

            internal::UpdateJointGraphPoseVisitor ujgpv(q_ref);
            pinocchio::SE3 pose_offset = boost::apply_visitor(ujgpv, edge_data.joint);

            g_locked.addJoint(
              edge_data.name, JointFixedGraph(pose_offset), src_name, edge_data.out_to_joint,
              tgt_name, edge_data.joint_to_in, q_ref);
          }
          else
          {
            g_locked.addJoint(
              edge_data.name, edge_data.joint, src_name, edge_data.out_to_joint, tgt_name,
              edge_data.joint_to_in);
          }
        }
      }
      return g_locked;
    }
  } // namespace graph
} // namespace pinocchio
