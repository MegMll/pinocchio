//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/graph/model-graph.hpp"
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

    void ModelGraph::addFrame(const std::string & vertex_name, const FrameGraphVariant & frame)
    {
      if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - vertex already in graph");

      auto vertex_desc = boost::add_vertex(g);
      ModelGraphVertex & vertex = g[vertex_desc];
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
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - out_vertex does not exist");
      }
      auto in_vertex = name_to_vertex.find(in_body);
      if (in_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - in_vertex does not exist");
      }
      if (isJointNameExists(g, joint_name))
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - joint_name already exists");
      }
      if (boost::edge(out_vertex->second, in_vertex->second, g).second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Joint already connect in_body to out_body");
      }
      if (boost::get<BodyFrameGraph>(&g[out_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - sensor and op_frame can only be appended to bodies");

      auto edge_desc = boost::add_edge(out_vertex->second, in_vertex->second, g);
      if (!edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - Edge cannot be added between these two vertexes");
      }

      ModelGraphEdge & edge = g[edge_desc.first];
      edge.name = joint_name;
      edge.joint = joint;
      if (q_ref)
        edge.out_to_joint =
          out_to_joint
          * boost::apply_visitor(internal::UpdateJointGraphPoseVisitor(*q_ref), edge.joint);
      else
        edge.out_to_joint = out_to_joint;

      edge.joint_to_in = joint_to_in;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, g);
      if (!reverse_edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - Reverse edge cannot be added between these two vertexes");
      }

      ModelGraphEdge & reverse_edge = g[reverse_edge_desc.first];
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
      reverse_edge.reverse = true;
    }

    Model ModelGraph::buildModel(
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const boost::optional<JointGraphVariant> & root_joint,
      const std::string & root_joint_name) const
    {
      auto root_vertex = name_to_vertex.find(root_body);
      if (root_vertex == name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - root_body does not exist in the graph");

      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g));
      internal::RecordTreeEdgeVisitor<Graph> tree_edge_visitor(&edges);
      boost::depth_first_search(g, tree_edge_visitor, colors.data(), root_vertex->second);

      Model model;
      const ModelGraphVertex & root_vertex_data = g[root_vertex->second];

      if (root_joint) // Root joint provided
      {
        JointIndex j_id = model.addJoint(
          0, boost::apply_visitor(internal::CreateJointModelVisitor(), *root_joint), root_position,
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
      for (const EdgeDesc & edge_desc : edges)
      {
        const VertexDesc & source_vertex_desc = boost::source(edge_desc, g);
        const VertexDesc & target_vertex_desc = boost::target(edge_desc, g);
        const ModelGraphEdge & edge = g[edge_desc];
        const ModelGraphVertex & source_vertex = g[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g[target_vertex_desc];

        internal::AddJointModelVisitor visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint, target_vertex.frame);
      }
      return model;
    }

    Model ModelGraph::buildModel(
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const JointGraphVariant & root_joint,
      const std::string & root_joint_name) const
    {
      return buildModel(
        root_body, root_position, boost::make_optional(root_joint), root_joint_name);
    }

    void ModelGraph::copyGraph(const ModelGraph & g, const std::string & prefix)
    {
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.g[old_v];

        this->addFrame(prefix + name, vertex_data.frame);
      }

      // Copy all edges from g
      for (auto e_it = boost::edges(g.g); e_it.first != e_it.second; ++e_it.first)
      {
        const auto & edge = *e_it.first;
        auto src = boost::source(edge, g.g);
        auto tgt = boost::target(edge, g.g);
        const auto & edge_data = g.g[edge];

        const auto & src_name = g.g[src].name;
        const auto & tgt_name = g.g[tgt].name;

        this->addJoint(
          prefix + edge_data.name, edge_data.joint, prefix + src_name, edge_data.out_to_joint,
          prefix + tgt_name, edge_data.joint_to_in);
      }
    }

    ModelGraph mergeGraphs(
      const ModelGraph & g1,
      const ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const boost::optional<JointGraphVariant> & merging_joint,
      const std::string & merging_joint_name,
      const std::string & g2_prefix)
    {
      JointGraphVariant joint(JointFixedGraph{});
      if (merging_joint)
      {
        joint = *merging_joint;
      }
      return mergeGraphs(
        g1, g2, g1_body, g2_body, pose_g2_body_in_g1, joint, merging_joint_name, g2_prefix);
    }

    ModelGraph mergeGraphs(
      const ModelGraph & g1,
      const ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const JointGraphVariant & merging_joint,
      const std::string & merging_joint_name,
      const std::string & g2_prefix)
    {
      // Check bodies exists in graphs
      if (g1.name_to_vertex.find(g1_body) == g1.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "mergeGraph - g1_body not found");

      auto g1_vertex = g1.name_to_vertex.find(g1_body);
      if (boost::get<BodyFrameGraph>(&g1.g[g1_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "mergeGraph - Merging graphes needs to be done between two bodies. "
                              "Vertex in g1 is not a body");

      if (g2.name_to_vertex.find(g2_body) == g2.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "mergeGraph - g2_body not found");

      auto g2_vertex = g2.name_to_vertex.find(g2_body);
      if (boost::get<BodyFrameGraph>(&g2.g[g2_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "mergeGraph - Merging graphes needs to be done between two bodies. "
                              "Vertex in g2 is not a body");

      ModelGraph g_merged;

      g_merged.copyGraph(g1);
      g_merged.copyGraph(g2, g2_prefix);

      const std::string g2_body_merged = g2_prefix + g2_body;

      g_merged.addJoint(
        merging_joint_name, merging_joint, g1_body, SE3::Identity(), g2_body_merged,
        pose_g2_body_in_g1);

      return g_merged;
    }

    ModelGraph fixJointsGraph(
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
        const auto & vertex_data = g.g[old_v];

        g_locked.addFrame(name, vertex_data.frame);
      }

      // Copy all edges from g
      for (auto e_it = boost::edges(g.g); e_it.first != e_it.second; ++e_it.first)
      {

        const auto & edge = *e_it.first;
        auto src = boost::source(edge, g.g);
        auto tgt = boost::target(edge, g.g);

        const auto & edge_data = g.g[edge];

        const auto & src_name = g.g[src].name;
        const auto & tgt_name = g.g[tgt].name;

        auto it = std::find(joints_to_lock.begin(), joints_to_lock.end(), edge_data.name);
        if (it != joints_to_lock.end())
        {
          int index = std::distance(joints_to_lock.begin(), it);
          const Eigen::VectorXd & q_ref = reference_configurations[index];

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
      return g_locked;
    }
  } // namespace graph
} // namespace pinocchio
