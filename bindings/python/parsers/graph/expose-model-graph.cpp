//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    static pinocchio::Model build_model_with_root_wrapper(
      const pinocchio::graph::ModelGraph & graph,
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const pinocchio::graph::JointGraphVariant & root_joint,
      const std::string & root_joint_name)
    {
      return graph.buildModel(root_body, root_position, root_joint, root_joint_name);
    }

    static pinocchio::Model build_model_wrapper(
      const pinocchio::graph::ModelGraph & graph,
      const std::string & root_body,
      const pinocchio::SE3 & root_position)
    {
      return graph.buildModel(root_body, root_position);
    }

    static pinocchio::graph::ModelGraph merge_graph_with_joint_wrapper(
      const pinocchio::graph::ModelGraph & g1,
      const pinocchio::graph::ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const pinocchio::graph::JointGraphVariant & merging_joint,
      const std::string & merging_joint_name,
      const std::string & g2_prefix)
    {
      return pinocchio::graph::mergeGraphs(
        g1, g2, g1_body, g2_body, pose_g2_body_in_g1, merging_joint, merging_joint_name, g2_prefix);
    }

    static pinocchio::graph::ModelGraph merge_graph_wrapper(
      const pinocchio::graph::ModelGraph & g1,
      const pinocchio::graph::ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      boost::none_t,
      const std::string & merging_joint_name,
      const std::string & g2_prefix)
    {
      return pinocchio::graph::mergeGraphs(
        g1, g2, g1_body, g2_body, pose_g2_body_in_g1, boost::none, merging_joint_name, g2_prefix);
    }

    void exposeModelGraphAlgo()
    {
      using namespace pinocchio::graph;

      bp::class_<ModelGraph>(
        "ModelGraph", "Represents multibody model as a bidirectional graph.", bp::init<>())
        .def(
          "addFrame", &ModelGraph::addFrame,
          (bp::arg("self"), bp::arg("vertex_name"), bp::arg("frame")),
          "Add a new vertex to the graph.")
        .def(
          "addBody", &ModelGraph::addBody,
          (bp::arg("self"), bp::arg("vertex_name"), bp::arg("inertia")),
          "Add a new body (vertex with inertia) to the graph.")
        .def(
          "addJoint", &ModelGraph::addJoint,
          (bp::arg("self"), bp::arg("joint_name"), bp::arg("joint"), bp::arg("out_body"),
           bp::arg("out_to_joint"), bp::arg("in_body"), bp::arg("joint_to_in"),
           bp::arg("q_ref") = boost::none), // Use boost::python::api::object() for default none
          "Add edges (joint) to the graph. Since it's a bidirectional graph, "
          "edge and its reverse are added to the graph.")
        .def(
          "buildModel", &build_model_wrapper,
          (bp::arg("self"), bp::arg("root_body"), bp::arg("root_position")),
          "Build a pinocchio model based on the graph.")
        .def(
          "buildModel", &build_model_with_root_wrapper,
          (bp::arg("self"), bp::arg("root_body"), bp::arg("root_position"), bp::arg("root_joint"),
           bp::arg("root_joint_name") = "root_joint"),
          "Build a pinocchio model based on the graph.")
        .def(
          "copyGraph", &ModelGraph::copyGraph,
          (bp::arg("self"), bp::arg("g"), bp::arg("prefix") = ""),
          "Copies another ModelGraph into this one, with an optional prefix for names.");

      // Expose the global functions
      bp::def(
        "mergeGraphs", &merge_graph_with_joint_wrapper,
        (bp::arg("g1"), bp::arg("g2"), bp::arg("g1_body"), bp::arg("g2_body"),
         bp::arg("pose_g2_body_in_g1"), bp::arg("merging_joint"),
         bp::arg("merging_joint_name") = "merging_joint", bp::arg("g2_prefix") = "g2/"),
        "Merge two ModelGraphs together by adding an edge between specified bodies.");

      bp::def(
        "mergeGraphs", &merge_graph_wrapper,
        (bp::arg("g1"), bp::arg("g2"), bp::arg("g1_body"), bp::arg("g2_body"),
         bp::arg("pose_g2_body_in_g1"), bp::arg("merging_joint"),
         bp::arg("merging_joint_name") = "merging_joint", bp::arg("g2_prefix") = "g2/"),
        "Merge two ModelGraphs together by adding an edge between specified bodies.");

      bp::def(
        "fixJointsGraph", &fixJointsGraph,
        (bp::arg("g"), bp::arg("joints_to_lock"), bp::arg("reference_configurations")),
        "Fixes (locks) specified joints in a ModelGraph at given reference configurations.");
    }
  } // namespace python
} // namespace pinocchio
