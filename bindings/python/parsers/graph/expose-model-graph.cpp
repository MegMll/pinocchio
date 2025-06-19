//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/bindings/python/parsers/graph/model-configuration-converter.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"

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
      return pinocchio::graph::buildModel(
        graph, root_body, root_position, root_joint, root_joint_name);
    }

    static pinocchio::Model build_model_wrapper(
      const pinocchio::graph::ModelGraph & graph,
      const std::string & root_body,
      const pinocchio::SE3 & root_position)
    {
      return pinocchio::graph::buildModel(graph, root_body, root_position);
    }

    static bp::tuple build_model_with_build_info_with_root_wrapper(
      const pinocchio::graph::ModelGraph & graph,
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const pinocchio::graph::JointGraphVariant & root_joint,
      const std::string & root_joint_name)
    {
      auto ret = pinocchio::graph::buildModelWithBuildInfo(
        graph, root_body, root_position, root_joint, root_joint_name);
      return bp::make_tuple(ret.model, ret.build_info);
    }

    static bp::tuple build_model_with_build_info_wrapper(
      const pinocchio::graph::ModelGraph & graph,
      const std::string & root_body,
      const pinocchio::SE3 & root_position)
    {
      auto ret = pinocchio::graph::buildModelWithBuildInfo(graph, root_body, root_position);
      return bp::make_tuple(ret.model, ret.build_info);
    }

    static pinocchio::graph::ModelGraph merge_with_joint_wrapper(
      const pinocchio::graph::ModelGraph & g1,
      const pinocchio::graph::ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const pinocchio::graph::JointGraphVariant & merging_joint,
      const std::string & merging_joint_name)
    {
      return pinocchio::graph::merge(
        g1, g2, g1_body, g2_body, pose_g2_body_in_g1, merging_joint, merging_joint_name);
    }

    static pinocchio::graph::ModelGraph merge_wrapper(
      const pinocchio::graph::ModelGraph & g1,
      const pinocchio::graph::ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1)
    {
      return pinocchio::graph::merge(g1, g2, g1_body, g2_body, pose_g2_body_in_g1);
    }

    void exposeModelGraphAlgo()
    {
      using namespace pinocchio::graph;

      bp::class_<ModelGraphBuildInfo> model_grap_build_info(
        "ModelGraphBuildInfo",
        "Contains information about how buildModel walked the ModelGraph to construct a Model");
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
           bp::arg("q_ref") = boost::none),
          "Add edges (joint) to the graph. Since it's a bidirectional graph, "
          "edge and its reverse are added to the graph.")
        .def(
          "appendGraph", &ModelGraph::appendGraph, (bp::arg("self"), bp::arg("g")),
          "Copies another ModelGraph into this one. Use with caution, because no edges are created "
          "to connect the new graph to itself.");

      // Expose the global functions
      bp::def(
        "buildModel", &build_model_wrapper,
        (bp::arg("g"), bp::arg("root_body"), bp::arg("root_position")),
        "Build a pinocchio model based on the graph.");

      bp::def(
        "buildModel", &build_model_with_root_wrapper,
        (bp::arg("self"), bp::arg("root_body"), bp::arg("root_position"), bp::arg("root_joint"),
         bp::arg("root_joint_name") = "root_joint"),
        "Build a pinocchio model based on the graph.");

      bp::def(
        "buildModelWithBuildInfo", &build_model_with_build_info_wrapper,
        (bp::arg("g"), bp::arg("root_body"), bp::arg("root_position")),
        "Build a pinocchio model based on the graph.");

      bp::def(
        "buildModelWithBuildInfo", &build_model_with_build_info_with_root_wrapper,
        (bp::arg("self"), bp::arg("root_body"), bp::arg("root_position"), bp::arg("root_joint"),
         bp::arg("root_joint_name") = "root_joint"),
        "Build a pinocchio model based on the graph.");

      bp::def(
        "merge", &merge_with_joint_wrapper,
        (bp::arg("g1"), bp::arg("g2"), bp::arg("g1_body"), bp::arg("g2_body"),
         bp::arg("pose_g2_body_in_g1"), bp::arg("merging_joint"),
         bp::arg("merging_joint_name") = "merging_joint"),
        "Merge two ModelGraphs together by adding an edge between specified bodies.");

      bp::def(
        "merge", &merge_wrapper,
        (bp::arg("g1"), bp::arg("g2"), bp::arg("g1_body"), bp::arg("g2_body"),
         bp::arg("pose_g2_body_in_g1")),
        "Merge two ModelGraphs together by adding an edge between specified bodies.");

      typedef std::vector<context::VectorXs> StdVec_VectorXs;
      StdVectorPythonVisitor<StdVec_VectorXs, false>::expose(
        "StdVec_VectorXs",
        eigenpy::details::overload_base_get_item_for_std_vector<StdVec_VectorXs>());
      bp::def(
        "lockJoints", &lockJoints,
        (bp::arg("g"), bp::arg("joints_to_lock"), bp::arg("reference_configurations")),
        "Lock specified joints in a ModelGraph at given reference configurations.");

      bp::def(
        "prefixNames", &prefixNames, (bp::arg("g"), bp::arg("prefix")),
        "Add a prefix to all names (body and joints) in the graph g. Useful to use before merging "
        "two model graphs.");

      graph::python::ModelConfigurationConverterVisitor<
        context::Scalar, context::Options, JointCollectionDefaultTpl>::expose();
    }
  } // namespace python
} // namespace pinocchio
