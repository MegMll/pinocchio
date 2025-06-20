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

      bp::class_<EdgeParameters>(
        "EdgeParameters", "Parameters for defining an edge (joint) in the ModelGraph.")
        // Default constructor
        .def(bp::init<>())
        // Parameterized constructor
        .def(bp::init<
             const std::string &, const std::string &, const SE3 &, const std::string &,
             const SE3 &, const JointGraphVariant &, const boost::optional<Eigen::VectorXd>>(
          (bp::arg("name"), bp::arg("source_vertex"), bp::arg("out_to_joint"),
           bp::arg("target_vertex"), bp::arg("joint_to_in"), bp::arg("joint"),
           bp::arg("q_ref") = boost::none),
          "Constructor to define an edge with specific parameters."))
        .def_readwrite("name", &EdgeParameters::name, "Name of the edge/joint.")
        .def_readwrite(
          "source_vertex", &EdgeParameters::source_vertex,
          "Name of the source vertex (parent body).")
        .def_readwrite(
          "target_vertex", &EdgeParameters::target_vertex,
          "Name of the target vertex (child body).")
        .def_readwrite(
          "out_to_joint", &EdgeParameters::out_to_joint,
          "Transformation from source_vertex to the joint origin.")
        .def_readwrite(
          "joint_to_in", &EdgeParameters::joint_to_in,
          "Transformation from joint origin to the target_vertex.")
        .def_readwrite(
          "q_ref", &EdgeParameters::q_ref, "Optional reference configuration for the joint.")
        .def_readwrite(
          "joint", &EdgeParameters::joint, "Type of the joint (e.g., fixed, revolute, prismatic).");

      bp::class_<EdgeBuilder>(
        "EdgeBuilder",
        "A builder class for conveniently constructing and adding edges (joints) to a ModelGraph.",
        bp::init<ModelGraph &>(
          (bp::arg("graph")), "Constructs an EdgeBuilder associated with a ModelGraph instance."))
        .def(
          "withName", &EdgeBuilder::withName, bp::return_self<>(), bp::arg("name"),
          "Sets the name of the edge/joint")
        .def(
          "withTargetVertex", &EdgeBuilder::withTargetVertex, bp::return_self<>(),
          bp::arg("target_name"), "Sets the target vertex name")
        .def(
          "withSourceVertex", &EdgeBuilder::withSourceVertex, bp::return_self<>(),
          bp::arg("source_name"), "Sets the source vertex name.")
        .def(
          "withTargetPose", &EdgeBuilder::withTargetPose, bp::return_self<>(),
          bp::arg("target_pose"), "Sets the transformation from joint origin to target vertex.")
        .def(
          "withSourcePose", &EdgeBuilder::withSourcePose, bp::return_self<>(),
          bp::arg("source_pose"), "Sets the transformation from source vertex to joint origin")
        .def(
          "withJointType", &EdgeBuilder::withJointType, bp::return_self<>(), bp::arg("jtype"),
          "Sets the type of the joint.")
        .def(
          "withQref", &EdgeBuilder::withQref, bp::return_self<>(), bp::arg("qref"),
          "Sets the optional reference configuration for the joint")
        .def(
          "build", &EdgeBuilder::build,
          "Builds the edge/joint parameters and adds the joint to the associated ModelGraph.")
        .def_readwrite(
          "param", &EdgeBuilder::param, "Direct access to the EdgeParameters object being built.");

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
          "addJoint",
          (void(ModelGraph::*)(
            const std::string &, const JointGraphVariant &, const std::string &, const SE3 &,
            const std::string &, const SE3 &, const boost::optional<Eigen::VectorXd> &))
            & ModelGraph::addJoint,
          (bp::arg("self"), bp::arg("joint_name"), bp::arg("joint"), bp::arg("out_body"),
           bp::arg("out_to_joint"), bp::arg("in_body"), bp::arg("joint_to_in"),
           bp::arg("q_ref") = boost::none),
          "Add edges (joint) to the graph. Since it's a bidirectional graph,\n"
          "edge and its reverse are added to the graph.\n")
        .def(
          "addJoint", (void(ModelGraph::*)(const EdgeParameters &)) & ModelGraph::addJoint,
          (bp::arg("self"), bp::arg("params")),
          "Add edges (joint) to the graph using EdgeParameters.")
        .def(
          "useEdgeBuilder", &ModelGraph::useEdgeBuilder, bp::arg("self"),
          "Returns an EdgeBuilder object to construct edges.")
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
