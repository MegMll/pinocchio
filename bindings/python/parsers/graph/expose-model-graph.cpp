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

    void exposeModelGraphAlgo()
    {
      using namespace pinocchio::graph;

      bp::class_<ModelGraph>("ModelGraph", "Represents multibody model as a bidirectional graph.", bp::init<>())
        .def("addFrame", &ModelGraph::addFrame,
             (bp::arg("self"), bp::arg("vertex_name"), bp::arg("frame")),
             "Add a new vertex to the graph.")
        .def("addBody", &ModelGraph::addBody,
             (bp::arg("self"), bp::arg("vertex_name"), bp::arg("inertia")),
             "Add a new body (vertex with inertia) to the graph.")
        .def("addJoint", &ModelGraph::addJoint,
             (bp::arg("self"), bp::arg("joint_name"), bp::arg("joint"),
              bp::arg("out_body"), bp::arg("out_to_joint"),
              bp::arg("in_body"), bp::arg("joint_to_in"),
              bp::arg("q_ref") = boost::none), // Use boost::python::api::object() for default none
             "Add edges (joint) to the graph. Since it's a bidirectional graph, "
             "edge and its reverse are added to the graph.")
        .def("buildModel", &ModelGraph::buildModel,
             (bp::arg("self"), bp::arg("root_body"), bp::arg("root_position"),
              bp::arg("root_joint"), // Use boost::python::api::object() for default none
              bp::arg("root_joint_name") = "root_joint"),
             "Build a pinocchio model based on the graph.")
        .def("copyGraph", &ModelGraph::copyGraph,
             (bp::arg("self"), bp::arg("g"), bp::arg("prefix") = ""),
             "Copies another ModelGraph into this one, with an optional prefix for names.");

      // Expose the global functions
      bp::def("mergeGraphs", &mergeGraphs,
              (bp::arg("g1"), bp::arg("g2"), bp::arg("g1_body"), bp::arg("g2_body"),
               bp::arg("pose_g2_body_in_g1"),
               bp::arg("merging_joint") = boost::none, // Use boost::python::api::object()
               bp::arg("merging_joint_name") = "merging_joint",
               bp::arg("g2_prefix") = "g2/"),
              "Merge two ModelGraphs together by adding an edge between specified bodies.");

      bp::def("fixJointsGraph", &fixJointsGraph,
              (bp::arg("g"), bp::arg("joints_to_lock"), bp::arg("reference_configurations")),
              "Fixes (locks) specified joints in a ModelGraph at given reference configurations.");
    }
  }
}
