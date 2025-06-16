//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename GraphVariant>
    struct VariantToPythonVisitor : boost::static_visitor<PyObject *>
    {
      static result_type convert(GraphVariant const & v)
      {
        return apply_visitor(VariantToPythonVisitor(), v);
      }

      template<typename T>
      result_type operator()(T const & t) const
      {
        return boost::python::incref(boost::python::object(t).ptr());
      }
    };

    void exposeFramesGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<BodyFrameGraph>(
        "BodyFrameGraph", "Represents a body frame in the model graph, including its inertia.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::Inertia &>(
          bp::args("self", "inertia"), "Constructor initializing with a specific inertia."))
        .def_readwrite(
          "inertia", &BodyFrameGraph::inertia,
          "Spatial inertia of the body, expressed at its center of mass (CoM).")
        .def_readwrite(
          "f_type", &BodyFrameGraph::f_type, "Type of the frame (e.g., pinocchio.FrameType.BODY).");
      // Expose SensorFrameGraph struct to Python
      bp::class_<SensorFrameGraph>(
        "SensorFrameGraph", "Represents a sensor frame in the FrameGraph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &SensorFrameGraph::f_type,
          "Type of the frame (should be pinocchio.FrameType.SENSOR).");

      // Expose OpFrameGraph struct to Python
      bp::class_<OpFrameGraph>(
        "OpFrameGraph", "Represents an operational (task) frame in the FrameGraph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &OpFrameGraph::f_type,
          "Type of the frame (should be pinocchio.FrameType.OP_FRAME).");

      bp::to_python_converter<FrameGraphVariant, VariantToPythonVisitor<FrameGraphVariant>>();

      bp::implicitly_convertible<BodyFrameGraph, FrameGraphVariant>();
      bp::implicitly_convertible<SensorFrameGraph, FrameGraphVariant>();
      bp::implicitly_convertible<OpFrameGraph, FrameGraphVariant>();
    }

    void exposeJointsGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<JointFixedGraph>(
        "JointFixedGraph", "Represents a fixed joint in the graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::SE3 &>(
          bp::args("self", "pose"), "Constructor with joint offset."))
        .def_readwrite("joint_offset", &JointFixedGraph::joint_offset, "Offset of the joint.")
        .def_readwrite(
          "nq", &JointFixedGraph::nq, "Number of configuration variables (0 for fixed).");

      bp::class_<JointRevoluteGraph>(
        "JointRevoluteGraph", "Represents a revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevoluteGraph::axis, "Rotation axis.")
        .def_readwrite(
          "nq", &JointRevoluteGraph::nq, "Number of configuration variables (1 for revolute).");

      bp::class_<JointRevoluteUnboundedGraph>(
        "JointRevoluteUnboundedGraph", "Represents an unbounded revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevoluteUnboundedGraph::axis, "Rotation axis.")
        .def_readwrite(
          "nq", &JointRevoluteUnboundedGraph::nq,
          "Number of configuration variables (2 for unbounded revolute - typically pos/vel or "
          "sin/cos).");

      bp::class_<JointPrismaticGraph>(
        "JointPrismaticGraph", "Represents a prismatic joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with translation axis."))
        .def_readwrite("axis", &JointPrismaticGraph::axis, "Translation axis.")
        .def_readwrite(
          "nq", &JointPrismaticGraph::nq, "Number of configuration variables (1 for prismatic).");

      bp::class_<JointFreeFlyerGraph>(
        "JointFreeFlyerGraph", "Represents a free-flyer joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "nq", &JointFreeFlyerGraph::nq, "Number of configuration variables (7 for free-flyer).");

      bp::class_<JointSphericalGraph>(
        "JointSphericalGraph", "Represents a spherical joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "nq", &JointSphericalGraph::nq,
          "Number of configuration variables (4 for spherical - quaternion).");

      bp::class_<JointSphericalZYXGraph>(
        "JointSphericalZYXGraph", "Represents a spherical ZYX joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "nq", &JointSphericalZYXGraph::nq,
          "Number of configuration variables (3 for ZYX spherical).");

      bp::class_<JointTranslationGraph>(
        "JointTranslationGraph", "Represents a translation joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "nq", &JointTranslationGraph::nq,
          "Number of configuration variables (3 for translation).");

      bp::class_<JointPlanarGraph>(
        "JointPlanarGraph", "Represents a planar joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "nq", &JointPlanarGraph::nq,
          "Number of configuration variables (4 for planar - typically x, y, cos_theta, sin_theta "
          "or 3 if x,y,theta).");

      bp::class_<JointHelicalGraph>(
        "JointHelicalGraph", "Represents a helical joint.",
        bp::init<const Eigen::Vector3d &, double>(
          bp::args("self", "axis", "pitch"), "Constructor with axis and pitch."))
        .def_readwrite("axis", &JointHelicalGraph::axis, "Axis of the helical joint.")
        .def_readwrite("pitch", &JointHelicalGraph::pitch, "Pitch of the helical joint.")
        .def_readwrite(
          "nq", &JointHelicalGraph::nq, "Number of configuration variables (1 for helical).");

      bp::class_<JointUniversalGraph>(
        "JointUniversalGraph", "Represents a universal joint.",
        bp::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(
          bp::args("self", "axis1", "axis2"), "Constructor with two axes."))
        .def_readwrite("axis1", &JointUniversalGraph::axis1, "First axis of the universal joint.")
        .def_readwrite("axis2", &JointUniversalGraph::axis2, "Second axis of the universal joint.")
        .def_readwrite(
          "nq", &JointUniversalGraph::nq, "Number of configuration variables (2 for universal).");

      bp::class_<JointCompositeGraph>(
        "JointCompositeGraph", "Represents a composite joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointGraphVariant &, const pinocchio::SE3 &>(
          bp::args("self", "joint_variant", "joint_pose"),
          "Constructor with a single joint and its placement."))
        .def(bp::init<const std::vector<JointGraphVariant> &, const std::vector<SE3> &>(
          bp::args("self", "joints_variants", "joint_poses"),
          "Constructor with multiple joints and their placements."))
        .def_readwrite(
          "joints", &JointCompositeGraph::joints, "List of joints in the composite joint.")
        .def_readwrite(
          "jointsPlacements", &JointCompositeGraph::jointsPlacements,
          "List of placements for the joints.")
        .def_readwrite(
          "nq", &JointCompositeGraph::nq,
          "Total number of configuration variables for the composite joint.")
        .def(
          "addJoint", &JointCompositeGraph::addJoint,
          (bp::arg("self"), bp::arg("joint_model"), bp::arg("pose") = pinocchio::SE3::Identity()),
          "Adds a joint to the composite joint with an optional placement.");

      bp::class_<JointMimicGraph>(
        "JointMimicGraph", "Represents a mimic joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointGraphVariant &, const std::string &, double, double>(
          bp::args("self", "secondary_joint_model", "primary_name", "scaling", "offset"),
          "Constructor for mimic joint."))
        .def_readwrite(
          "primary_name", &JointMimicGraph::primary_name,
          "Name of the primary joint being mimicked.")
        .def_readwrite(
          "secondary_joint", &JointMimicGraph::secondary_joint,
          "The model of the secondary (mimicking) joint.")
        .def_readwrite("scaling", &JointMimicGraph::scaling, "Scaling factor for the mimicry.")
        .def_readwrite("offset", &JointMimicGraph::offset, "Offset for the mimicry.")
        .def_readwrite(
          "nq", &JointMimicGraph::nq,
          "Number of configuration variables (0 for mimic, as it depends on primary).");

      bp::to_python_converter<JointGraphVariant, VariantToPythonVisitor<JointGraphVariant>>();

      bp::implicitly_convertible<JointFixedGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointRevoluteGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointRevoluteUnboundedGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointPrismaticGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointFreeFlyerGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointSphericalGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointSphericalZYXGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointTranslationGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointPlanarGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointHelicalGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointUniversalGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointCompositeGraph, JointGraphVariant>();
      bp::implicitly_convertible<JointMimicGraph, JointGraphVariant>();

      StdAlignedVectorPythonVisitor<JointGraphVariant>::expose("StdVec_JointGraphVariant");
    }
  } // namespace python
} // namespace pinocchio
