//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace graph
  {
    // Define all static const int members for Boost.Python exposure
    const int JointFixed::nq;
    const int JointFixed::nv;

    const int JointRevolute::nq;
    const int JointRevolute::nv;

    const int JointRevoluteUnbounded::nq;
    const int JointRevoluteUnbounded::nv;

    const int JointPrismatic::nq;
    const int JointPrismatic::nv;

    const int JointFreeFlyer::nq;
    const int JointFreeFlyer::nv;

    const int JointSpherical::nq;
    const int JointSpherical::nv;

    const int JointSphericalZYX::nq;
    const int JointSphericalZYX::nv;

    const int JointTranslation::nq;
    const int JointTranslation::nv;

    const int JointPlanar::nq;
    const int JointPlanar::nv;

    const int JointHelical::nq;
    const int JointHelical::nv;

    const int JointUniversal::nq;
    const int JointUniversal::nv;

    const int JointMimic::nq;
    const int JointMimic::nv;
  } // namespace graph

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

      // Expose BodyFrame struct to Python
      bp::class_<BodyFrame>(
        "BodyFrame", "Represents a body frame in the model graph, including its inertia.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::Inertia &>(
          bp::args("self", "inertia"), "Constructor initializing with a specific inertia."))
        .def_readwrite(
          "inertia", &BodyFrame::inertia,
          "Spatial inertia of the body, expressed at its center of mass (CoM).")
        .def_readwrite(
          "f_type", &BodyFrame::f_type, "Type of the frame (e.g., pinocchio.FrameType.BODY).");
      // Expose SensorFrame struct to Python
      bp::class_<SensorFrame>(
        "SensorFrame", "Represents a sensor frame in the model graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &SensorFrame::f_type,
          "Type of the frame (should be pinocchio.FrameType.SENSOR).");

      // Expose OpFrame struct to Python
      bp::class_<OpFrame>(
        "OpFrame", "Represents an operational (task) frame in the model graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &OpFrame::f_type,
          "Type of the frame (should be pinocchio.FrameType.OP_FRAME).");

      bp::to_python_converter<FrameVariant, VariantToPythonVisitor<FrameVariant>>();

      bp::implicitly_convertible<BodyFrame, FrameVariant>();
      bp::implicitly_convertible<SensorFrame, FrameVariant>();
      bp::implicitly_convertible<OpFrame, FrameVariant>();
    }

    void exposeJointsGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<JointFixed>(
        "JointFixed", "Represents a fixed joint in the graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::SE3 &>(
          bp::args("self", "pose"), "Constructor with joint offset."))
        .def_readwrite("joint_offset", &JointFixed::joint_offset, "Offset of the joint.")
        .def_readonly("nq", &JointFixed::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointFixed::nv, "Number of tangent variables.");

      bp::class_<JointRevolute>(
        "JointRevolute", "Represents a revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevolute::axis, "Rotation axis.")
        .def_readonly("nq", &JointRevolute::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointRevolute::nv, "Number of tangent variables.");

      bp::class_<JointRevoluteUnbounded>(
        "JointRevoluteUnbounded", "Represents an unbounded revolute joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with rotation axis."))
        .def_readwrite("axis", &JointRevoluteUnbounded::axis, "Rotation axis.")
        .def_readonly("nq", &JointRevoluteUnbounded::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointRevoluteUnbounded::nv, "Number of tangent variables.");

      bp::class_<JointPrismatic>(
        "JointPrismatic", "Represents a prismatic joint.",
        bp::init<const Eigen::Vector3d &>(
          bp::args("self", "axis"), "Constructor with translation axis."))
        .def_readwrite("axis", &JointPrismatic::axis, "Translation axis.")
        .def_readonly("nq", &JointPrismatic::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointPrismatic::nv, "Number of tangent variables.");

      bp::class_<JointFreeFlyer>(
        "JointFreeFlyer", "Represents a free-flyer joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointFreeFlyer::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointFreeFlyer::nv, "Number of tangent variables.");

      bp::class_<JointSpherical>(
        "JointSpherical", "Represents a spherical joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointSpherical::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointSpherical::nv, "Number of tangent variables.");

      bp::class_<JointSphericalZYX>(
        "JointSphericalZYX", "Represents a spherical ZYX joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointSphericalZYX::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointSphericalZYX::nv, "Number of tangent variables.");

      bp::class_<JointTranslation>(
        "JointTranslation", "Represents a translation joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointTranslation::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointTranslation::nv, "Number of tangent variables.");

      bp::class_<JointPlanar>(
        "JointPlanar", "Represents a planar joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readonly("nq", &JointPlanar::nq, "Number of configuration variables ")
        .def_readonly("nv", &JointPlanar::nv, "Number of tangent variables ");

      bp::class_<JointHelical>(
        "JointHelical", "Represents a helical joint.",
        bp::init<const Eigen::Vector3d &, double>(
          bp::args("self", "axis", "pitch"), "Constructor with axis and pitch."))
        .def_readwrite("axis", &JointHelical::axis, "Axis of the helical joint.")
        .def_readwrite("pitch", &JointHelical::pitch, "Pitch of the helical joint.")
        .def_readonly("nq", &JointHelical::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointHelical::nv, "Number of tangent variables.");

      bp::class_<JointUniversal>(
        "JointUniversal", "Represents a universal joint.",
        bp::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(
          bp::args("self", "axis1", "axis2"), "Constructor with two axes."))
        .def_readwrite("axis1", &JointUniversal::axis1, "First axis of the universal joint.")
        .def_readwrite("axis2", &JointUniversal::axis2, "Second axis of the universal joint.")
        .def_readonly("nq", &JointUniversal::nq, "Number of configuration variables.")
        .def_readonly("nv", &JointUniversal::nv, "Number of tangent variables.");

      bp::class_<JointComposite>(
        "JointComposite", "Represents a composite joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointVariant &, const pinocchio::SE3 &>(
          bp::args("self", "joint_variant", "joint_pose"),
          "Constructor with a single joint and its placement."))
        .def(bp::init<const std::vector<JointVariant> &, const std::vector<SE3> &>(
          bp::args("self", "joints_variants", "joint_poses"),
          "Constructor with multiple joints and their placements."))
        .def_readwrite("joints", &JointComposite::joints, "List of joints in the composite joint.")
        .def_readwrite(
          "jointsPlacements", &JointComposite::jointsPlacements,
          "List of placements for the joints.")
        .def_readwrite(
          "nq", &JointComposite::nq,
          "Total number of configuration variables for the composite joint.")
        .def_readwrite(
          "nv", &JointComposite::nv,
          "Total number of configuration variables for the composite joint.")
        .def(
          "addJoint", &JointComposite::addJoint,
          (bp::arg("self"), bp::arg("joint_model"), bp::arg("pose") = pinocchio::SE3::Identity()),
          "Adds a joint to the composite joint with an optional placement.");

      bp::class_<JointMimic>(
        "JointMimic", "Represents a mimic joint.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const JointVariant &, const std::string &, double, double>(
          bp::args("self", "secondary_joint_model", "primary_name", "scaling", "offset"),
          "Constructor for mimic joint."))
        .def_readwrite(
          "primary_name", &JointMimic::primary_name, "Name of the primary joint being mimicked.")
        .def_readwrite(
          "secondary_joint", &JointMimic::secondary_joint,
          "The model of the secondary (mimicking) joint.")
        .def_readwrite("scaling", &JointMimic::scaling, "Scaling factor for the mimicry.")
        .def_readwrite("offset", &JointMimic::offset, "Offset for the mimicry.")
        .def_readonly(
          "nq", &JointMimic::nq,
          "Number of configuration variables (0 for mimic, as it depends on primary).")
        .def_readonly(
          "nv", &JointMimic::nv,
          "Number of tangent variables (0 for mimic, as it depends on primary).");

      bp::to_python_converter<JointVariant, VariantToPythonVisitor<JointVariant>>();

      bp::implicitly_convertible<JointFixed, JointVariant>();
      bp::implicitly_convertible<JointRevolute, JointVariant>();
      bp::implicitly_convertible<JointRevoluteUnbounded, JointVariant>();
      bp::implicitly_convertible<JointPrismatic, JointVariant>();
      bp::implicitly_convertible<JointFreeFlyer, JointVariant>();
      bp::implicitly_convertible<JointSpherical, JointVariant>();
      bp::implicitly_convertible<JointSphericalZYX, JointVariant>();
      bp::implicitly_convertible<JointTranslation, JointVariant>();
      bp::implicitly_convertible<JointPlanar, JointVariant>();
      bp::implicitly_convertible<JointHelical, JointVariant>();
      bp::implicitly_convertible<JointUniversal, JointVariant>();
      bp::implicitly_convertible<JointComposite, JointVariant>();
      bp::implicitly_convertible<JointMimic, JointVariant>();

      StdAlignedVectorPythonVisitor<JointVariant>::expose("StdVec_JointVariant");
    }
  } // namespace python
} // namespace pinocchio
