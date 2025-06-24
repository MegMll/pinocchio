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

    void exposeGeometriesVariant()
    {      
        using namespace pinocchio::graph;

        bp::class_<pinocchio::graph::Mesh>("Mesh", "Represents a mesh geometry.", bp::init<>())
            .def(bp::init<const std::string &>((bp::arg("path")), "Constructor from mesh file path."))
            .def_readwrite("path", &pinocchio::graph::Mesh::path, "Path to the mesh file.");

        bp::class_<pinocchio::graph::Box>("Box", "Represents a box.", bp::init<>())
            .def(bp::init<const Eigen::Vector3d &>((bp::arg("size")), "Constructor from 3D size vector."))
            .def_readwrite("size", &pinocchio::graph::Box::size, "Size of the box.");

        bp::class_<pinocchio::graph::Cylinder>("Cylinder", "Represents a cylinder.", bp::init<>())
            .def(bp::init<const Eigen::Vector2d &>((bp::arg("size")), "Constructor from 2D size vector (radius, height)."))
            .def_readwrite("size", &pinocchio::graph::Cylinder::size, "Size of the cylinder (radius, height).");

        bp::class_<pinocchio::graph::Capsule>("Capsule", "Represents a capsule.", bp::init<>())
            .def(bp::init<const Eigen::Vector2d &>((bp::arg("size")), "Constructor from 2D size vector (radius, length)."))
            .def_readwrite("size", &pinocchio::graph::Capsule::size, "Size of the capsule (radius, length).");

        bp::class_<pinocchio::graph::Sphere>("Sphere", "Represents a sphere.", bp::init<>())
            .def(bp::init<const double>((bp::arg("radius")), "Constructor from radius."))
            .def_readwrite("radius", &pinocchio::graph::Sphere::radius, "Radius of the sphere.");


        bp::to_python_converter<pinocchio::graph::GeomVariant, VariantToPythonVisitor<GeomVariant>>();
        bp::implicitly_convertible<pinocchio::graph::Mesh, pinocchio::graph::GeomVariant>();
        bp::implicitly_convertible<pinocchio::graph::Box, pinocchio::graph::GeomVariant>();
        bp::implicitly_convertible<pinocchio::graph::Cylinder, pinocchio::graph::GeomVariant>();
        bp::implicitly_convertible<pinocchio::graph::Capsule, pinocchio::graph::GeomVariant>();
        bp::implicitly_convertible<pinocchio::graph::Sphere, pinocchio::graph::GeomVariant>();
    }

    void exposeGeometryGraph()
    {
        using namespace pinocchio::graph;

        bp::enum_<pinocchio::graph::GeomType>("GeomType")
        .value("VISUAL", pinocchio::graph::GeomType::VISUAL)
        .value("COLLISION", pinocchio::graph::GeomType::COLLISION)
        .value("BOTH", pinocchio::graph::GeomType::BOTH)
        .export_values();

        bp::class_<pinocchio::graph::Geometry>("Geometry", "Main geometry object containing all properties for the Model Graph.", bp::init<>())
        .def(bp::init<const std::string &, const pinocchio::SE3 &, const pinocchio::graph::GeomType &,
                      const Eigen::Vector3d &, const Eigen::Vector4d &, const pinocchio::graph::GeomVariant &>(
             (bp::arg("name"), bp::arg("placement"), bp::arg("type"), bp::arg("scale"), bp::arg("color"), bp::arg("geometry")),
             "Full constructor for Geometry."
        ))
        .def_readwrite("name", &pinocchio::graph::Geometry::name, "Name of the geometry object.")
        .def_readwrite("type", &pinocchio::graph::Geometry::type, "Type of geometry (VISUAL, COLLISION, BOTH).")
        .def_readwrite("scale", &pinocchio::graph::Geometry::scale, "Scaling factors.")
        .def_readwrite("color", &pinocchio::graph::Geometry::color, "RGBA color.")
        .def_readwrite("placement", &pinocchio::graph::Geometry::placement, "SE3 placement (pose) of the geometry wrt to the body.")
        .def_readwrite("geometry", &pinocchio::graph::Geometry::geometry, "The actual geometric primitive.");

        StdAlignedVectorPythonVisitor<Geometry>::expose("StdVec_Geometry");
    }

    void exposeGeometryBuilder()
    {
      using namespace pinocchio::graph;

        bp::class_<pinocchio::graph::GeometryBuilder>("GeometryBuilder", "A builder for Geometries in Model Graph.", bp::init<pinocchio::graph::ModelGraph &>((bp::arg("model_graph"))))
        .def("withName", &pinocchio::graph::GeometryBuilder::withName, bp::return_self<>(),
             (bp::arg("name")), "Sets the name of the geometry.")
        .def("withBody", &pinocchio::graph::GeometryBuilder::withBody, bp::return_self<>(),
             (bp::arg("body_name")), "Sets the name of the body this geometry is attached to.")
        .def("withPlacement", &pinocchio::graph::GeometryBuilder::withPlacement, bp::return_self<>(),
             (bp::arg("placement")), "Sets the SE3 placement of the geometry.")
        .def("withScale", &pinocchio::graph::GeometryBuilder::withScale, bp::return_self<>(),
             (bp::arg("scale")), "Sets the scale factors.")
        .def("withColor", &pinocchio::graph::GeometryBuilder::withColor, bp::return_self<>(),
             (bp::arg("color")), "Sets the RGBA color.")
        .def("withGeomType", &pinocchio::graph::GeometryBuilder::withGeomType, bp::return_self<>(),
             (bp::arg("geom_type")), "Sets the type of geometry (VISUAL, COLLISION, BOTH).")
        .def("withGeom", &pinocchio::graph::GeometryBuilder::withGeom, bp::return_self<>(),
             (bp::arg("geometry_primitive")), "Sets the geometry primitive .")
        .def("build", &pinocchio::graph::GeometryBuilder::build,
             "Builds the Geometry object and adds it to the right vertex in the ModelGraph "
             "Throws if geometry name is empty.");
    }
  } // namespace python
} // namespace pinocchio
