//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_geometries_hpp__
#define __pinocchio_parsers_graph_geometries_hpp__

#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  namespace graph
  {
    // Kind of possible geometry
    enum struct GeomType
    {
      VISUAL,
      COLLISION,
      BOTH
    };

    struct Mesh
    {
      std::string path;

      Mesh() = default;
      Mesh(const std::string & name_path)
      : path(name_path)
      {
      }
    };

    struct Box
    {
      Eigen::Vector3d size = Eigen::Vector3d::Constant(0);

      Box() = default;
      Box(const Eigen::Vector3d & size)
      : size(size)
      {
      }
    };

    struct Cylinder
    {
      Eigen::Vector2d size = Eigen::Vector2d::Constant(0);

      Cylinder() = default;
      Cylinder(const Eigen::Vector2d & size)
      : size(size)
      {
      }
    };

    struct Capsule
    {
      Eigen::Vector2d size = Eigen::Vector2d::Constant(0);

      Capsule() = default;
      Capsule(const Eigen::Vector2d & size)
      : size(size)
      {
      }
    };

    struct Sphere
    {
      double radius = 0;

      Sphere() = default;
      Sphere(const double r)
      : radius(r)
      {
      }
    };

    typedef boost::variant<Mesh, Box, Cylinder, Capsule, Sphere> GeomVariant;

    struct Geometry
    {
      std::string name;

      GeomType type = GeomType::BOTH;

      Eigen::Vector3d scale = Eigen::Vector3d::Constant(1);

      Eigen::Vector4d color = Eigen::Vector4d::Constant(1);

      SE3 placement = SE3::Identity();

      GeomVariant geometry;

      Geometry() = default;

      Geometry(
        const std::string & name,
        const SE3 & placement,
        const GeomType & type,
        const Eigen::Vector3d & scale,
        const Eigen::Vector4d & color,
        const GeomVariant & geom)
      : name(name)
      , placement(placement)
      , scale(scale)
      , color(color)
      , type(type)
      , geometry(geom)
      {
      }
    };
  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_geometries_hpp__
