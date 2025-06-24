//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_geometries_graph_hpp__
#define __pinocchio_parsers_geometries_graph_hpp__

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

    struct MeshGeom
    {
      std::string path;

      MeshGeom() = default;
      MeshGeom(const std::string & name_path)
      : path(name_path)
      {
      }
    };

    struct BoxGeom
    {
      Eigen::Vector3d size = Eigen::Vector3d::Constant(0);

      BoxGeom() = default;
      BoxGeom(const Eigen::Vector3d & size)
      : size(size)
      {
      }
    };

    struct CylinderGeom
    {
      Eigen::Vector2d size = Eigen::Vector2d::Constant(0);

      CylinderGeom() = default;
      CylinderGeom(const Eigen::Vector2d & size)
      : size(size)
      {
      }
    };

    struct CapsuleGeom
    {
      Eigen::Vector2d size = Eigen::Vector2d::Constant(0);

      CapsuleGeom() = default;
      CapsuleGeom(const Eigen::Vector2d & size)
      : size(size)
      {
      }
    };

    struct SphereGeom
    {
      double radius = 0;

      SphereGeom() = default;
      SphereGeom(const double r)
      : radius(r)
      {
      }
    };

    typedef boost::variant<MeshGeom, BoxGeom, CylinderGeom, CapsuleGeom, SphereGeom> GeomVariant;

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

#endif //__pinocchio_parsers_geometries_graph_hpp__
