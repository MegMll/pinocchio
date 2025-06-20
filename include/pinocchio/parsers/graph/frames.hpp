//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_frame_graph_hpp__
#define __pinocchio_parsers_frame_graph_hpp__

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/fwd.hpp"

namespace pinocchio
{
  namespace graph
  {
    struct MeshGeom
    {
    };

    struct BoxGeom
    {
    };

    struct CylinderGeom
    {
    };

    struct CapsuleGeom
    {
    };

    struct SphereGeom
    {
    };

    typedef boost::variant<MeshGeom, BoxGeom, CylinderGeom, CapsuleGeom, SphereGeom> GeomVariant;

    struct BodyFrameGraph
    {
      /// @brief Spatial inertia of the body, expressed at its center of mass (CoM).
      ///
      /// Note: If the joint is reversed in the model graph, the body frame pose
      /// is kept the same in the model, so this inertia remains valid.
      Inertia inertia = pinocchio::Inertia::Identity();

      pinocchio::FrameType f_type = BODY;

      BodyFrameGraph() = default;
      BodyFrameGraph(const pinocchio::Inertia & in)
      : inertia(in)
      {
      }
    };

    struct SensorFrameGraph
    {
      pinocchio::FrameType f_type = SENSOR;

      SensorFrameGraph() = default;
    };

    struct OpFrameGraph
    {
      pinocchio::FrameType f_type = OP_FRAME;

      OpFrameGraph() = default;
    };

    typedef boost::variant<BodyFrameGraph, SensorFrameGraph, OpFrameGraph> FrameGraphVariant;

  } // namespace graph
} // namespace pinocchio
#endif
