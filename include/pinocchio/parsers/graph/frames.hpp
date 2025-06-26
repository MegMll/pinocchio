//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_frames_hpp__
#define __pinocchio_parsers_graph_frames_hpp__

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/fwd.hpp"

namespace pinocchio
{
  namespace graph
  {
    struct BodyFrame
    {
      /// @brief Spatial inertia of the body, expressed at its center of mass (CoM).
      ///
      /// Note: If the joint is reversed in the model graph, the body frame pose
      /// is kept the same in the model, so this inertia remains valid.
      Inertia inertia = pinocchio::Inertia::Identity();

      pinocchio::FrameType f_type = BODY;

      BodyFrame() = default;
      BodyFrame(const pinocchio::Inertia & in)
      : inertia(in)
      {
      }
    };

    struct SensorFrame
    {
      pinocchio::FrameType f_type = SENSOR;

      SensorFrame() = default;
    };

    struct OpFrame
    {
      pinocchio::FrameType f_type = OP_FRAME;

      OpFrame() = default;
    };

    typedef boost::variant<BodyFrame, SensorFrame, OpFrame> FrameVariant;

  } // namespace graph
} // namespace pinocchio
#endif // ifndef __pinocchio_parsers_graph_frames_hpp__
