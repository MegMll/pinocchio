//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_parsers_model_graph_hpp__
#define __pinocchio_python_parsers_model_graph_hpp__

#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

namespace pinocchio
{
  namespace python
  {
    struct GraphNamespace
    {
    };

    void exposeFramesGraph();
    void exposeJointsGraph();
    void exposeJointLimits();
    void exposeEdgesAlgo();
    void exposeGeometriesVariant();
    void exposeGeometryGraph();
    void exposeGeometryBuilder();
    void exposeModelGraph();
    void exposeModelGraphAlgo();

    void exposeAlgoGeometry();

    inline void exposeGraph()
    {
      boost::python::scope graph = boost::python::class_<GraphNamespace>("graph");

      exposeFramesGraph();
      exposeJointsGraph();
      exposeJointLimits();
      exposeEdgesAlgo();
      exposeGeometriesVariant();
      exposeGeometryGraph();
      exposeGeometryBuilder();
      exposeModelGraph();
      exposeModelGraphAlgo();
#if defined(PINOCCHIO_WITH_HPP_FCL)
      exposeAlgoGeometry();
#endif
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_mjcf_hpp__
