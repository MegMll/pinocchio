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
    void exposeModelGraphAlgo();

    inline void exposeModelGraph()
    {
      boost::python::scope graph = boost::python::class_<GraphNamespace>("graph");
      exposeFramesGraph();
      exposeJointsGraph();
      exposeModelGraphAlgo();
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_mjcf_hpp__
