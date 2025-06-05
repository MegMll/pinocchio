//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_parsers_model_graph_hpp__
#define __pinocchio_python_parsers_model_graph_hpp__

namespace pinocchio
{
  namespace python
  {
    void exposeFramesGraph();
    void exposeJointsGraph();
    void exposeModelGraphAlgo();

    inline void exposeModelGraph()
    {
      exposeFramesGraph();
      exposeJointsGraph();
      exposeModelGraphAlgo();
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_mjcf_hpp__
