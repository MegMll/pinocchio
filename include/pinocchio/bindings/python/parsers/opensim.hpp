//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_python_parsers_opensim_hpp__
#define __pinocchio_python_parsers_opensim_hpp__

namespace pinocchio
{
  namespace python
  {
    void exposeOpensimModel();

    inline void exposeOpensimParser()
    {
      exposeOpensimModel();
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_opensim_hpp__
