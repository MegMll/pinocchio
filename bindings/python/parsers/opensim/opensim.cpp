//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/opensim.hpp"
#include "pinocchio/bindings/python/parsers/opensim.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeOpensimModel()
    {
      bp::def(
        "parseModelFromOpensim",&pinocchio::opensim::parseModel,
        (bp::arg("filename"), bp::arg("path_geom"), bp::arg("verbose")=false),
        "Parse the Opensim file given in input and return a pinocchio ModelGraph.");
    }

  }
}