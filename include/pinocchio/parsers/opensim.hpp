//
// Copyright (c) 2025 INRIA 
//

#ifndef __pinocchio_parsers_opensim_hpp__
#define __pinocchio_parsers_opensim_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"

namespace pinocchio
{
  namespace opensim
  {
    PINOCCHIO_PARSERS_DLLAPI graph::ModelGraph parseModel(const std::string & filename, const std::string & path_geom, const bool verbose=false);
    PINOCCHIO_PARSERS_DLLAPI void buildModel(const std::string & filename, Model & model, const bool verbose = false);

  } // namespace opensim
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_opensim_hpp__
