//
// Copyright (c) 2025 INRIA 
//

#ifndef __pinocchio_parsers_opensim_parser_hpp__
#define __pinocchio_parsers_opensim_parser_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>


namespace pinocchio
{
  namespace opensim
  {
    PINOCCHIO_PARSERS_DLLAPI void addBody(const boost::property_tree::ptree & pt, const std::string & prefix, const std::string & path_geom, graph::ModelGraph & g);
    PINOCCHIO_PARSERS_DLLAPI void parseBodies(const boost::property_tree::ptree & pt, const std::string & prefix,  const std::string & path_geom, graph::ModelGraph & g);
    PINOCCHIO_PARSERS_DLLAPI void addJoint(const boost::property_tree::ptree & pt, const std::string & joint_name, const std::string & joint_type, graph::ModelGraph & g);
    PINOCCHIO_PARSERS_DLLAPI void addCustomJoint(const boost::property_tree::ptree & pt, const std::string & joint_name, graph::ModelGraph & g);
    PINOCCHIO_PARSERS_DLLAPI void parseJoints(const boost::property_tree::ptree & pt, graph::ModelGraph & g);

  } // namespace opensim
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_opensim_parser_hpp__
