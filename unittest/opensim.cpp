//
// Copyright (c) 2025 INRIA 
//

#include "pinocchio/parsers/opensim.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"
#include "pinocchio/parsers/graph/model-graph-algo-geometry.hpp"

// #include "pinocchio/parsers/opensim/opensim-parser.hpp"

#include <iostream>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)


BOOST_AUTO_TEST_CASE(test_parse_model)
{
    std::string filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/Arm26/arm26.osim");
    std::string path_geom = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/Arm26/Geometry/");
    pinocchio::graph::ModelGraph g = pinocchio::opensim::parseModel(filename, path_geom);

    pinocchio::Model m = pinocchio::graph::buildModel(g, "/ground", pinocchio::SE3::Identity());
    std::cout << m << std::endl;
    
    pinocchio::GeometryModel gm = pinocchio::graph::buildGeometryModel(g, m, pinocchio::VISUAL);
}

BOOST_AUTO_TEST_SUITE_END()