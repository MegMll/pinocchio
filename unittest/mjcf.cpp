//
// Copyright (c) 2015-2024 CNRS INRIA
//

#include <iostream>
#include <cstdio> // for std::tmpnam

#include "pinocchio/multibody/model.hpp"

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/multibody/joint/joints.hpp"

#include <boost/test/unit_test.hpp>


static std::string createTempFile(const std::istringstream &data)
{
    char template_name[] = "/tmp/mytempfileXXXXXX"; // Template for the temporary file name
    // Create the temporary file securely
    if ((mkstemp(template_name)) == -1) {
        perror("mkstemp"); // Print an error message if mkstemp fails
        exit(EXIT_FAILURE);
    }

    // Write the XML data to the temporary file
    std::ofstream file_stream;
    file_stream.open(template_name);
    if (!file_stream) {
        std::cerr << "Error opening file for writing" << std::endl;
        exit(EXIT_FAILURE);
    }
    file_stream << data.rdbuf();
    file_stream.close();

    return std::string(template_name);
}

static bool comparePropertyTrees(const boost::property_tree::ptree& pt1, const boost::property_tree::ptree& pt2) 
{
    // Check if the number of children is the same
    if (pt1.size() != pt2.size())
        return false;

    // Iterate over all children
    for (auto it1 = pt1.begin(), it2 = pt2.begin(); it1 != pt1.end(); ++it1, ++it2) {
        // Compare keys
        if (it1->first != it2->first)
            return false;
        
        // Compare values
        if (it1->second.data() != it2->second.data())
            return false;
        
        // Recursively compare child trees
        if (!comparePropertyTrees(it1->second, it2->second))
            return false;
    }

    return true;
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

// /// @brief Test for the inertia conversion from mjcf to pinocchio inertia
// /// @param  
BOOST_AUTO_TEST_CASE(convert_inertia_fullinertia)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;
    // Parse the XML
    std::istringstream xmlData(R"(
        <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
    )");

    // Create a Boost Property Tree
    boost::property_tree::ptree pt;
    boost::property_tree::read_xml(xmlData, pt);

    pinocchio::Model model;
    pinocchio::urdf::details::UrdfVisitor visitor (model);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

    MjcfGraph graph (visitor);

    //  // Try to get the "name" element using get_optional
    pinocchio::Inertia inertia = graph.convertInertiaFromMjcf(pt.get_child("inertial"));

    Matrix3 inertia_matrix;
    // M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)
    inertia_matrix << 0.00315, 8.2904e-7, 0.00015, 8.2904e-7, 0.00388, 8.2299e-6, 0.00015, 8.2299e-6, 0.004285;

    pinocchio::Inertia real_inertia(0.629769, Vector3(-0.041018, -0.00014, 0.049974), inertia_matrix);
    BOOST_CHECK(inertia.isEqual(real_inertia));
}

// /// @brief Test for the inertia conversion from mjcf to pinocchio inertia
// /// @param  
BOOST_AUTO_TEST_CASE(convert_inertia_diaginertia)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;
    // Parse the XML
    std::istringstream xmlData(R"(
        <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        diaginertia="0.00315 0.00388 0.004285"/>
    )");

    // Create a Boost Property Tree
    boost::property_tree::ptree pt;
    boost::property_tree::read_xml(xmlData, pt);

    pinocchio::Model model;
    pinocchio::urdf::details::UrdfVisitor visitor (model);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

    MjcfGraph graph (visitor);

    //  // Try to get the "name" element using get_optional
    pinocchio::Inertia inertia = graph.convertInertiaFromMjcf(pt.get_child("inertial"));

    Matrix3 inertia_matrix = Eigen::Matrix3d::Zero();
    inertia_matrix(0, 0) = 0.00315;
    inertia_matrix(1, 1) = 0.00388;
    inertia_matrix(2, 2) = 0.004285;
    pinocchio::Inertia real_inertia(0.629769, Vector3(-0.041018, -0.00014, 0.049974), inertia_matrix);

    BOOST_CHECK(inertia.isApprox(real_inertia, 1e-7));
}

// /// @brief Test for the pose conversion from mjcf model to pinocchio
// /// @param  
BOOST_AUTO_TEST_CASE(convert_orientation)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
        <compiler angle="radian" eulerseq="xyz"/>
        <quaternion pos="0.3 0.2 0.5" quat="1 -1 0 0"/>
        <axis pos="0.3 0.2 0.5" axisangle="-1 0 0 1.5707963"/>
        <euler pos="0.3 0.2 0.5" euler="-1.57079633 0 0"/>
        <xyaxes pos="0.3 0.2 0.5" xyaxes="1 0 0 0 0 -1"/>
        <zaxis pos="0.3 0.2 0.5" zaxis="0 -1 0"/>
    )");

    // Create a Boost Property Tree
    boost::property_tree::ptree pt;
    boost::property_tree::read_xml(xmlData, pt);

    pinocchio::Model model;
    pinocchio::urdf::details::UrdfVisitor visitor (model);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

    MjcfGraph graph (visitor);

    graph.parseCompiler(pt.get_child("compiler"));

    pinocchio::SE3 placement_q = graph.convertPosition(pt.get_child("quaternion"));
    pinocchio::SE3 placement_a = graph.convertPosition(pt.get_child("axis"));
    pinocchio::SE3 placement_e = graph.convertPosition(pt.get_child("euler"));
    pinocchio::SE3 placement_xy = graph.convertPosition(pt.get_child("xyaxes"));
    pinocchio::SE3 placement_z = graph.convertPosition(pt.get_child("zaxis"));

    Matrix3 rotation_matrix;
    rotation_matrix <<  1., 0.,  0.,
                        0.,  0.,  1.,
                        0., -1.,  0.;

    pinocchio::SE3 real_placement(rotation_matrix, Vector3(0.3, 0.2, 0.5));

    BOOST_CHECK(placement_q.isApprox(real_placement, 1e-7));
    BOOST_CHECK(placement_e.isApprox(real_placement, 1e-7));
    BOOST_CHECK(placement_a.isApprox(real_placement, 1e-7));
    BOOST_CHECK(placement_xy.isApprox(real_placement, 1e-7));
    BOOST_CHECK(placement_z.isApprox(real_placement, 1e-7));
}

BOOST_AUTO_TEST_CASE(merge_default)
{
namespace pt = boost::property_tree;
    std::istringstream xmlIn(R"(
            <default>
                <default class="mother">
                    <joint A="a0" B="b0" />
                    <geom A="a0" />
                    <default class="layer1">
                        <joint A="a1" C="c1" />
                        <default class="layer2">
                            <joint B="b2" />
                            <geom C="c2" />
                        </default>
                    </default>
                    <default class="layerP">
                        <joint K="b2" />
                        <site  H="f1"/>
                    </default>
                </default>
            </default>)");

    // Create a Boost Property Tree
    pt::ptree ptr;
    pt::read_xml(xmlIn, ptr, pt::xml_parser::trim_whitespace);

    pinocchio::Model model;
    pinocchio::urdf::details::UrdfVisitor visitor (model);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

    MjcfGraph graph (visitor);
    graph.parseDefault(ptr.get_child("default"), ptr);

    std::unordered_map<std::string, pt::ptree> TrueMap;
    
    std::istringstream xml1(R"(<default class="mother">
                                    <joint A="a0" B="b0" />
                                    <geom A="a0" />
                                    <default class="layer1">
                                        <joint A="a1" C="c1" />
                                        <default class="layer2">
                                            <joint B="b2" />
                                            <geom C="c2" />
                                        </default>
                                    </default>
                                    <default class="layerP">
                                        <joint K="b2" />
                                        <site  H="f1"/>
                                    </default>
                                </default>)");
    pt::ptree p1;
    pt::read_xml(xml1, p1, pt::xml_parser::trim_whitespace);
    BOOST_CHECK(comparePropertyTrees(graph.mapOfClasses.at("mother").classElement,  p1.get_child("default")));

    std::istringstream xml2(R"(<default class="layer1">
                                    <joint A="a1" B="b0" C="c1" />
                                    <default class="layer2">
                                        <joint B="b2" />
                                        <geom C="c2" />
                                    </default>
                                    <geom A="a0" />
                                </default>)");
    pt::ptree p2;
    pt::read_xml(xml2, p2, pt::xml_parser::trim_whitespace);
    BOOST_CHECK(comparePropertyTrees(graph.mapOfClasses.at("layer1").classElement,  p2.get_child("default")));
    std::string name = "layer1";
    TrueMap.insert(std::make_pair(name, p2.get_child("default")));
    
    std::istringstream xml3(R"(<default class="layer2">
                                    <joint A="a1" B="b2" C="c1"/>
                                    <geom A="a0" C="c2"/>
                                </default>)");
    pt::ptree p3;
    pt::read_xml(xml3, p3, pt::xml_parser::trim_whitespace);
    BOOST_CHECK(comparePropertyTrees(graph.mapOfClasses.at("layer2").classElement,  p3.get_child("default")));
    
    std::istringstream xml4(R"(<default class="layerP">
                                    <joint A="a0" B="b0" K="b2"/>
                                    <site H="f1"/>
                                    <geom A="a0"/>
                                </default>)");
    pt::ptree p4;
    pt::read_xml(xml4, p4, pt::xml_parser::trim_whitespace);
    BOOST_CHECK(comparePropertyTrees(graph.mapOfClasses.at("layerP").classElement,  p4.get_child("default")));
}

// @brief Test to check that default classes and child classes are taken into account
BOOST_AUTO_TEST_CASE(parse_default_class)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::string filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_mjcf.xml");

    pinocchio::Model model_m;
    pinocchio::mjcf::buildModel(filename, model_m);

    std::string file_u = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_mjcf.urdf");
    pinocchio::Model model_u;
    pinocchio::urdf::buildModel(file_u, model_u);
    
    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    idx = model_u.addJoint(model_u.njoints - 1, pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(), "joint3");
    model_u.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], model_u.joints[i]);
}

//Test for parsing Revolute
BOOST_AUTO_TEST_CASE(parse_RX)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="model_RX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelRX;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    idx = modelRX.addJoint(0, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "rx");
    modelRX.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelRX.joints[i]);
}

// Test for parsing prismatic
BOOST_AUTO_TEST_CASE(parse_PX)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="model_PX">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelPX;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    idx = modelPX.addJoint(0, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "px");
    modelPX.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelPX.joints[i]);
}

// Test parsing sphere
BOOST_AUTO_TEST_CASE(parse_Sphere)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="model_Sphere">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="ball"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelS;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    idx = modelS.addJoint(0, pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(), "s");
    modelS.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelS.joints[i]);
}

// Test parsing free flyer
BOOST_AUTO_TEST_CASE(parse_Free)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="model_Free">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <freejoint name="joint1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelF;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    idx = modelF.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "f");
    modelF.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelF.joints[i]);
}

//Test for Composite RXRY
BOOST_AUTO_TEST_CASE(parse_composite_RXRY)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="composite_RXRY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="hinge" axis="1 0 0"/>
                            <joint name="joint2" type="hinge" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelRXRY;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    pinocchio::JointModelComposite joint_model_RXRY;
    joint_model_RXRY.addJoint(pinocchio::JointModelRX());
    joint_model_RXRY.addJoint(pinocchio::JointModelRY());
    
    idx = modelRXRY.addJoint(0, joint_model_RXRY, pinocchio::SE3::Identity(), "rxry");
    modelRXRY.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelRXRY.joints[i]);
}

//Test for Composite PXPY
BOOST_AUTO_TEST_CASE(parse_composite_PXPY)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="composite_PXPY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="slide" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelPXPY;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    pinocchio::JointModelComposite joint_model_PXPY;
    joint_model_PXPY.addJoint(pinocchio::JointModelPX());
    joint_model_PXPY.addJoint(pinocchio::JointModelPY());
    
    idx = modelPXPY.addJoint(0, joint_model_PXPY, pinocchio::SE3::Identity(), "pxpy");
    modelPXPY.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelPXPY.joints[i]);
}

//Test for Composite PXRY
BOOST_AUTO_TEST_CASE(parse_composite_PXRY)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="composite_PXRY">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="hinge" axis="0 1 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelPXRY;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    pinocchio::JointModelComposite joint_model_PXRY;
    joint_model_PXRY.addJoint(pinocchio::JointModelPX());
    joint_model_PXRY.addJoint(pinocchio::JointModelRY());
    
    idx = modelPXRY.addJoint(0, joint_model_PXRY, pinocchio::SE3::Identity(), "pxry");
    modelPXRY.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelPXRY.joints[i]);
}

//Test for Composite PXSphere
BOOST_AUTO_TEST_CASE(parse_composite_PXSphere)
{
    typedef pinocchio::SE3::Vector3 Vector3;
    typedef pinocchio::SE3::Matrix3 Matrix3;

    std::istringstream xmlData(R"(
            <mujoco model="composite_PXSphere">
                <worldbody>
                    <body name="link0">
                        <body name="link1" pos="0 0 0">
                            <joint name="joint1" type="slide" axis="1 0 0"/>
                            <joint name="joint2" type="ball"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>)");

    std::string namefile = createTempFile(xmlData);

    pinocchio::Model model_m, modelPXSphere;
    pinocchio::urdf::details::UrdfVisitor visitor (model_m);
    typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;
    
    MjcfGraph graph (visitor);
    graph.parseGraphFromXML(namefile);
    graph.parseRootTree();

    pinocchio::Model::JointIndex idx;
    pinocchio::Inertia inertia (0, Vector3 (0.0, 0., 0.0), Matrix3::Identity ());
    
    pinocchio::JointModelComposite joint_model_PXSphere;
    joint_model_PXSphere.addJoint(pinocchio::JointModelPX());
    joint_model_PXSphere.addJoint(pinocchio::JointModelSpherical());
    
    idx = modelPXSphere.addJoint(0, joint_model_PXSphere, pinocchio::SE3::Identity(), "pxsphere");
    modelPXSphere.appendBodyToJoint(idx,inertia);

    for(int i = 0; i < model_m.njoints; i++)
        BOOST_CHECK_EQUAL(model_m.joints[i], modelPXSphere.joints[i]);
}

// Test loading a mujoco composite and compare body position with mujoco results
BOOST_AUTO_TEST_CASE(parse_composite_Mujoco_comparison)
{
    using namespace pinocchio;
    std::string filename = PINOCCHIO_MODEL_DIR + std::string("/../unittest/models/test_composite.xml");   

    pinocchio::Model model;
    pinocchio::mjcf::buildModel(filename, model);

    Data data(model);
    Eigen::Vector3d q;
    q << 1.57079633, 0, 0.5;

    framesForwardKinematics(model, data, q);

    FrameIndex f_id = model.getBodyId("body1");
    SE3 pinPos = data.oMf[f_id];
    Eigen::Matrix3d refOrient;
    refOrient << 0, -1, 0,
                 1, 0, 0,
                 0, 0, 1;
    Eigen::Vector3d pos;
    pos << 1, 2, 0.5;

    BOOST_CHECK(pinPos.isApprox(SE3(refOrient, pos), 1e-7));

    f_id = model.getBodyId("body2");
    pinPos = data.oMf[f_id];
    refOrient << 0, -1, 0,
                 1, 0, 0,
                 0, 0, 1;

    pos << 1, 4, 0.5;
    BOOST_CHECK(pinPos.isApprox(SE3(refOrient, pos), 1e-7));
}
/// @brief test that a fixed model is well parsed 
/// @param  
BOOST_AUTO_TEST_CASE (build_model_no_root_joint)
{
    using namespace pinocchio;
    const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.xml");

    pinocchio::Model model_m;
    pinocchio::mjcf::buildModel(filename, model_m);

    BOOST_CHECK_EQUAL(model_m.nq, 29);
}

/// @brief Test all the data of the humanoid model (Need to find the urdf yet)
/// @param  
BOOST_AUTO_TEST_CASE (compare_to_urdf)
{
    using namespace pinocchio;
    typedef typename pinocchio::Model::ConfigVectorMap ConfigVectorMap;

    const std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.xml");

    Model model_m;
    pinocchio::mjcf::buildModel(filename, pinocchio::JointModelFreeFlyer(), model_m);
    
    const std::string filename_urdf = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
    const std::string dir_urdf = PINOCCHIO_MODEL_DIR;
    pinocchio::Model model_urdf;
    pinocchio::urdf::buildModel(filename_urdf, pinocchio::JointModelFreeFlyer(), model_urdf);

    BOOST_CHECK(model_urdf.nq == model_m.nq);
    BOOST_CHECK(model_urdf.nv == model_m.nv);
    BOOST_CHECK(model_urdf.njoints == model_m.njoints);
    BOOST_CHECK(model_urdf.nbodies == model_m.nbodies);
    BOOST_CHECK(model_urdf.nframes == model_m.nframes);
    BOOST_CHECK(model_urdf.parents == model_m.parents);
    BOOST_CHECK(model_urdf.children == model_m.children);
    BOOST_CHECK(model_urdf.names == model_m.names);
    BOOST_CHECK(model_urdf.subtrees == model_m.subtrees);
    BOOST_CHECK(model_urdf.gravity == model_m.gravity);
    BOOST_CHECK(model_urdf.name == model_m.name);
    BOOST_CHECK(model_urdf.idx_qs == model_m.idx_qs);
    BOOST_CHECK(model_urdf.nqs == model_m.nqs);
    BOOST_CHECK(model_urdf.idx_vs == model_m.idx_vs);
    BOOST_CHECK(model_urdf.nvs == model_m.nvs);

    typename ConfigVectorMap::const_iterator it = model_m.referenceConfigurations.begin();
    typename ConfigVectorMap::const_iterator it_model_urdf = model_urdf.referenceConfigurations.begin();
    for(long k = 0; k < (long)model_m.referenceConfigurations.size(); ++k)
    {
        std::advance(it,k); std::advance(it_model_urdf,k);
        BOOST_CHECK(it->second.size() == it_model_urdf->second.size());
        BOOST_CHECK(it->second == it_model_urdf->second);    
    }

    BOOST_CHECK(model_urdf.armature.size() == model_m.armature.size());

    BOOST_CHECK(model_urdf.armature == model_m.armature);
    BOOST_CHECK(model_urdf.friction.size() == model_m.friction.size());
    BOOST_CHECK(model_urdf.friction == model_m.friction);

    BOOST_CHECK(model_urdf.damping.size() == model_m.damping.size());

    BOOST_CHECK(model_urdf.damping == model_m.damping);

    BOOST_CHECK(model_urdf.rotorInertia.size() == model_m.rotorInertia.size());

    BOOST_CHECK(model_urdf.rotorInertia == model_m.rotorInertia);

    BOOST_CHECK(model_urdf.rotorGearRatio.size() == model_m.rotorGearRatio.size());

    BOOST_CHECK(model_urdf.rotorGearRatio == model_m.rotorGearRatio);

    BOOST_CHECK(model_urdf.effortLimit.size() == model_m.effortLimit.size());
    BOOST_CHECK(model_urdf.effortLimit == model_m.effortLimit);
    // Cannot test velocity limit since it does not exist in mjcf

    BOOST_CHECK(model_urdf.lowerPositionLimit.size() == model_m.lowerPositionLimit.size());
    BOOST_CHECK(model_urdf.lowerPositionLimit == model_m.lowerPositionLimit);

    BOOST_CHECK(model_urdf.upperPositionLimit.size() == model_m.upperPositionLimit.size());
    BOOST_CHECK(model_urdf.upperPositionLimit == model_m.upperPositionLimit);

    for(size_t k = 1; k < model_m.inertias.size(); ++k)
    {
    BOOST_CHECK(model_urdf.inertias[k].isApprox(model_m.inertias[k]));
    }

    for(size_t k = 1; k < model_urdf.jointPlacements.size(); ++k)
    {
    BOOST_CHECK(model_urdf.jointPlacements[k] == model_m.jointPlacements[k]);
    }

    BOOST_CHECK(model_urdf.joints == model_m.joints);

    BOOST_CHECK(model_urdf.frames.size() == model_m.frames.size());
    for(size_t k = 1; k < model_urdf.frames.size(); ++k)
    {
    BOOST_CHECK(model_urdf.frames[k] == model_m.frames[k]);
    }
}

BOOST_AUTO_TEST_SUITE_END()
