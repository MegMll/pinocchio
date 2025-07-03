//
// Copyright (c) 2025 INRIA 
//

#include "pinocchio/parsers/opensim.hpp"
#include "pinocchio/parsers/opensim/opensim-parser.hpp"
#include "pinocchio/parsers/mjcf/mjcf-graph.hpp"

#include <iostream>

namespace pinocchio
{
  namespace opensim
  {
    void addBody(const boost::property_tree::ptree & pt, const std::string & prefix, const std::string & path_geom, graph::ModelGraph & g)
    {
        std::string name;
        Inertia inert = Inertia::Identity();
        boost::property_tree::ptree geometry_tree;
        Inertia::Matrix3 I = Eigen::Matrix3d::Identity();
        double mass = 0;
        Eigen::Vector3d lever = Eigen::Vector3d::Zero();

        for(const boost::property_tree::ptree::value_type & v : pt)
        {
          // Current body node get name
          if (v.first == "<xmlattr>")
          {
            if(!v.second.get_optional<std::string>("name"))
                PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Opensim - Missing name for body in osim file");
            
            name = "/" + prefix + "/" + v.second.get<std::string>("name");
          }
          if(v.first == "attached_geometry")
            geometry_tree = v.second; // to add later when vertex is created in the graph

          if(v.first == "mass")
            mass = pt.get<double>("mass");

          if(v.first == "mass_center")
          {
            lever = mjcf::details::internal::getVectorFromStream<3>(pt.get<std::string>("mass_center"));
          }
          if(v.first == "inertia") // [Ixx Iyy Izz Ixy Ixz Iyz] 
          {
            std::istringstream inertiaStream = mjcf::details::internal::getConfiguredStringStream(pt.get<std::string>("inertia"));
            inertiaStream >> I(0, 0);
            inertiaStream >> I(1, 1);
            inertiaStream >> I(2, 2);
            inertiaStream >> I(0, 1);
            inertiaStream >> I(0, 2);
            inertiaStream >> I(1, 2);

            I(1, 0) = I(0, 1);
            I(2, 0) = I(0, 2);
            I(2, 1) = I(1, 2);

          } 
        }
        g.addBody(name, Inertia(mass, lever, I));

        // Now add geometries
        for(const boost::property_tree::ptree::value_type & v : geometry_tree)
        {
            // Info common to all geometries are added 
            auto geom_b = g.geometryBuilder().withBody(name).withPlacement(SE3::Identity()).withGeomType(graph::GeomType::BOTH);
            if(v.first == "Mesh") // only meshes exist in opensim ?
            {
                if(!v.second.get_optional<std::string>("<xmlattr>.name"))
                    PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Opensim - Missing name for an attached_geometry in osim file");
                
                geom_b.withName(v.second.get<std::string>("<xmlattr>.name"));

                // get scale, color, and mesh file
                for(const boost::property_tree::ptree::value_type & val : v.second)
                {
                    if(val.first == "scale_factors")
                        geom_b.withScale(mjcf::details::internal::getVectorFromStream<3>(v.second.get<std::string>("scale_factors")));

                    if(val.first == "mesh_file")
                    {
                        std::cout << path_geom + v.second.get<std::string>("mesh_file") << std::endl;
                        geom_b.withGeom(graph::Mesh(path_geom + v.second.get<std::string>("mesh_file")));
                    }

                    if(val.first == "Appearance")
                    {
                        Eigen::VectorXd color = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("color"));
                        color.conservativeResize(4);
                        color[3] = val.second.get<double>("opacity");
                        geom_b.withColor(color);
                    }
                }
            }
            geom_b.build();
        }
    }

    void parseBodies(const boost::property_tree::ptree & pt, const std::string & prefix,  const std::string & path_geom, graph::ModelGraph & g)
    {
        for(const boost::property_tree::ptree::value_type & v : pt)
        {
            addBody(v.second, prefix, path_geom, g);
        }
    }

    void addJoint(const boost::property_tree::ptree & pt, const std::string & joint_name, const std::string & joint_type, graph::ModelGraph & g)
    {
        auto edge_b = g.edgeBuilder().withName(joint_name);
        std::string parent_frame, child_frame;
        for(const boost::property_tree::ptree::value_type & v : pt)
        {
            if(v.first == "socket_parent_frame")
                parent_frame = pt.get<std::string>("socket_parent_frame");
            
            if(v.first == "socket_child_frame")
                child_frame = pt.get<std::string>("socket_child_frame");

            // Go through the frames to see which bodies it's linked to
            if(v.first == "frames")
            {
                for(const boost::property_tree::ptree::value_type & val : v.second)
                {
                    if(val.first == "PhysicalOffsetFrame" && val.second.get<std::string>("<xmlattr>.name") == parent_frame)
                    {
                        // get source body
                        std::string source_body = val.second.get<std::string>("socket_parent");
                        std::cout << source_body << std::endl;

                        edge_b.withSourceVertex(source_body);

                        // get translation
                        Eigen::Vector3d translation = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("translation"));
                        // get rotation - x y z angles
                        Eigen::Vector3d rot = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("orientation"));
                        Eigen::AngleAxisd R_x(rot[0], Eigen::Vector3d::UnitX());                        
                        Eigen::AngleAxisd R_y(rot[1], Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd R_z(rot[2], Eigen::Vector3d::UnitZ());

                        edge_b.withSourcePose(SE3(R_x.toRotationMatrix()*R_y.toRotationMatrix()*R_z.toRotationMatrix(), translation));

                    }
                    if(val.first == "PhysicalOffsetFrame" && val.second.get<std::string>("<xmlattr>.name") == child_frame)
                    {
                        // get soutargetrce body
                        std::string target_body = val.second.get<std::string>("socket_parent");
                        std::cout << target_body << std::endl;
                        edge_b.withTargetVertex(target_body);

                        // get translation
                        Eigen::Vector3d translation = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("translation"));
                        // get rotation - x y z angles
                        Eigen::Vector3d rot = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("orientation"));
                        Eigen::AngleAxisd R_x(rot[0], Eigen::Vector3d::UnitX());                        
                        Eigen::AngleAxisd R_y(rot[1], Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd R_z(rot[2], Eigen::Vector3d::UnitZ());

                        edge_b.withTargetPose(SE3(R_x.toRotationMatrix()*R_y.toRotationMatrix()*R_z.toRotationMatrix(), translation));
                    }
                }
            }
        }
        // check joint type 
        if(joint_type == "PinJoint")
            edge_b.withJointType(graph::JointRevolute(Eigen::Vector3d::UnitZ()));
        else if(joint_type == "SliderJoint")
            edge_b.withJointType(graph::JointPrismatic(Eigen::Vector3d::UnitX()));

        edge_b.build();
    }

    void addCustomJoint(const boost::property_tree::ptree & pt, const std::string & joint_name, graph::ModelGraph & g)
    {
        auto edge_b = g.edgeBuilder().withName(joint_name);
        std::string parent_frame, child_frame;
        boost::property_tree::ptree joint_tree;
        graph::JointComposite jmodel;
        for(const boost::property_tree::ptree::value_type & v : pt)
        {
            if(v.first == "socket_parent_frame")
                parent_frame = pt.get<std::string>("socket_parent_frame");
            
            if(v.first == "socket_child_frame")
                child_frame = pt.get<std::string>("socket_child_frame");
            if(v.first == "SpatialTransform")
                joint_tree = v.second;
            // Go through the frames to see which bodies it's linked to
            if(v.first == "frames")
            {
                for(const boost::property_tree::ptree::value_type & val : v.second)
                {
                    if(val.first == "PhysicalOffsetFrame" && val.second.get<std::string>("<xmlattr>.name") == parent_frame)
                    {
                        // get source body
                        std::string source_body = val.second.get<std::string>("socket_parent");
                        std::cout << source_body << std::endl;

                        edge_b.withSourceVertex(source_body);

                        // get translation
                        Eigen::Vector3d translation = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("translation"));
                        // get rotation - x y z angles
                        Eigen::Vector3d rot = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("orientation"));
                        Eigen::AngleAxisd R_x(rot[0], Eigen::Vector3d::UnitX());                        
                        Eigen::AngleAxisd R_y(rot[1], Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd R_z(rot[2], Eigen::Vector3d::UnitZ());

                        edge_b.withSourcePose(SE3(R_x.toRotationMatrix()*R_y.toRotationMatrix()*R_z.toRotationMatrix(), translation));

                    }
                    if(val.first == "PhysicalOffsetFrame" && val.second.get<std::string>("<xmlattr>.name") == child_frame)
                    {
                        // get soutargetrce body
                        std::string target_body = val.second.get<std::string>("socket_parent");
                        std::cout << target_body << std::endl;
                        edge_b.withTargetVertex(target_body);

                        // get translation
                        Eigen::Vector3d translation = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("translation"));
                        // get rotation - x y z angles
                        Eigen::Vector3d rot = mjcf::details::internal::getVectorFromStream<3>(val.second.get<std::string>("orientation"));
                        Eigen::AngleAxisd R_x(rot[0], Eigen::Vector3d::UnitX());                        
                        Eigen::AngleAxisd R_y(rot[1], Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd R_z(rot[2], Eigen::Vector3d::UnitZ());

                        edge_b.withTargetPose(SE3(R_x.toRotationMatrix()*R_y.toRotationMatrix()*R_z.toRotationMatrix(), translation));
                    }
                }
            }
        }
        // go through spatial transforms
        for(const boost::property_tree::ptree::value_type & v : joint_tree)
        {
            if(v.first == "TransformAxis")
            {
                if(v.second.get_child_optional("Constant")) // no joint to add if function is constant
                    break;
                 if(v.second.get_child_optional("SimmSpline"))
                    PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Opensim - Joints with SimmSpline are not supported yet");

                if(v.second.get_child_optional("LinearFunction"))
                {
                    Eigen::Vector2d coeffs = mjcf::details::internal::getVectorFromStream<2>(v.second.get<std::string>("LinearFunction.coefficients"));
                    if(coeffs[0] != 1 && coeffs[1] != 0)
                        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Opensim - LinearFunction is not supported with the provided coefficients");
                }
                // get axis
                Eigen::Vector3d axis =  mjcf::details::internal::getVectorFromStream<3>(v.second.get<std::string>("axis"));

                // get if it's a translation or rotation
                if(v.second.get<std::string>("<xmlattr>.name").find("rotation") != std::string::npos)
                    jmodel.addJoint(graph::JointRevolute(axis));
                else
                    jmodel.addJoint(graph::JointPrismatic(axis));
            }
        }
        edge_b.withJointType(jmodel).build();
    }

    void parseJoints(const boost::property_tree::ptree & pt, graph::ModelGraph & g)
    {
        for(const boost::property_tree::ptree::value_type & v : pt)
        {
            if(v.first == "CustomJoint")
                addCustomJoint(v.second, v.second.get<std::string>("<xmlattr>.name"), g);
            else
                addJoint(v.second, v.second.get<std::string>("<xmlattr>.name"),  v.first, g);
        }
    }

    graph::ModelGraph parseModel(const std::string & filename, const std::string & path_geom, const bool verbose)
    {
        graph::ModelGraph graph;
        boost::property_tree::ptree pt, el;
        boost::property_tree::read_xml(filename, pt);

        // Check if it's a opensim model and which version it is
        if(pt.get_child_optional("OpenSimDocument"))
        {
            el = pt.get_child("OpenSimDocument");
            boost::optional<int> version_opt = el.get_optional<int>("<xmlattr>.Version");

            if(!version_opt || version_opt.get() != 40000)
                PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Opensim - Model version is wrong. Only models from Opensim 4 can be parsed");

        }
        else
            PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Opensim - This is not a standard opensim model. Cannot parse it.");

        // Get actual model
        el = el.get_child("Model");
        // add ground if needed
        if(el.get_child_optional("Ground"))
            graph.addBody("/" + el.get<std::string>("Ground.<xmlattr>.name"), Inertia::Identity());

        for(const boost::property_tree::ptree::value_type & v : el)
        {
            // parseBodies
            if(v.first == "BodySet")
                parseBodies(el.get_child("BodySet.objects"), v.second.get<std::string>("<xmlattr>.name"), path_geom, graph);
            // parse joints
            if(v.first == "JointSet")
                parseJoints(el.get_child("JointSet.objects"), graph);

        } 
        return graph;  
    }
    void buildModel(const std::string & filename, Model & model, const bool verbose)
    {
        // graph::ModelGraph g = parseModel(filename, verbose);

        // return graph::buildModel(g, "/ground", SE3::Identity());

    }
  } // namespace opensim
} // namespace pinocchio