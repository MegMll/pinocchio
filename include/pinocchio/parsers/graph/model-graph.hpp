//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_model_graph_hpp__
#define __pinocchio_parsers_model_graph_hpp__

#include "pinocchio/parsers/config.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"

#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/frames.hpp"

#include <Eigen/Core>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

namespace pinocchio
{
  namespace graph
  {
    struct EdgeBuilder;
    struct EdgeParameters;

    /// @brief Represents a vertex (body, sensor, operational frame) in the model graph.
    struct ModelGraphVertex
    {
      /// @brief Unique name of the body.
      std::string name;

      FrameGraphVariant frame;
    };

    /// @brief Represents an edge (joint) in the model graph.
    struct ModelGraphEdge
    {
      /// @brief Unique name of the joint
      std::string name;

      /// @brief What is the type of the joint
      JointGraphVariant joint;

      /// @brief All the limits of the joint
      JointLimits jlimit;

      /// @brief Transformation from the previous vertex to edge
      ///
      /// Correspond to the transformation from body supporting joint to said joint
      SE3 out_to_joint;

      /// @brief Transformation from edge to next vertex
      ///
      /// Correspond to the transformation from the current joint to its supported body.
      SE3 joint_to_in;

      /// @brief boolean to know if we are in a forward or backward edge
      bool forward = true;
    };

    /// @brief Contains information about how \ref buildModel walked the \ref ModelGraph to
    /// construct a \ref Model.
    /// All members are considered internal.
    struct ModelGraphBuildInfo
    {
      /// Map joint name to joint direction.
      std::unordered_map<std::string, bool> _joint_forward;
      /// True if the root joint is fixed.
      bool _is_fixed;
    };

    /// @brief Represents multibody model as a bidirectional graph.
    ///
    /// This is an intermediate step before creating a model, that
    /// allows more flexibility as to which body will be the root...
    struct PINOCCHIO_PARSERS_DLLAPI ModelGraph
    {
      typedef boost::
        adjacency_list<boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
          Graph;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
      typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

      ModelGraph() = default;
      /// \brief Add a new vertex to the graph
      ///
      /// \param[in] vertex_name Name of the vertex
      /// \param[in] frame which type of frame will be added to the model (op_frame, sensor, body)
      void addFrame(const std::string & vertex_name, const FrameGraphVariant & frame);

      /// \brief Add a new body to the graph
      ///
      /// \param[in] vertex_name Name of the vertex
      /// \param[in] inert inertia of the body
      void addBody(const std::string & vertex_name, const Inertia & inert);

      /// \brief Add edges (joint) to the graph. Since it's a bidirectional graph,
      /// edge and its reverse are added to the graph.
      ///
      /// \param[in] joint_name Name of the edge
      /// \param[in] joint Type of the joint
      /// \param[in] out_body Vertex that is supporting the edge
      /// \param[in] out_to_joint Transformation from supporting vertex to edge
      /// \param[in] in_body Vertex that is supported by edge
      /// \param[in] joint_to_in Transformation from edge to supported vertex
      /// \param[in] q_ref q offset of the joint
      ///
      /// \note Since it's a bidirectional graph, two edges are added to the graph.
      /// Joints and transformation are inverted, to create reverse edge.
      void addJoint(
        const std::string & joint_name,
        const JointGraphVariant & joint,
        const std::string & out_body,
        const SE3 & out_to_joint,
        const std::string & in_body,
        const SE3 & joint_to_in,
        const boost::optional<Eigen::VectorXd> & q_ref = boost::none);

      void addJoint(const EdgeParameters & params);

      EdgeBuilder useEdgeBuilder();

      /// @brief  add all the vertex and edges from a graph to this one.
      /// Attention : it does not add an edge between the two, so it will be like having two graph
      /// coexisting in this structure.
      ///
      /// @param g graph that will be added
      ///
      void appendGraph(const ModelGraph & g);

      /// @brief Boost graph structure that holds the graph structure
      Graph graph;
      /// @brief Name of the vertexes in the graph. Useful for graph parcours.
      std::unordered_map<std::string, VertexDesc> name_to_vertex;
    };

    struct EdgeParameters
    {
      std::string name;

      std::string source_vertex;
      SE3 out_to_joint = SE3::Identity();
      std::string target_vertex;
      SE3 joint_to_in = SE3::Identity();

      JointGraphVariant joint = JointFixedGraph();

      boost::optional<Eigen::VectorXd> q_ref = boost::none;
      JointLimits jlimit;

      EdgeParameters() = default;

      EdgeParameters(
        const std::string & jname,
        const std::string & source_name,
        const SE3 & out_to_joint,
        const std::string & target_name,
        const SE3 & joint_to_in,
        const JointGraphVariant & joint,
        const boost::optional<Eigen::VectorXd> q_ref = boost::none)
      : name(jname)
      , source_vertex(source_name)
      , out_to_joint(out_to_joint)
      , target_vertex(target_name)
      , joint_to_in(joint_to_in)
      , joint(joint)
      , q_ref(q_ref)
      {
      }
    };

    struct PINOCCHIO_PARSERS_DLLAPI EdgeBuilder
    {
      ModelGraph & g;

      EdgeParameters param;

      EdgeBuilder(ModelGraph & graph)
      : g(graph)
      {
      }

      EdgeBuilder & withJointType(const JointGraphVariant & jtype);

      EdgeBuilder & withName(const std::string & name)
      {
        param.name = name;
        return *this;
      }

      EdgeBuilder & withTargetVertex(const std::string & target_name)
      {
        param.target_vertex = target_name;
        return *this;
      }

      EdgeBuilder & withSourceVertex(const std::string & source_name)
      {
        param.source_vertex = source_name;
        return *this;
      }

      EdgeBuilder & withTargetPose(const SE3 & target_pose)
      {
        param.joint_to_in = target_pose;
        return *this;
      }

      EdgeBuilder & withSourcePose(const SE3 & source_pose)
      {
        param.out_to_joint = source_pose;
        return *this;
      }

      EdgeBuilder & withQref(const Eigen::VectorXd & qref)
      {
        param.q_ref = qref;
        return *this;
      }

      EdgeBuilder & withMinConfig(const Eigen::VectorXd & minConfig)
      {
        param.jlimit.minConfig = minConfig;
        return *this;
      }

      EdgeBuilder & withMaxConfig(const Eigen::VectorXd & maxConfig)
      {
        param.jlimit.maxConfig = maxConfig;
        return *this;
      }

      EdgeBuilder & withMaxVel(const Eigen::VectorXd & maxVel)
      {
        param.jlimit.maxVel = maxVel;
        return *this;
      }

      EdgeBuilder & withMaxEffort(const Eigen::VectorXd & maxEffort)
      {
        param.jlimit.maxEffort = maxEffort;
        return *this;
      }

      EdgeBuilder & withFriction(const Eigen::VectorXd & friction)
      {
        param.jlimit.friction = friction;
        return *this;
      }

      EdgeBuilder & withDamping(const Eigen::VectorXd & damping)
      {
        param.jlimit.damping = damping;
        return *this;
      }

      EdgeBuilder & withArmature(const Eigen::VectorXd & armature)
      {
        param.jlimit.armature = armature;
        return *this;
      }

      EdgeBuilder & withFrictionLoss(const double frictionLoss)
      {
        param.jlimit.frictionLoss = frictionLoss;
        return *this;
      }

      void build()
      {
        g.addJoint(param);
      }
    };

  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
