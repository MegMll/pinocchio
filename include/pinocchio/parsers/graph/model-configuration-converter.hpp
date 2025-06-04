//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
#define __pinocchio_parsers_graph_model_configuration_converter_hpp__

#include "pinocchio/parsers/graph/model-graph.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"

#include <boost/variant.hpp>

#include <unordered_map>

namespace pinocchio
{

  namespace graph
  {

    /// Convert configuration or tangent vector from two model with different root.
    template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
    struct ModelConfigurationConverterTpl
    {
      typedef _Scalar Scalar;
      enum
      {
        Options = _Options
      };
      typedef JointCollectionTpl<Scalar, Options> JointCollection;
      typedef typename JointCollection::JointModelVariant JointModelVariant;

      struct ConfigurationMapping
      {
        int idx_qs_source;
        int idx_qs_target;
        int nq;
      };

      struct TangentMapping
      {
        int idx_vs_source;
        int idx_vs_target;
        int nv;
      };

      struct JointMapping
      {
        JointMapping() = default;
        JointMapping(JointModelVariant joint, bool same_direction)
        : joint(joint)
        , direction_sign(same_direction ? Scalar(1.) : Scalar(-1.))
        , same_direction(same_direction)
        {
        }

        JointModelVariant joint;
        Scalar direction_sign;
        bool same_direction;
      };

      ModelConfigurationConverterTpl() = default;
      ModelConfigurationConverterTpl(
        std::vector<ConfigurationMapping> configuration_mapping,
        std::vector<TangentMapping> tangent_mapping,
        std::vector<JointMapping> joint_mapping,
        int source_configuration_size,
        int source_tangent_size,
        int target_configuration_size,
        int target_tangent_size)
      : configuration_mapping(configuration_mapping)
      , tangent_mapping(tangent_mapping)
      , joint_mapping(joint_mapping)
      , source_configuration_size(source_configuration_size)
      , source_tangent_size(source_tangent_size)
      , target_configuration_size(target_configuration_size)
      , target_tangent_size(target_tangent_size)
      {
      }

      /// Convert \p q_source configuration vector from source model to \p q_target configuration
      /// vector from target model.
      template<typename ConfigVectorType1, typename ConfigVectorType2>
      void convertConfiguration(
        const Eigen::MatrixBase<ConfigVectorType1> & q_source,
        const Eigen::MatrixBase<ConfigVectorType2> & q_target) const;

      std::vector<ConfigurationMapping> configuration_mapping;
      std::vector<TangentMapping> tangent_mapping;
      std::vector<JointMapping> joint_mapping;
      int source_configuration_size;
      int source_tangent_size;
      int target_configuration_size;
      int target_tangent_size;
    };

    namespace internal
    {
      /// Record joint direction when walking in a graph
      struct RecordJointDirectionVisitor : public boost::default_dfs_visitor
      {
        typedef std::unordered_map<std::string, bool> JointNameToDirection;

        RecordJointDirectionVisitor(JointNameToDirection * joint_forward)
        : joint_forward(joint_forward)
        {
        }

        void tree_edge(ModelGraph::EdgeDesc edge_desc, const ModelGraph::Graph & g) const
        {
          const ModelGraphEdge & edge = g[edge_desc];
          (*joint_forward)[edge.name] = !edge.reverse;
        }

        /// Joint name to a bool that hold true if the joint is in forward direction
        JointNameToDirection * joint_forward;
      };

      template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
      FrameIndex findRootBodyFrame(const ModelTpl<Scalar, Options, JointCollectionTpl> & model)
      {
        for (std::size_t i = 0; i < model.frames.size(); ++i)
        {
          if (model.frames[i].type == FrameType::BODY)
          {
            return i;
          }
        }
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "findRootBodyFrame - No BODY frame");
      }
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl> createConverter(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
      const ModelGraph & graph)
    {
      typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
        ModelConfigurationConverter;
      typedef typename ModelConfigurationConverter::ConfigurationMapping ConfigurationMapping;
      typedef typename ModelConfigurationConverter::TangentMapping TangentMapping;
      typedef typename ModelConfigurationConverter::JointMapping JointMapping;

      std::vector<ConfigurationMapping> configuration_mapping;
      std::vector<TangentMapping> tangent_mapping;
      std::vector<JointMapping> joint_mapping;

      auto root_frame_index_source = internal::findRootBodyFrame(model_source);
      auto root_frame_index_target = internal::findRootBodyFrame(model_target);

      // Retrieve root frame
      const auto & root_frame_source = model_source.frames[root_frame_index_source];
      const auto & root_frame_target = model_target.frames[root_frame_index_target];

      // Is root joint fixed ?
      bool is_fixed_base_source = root_frame_source.parentJoint == 0;

      // Retrieve Model graph root vertex
      const auto & root_vertex_source = graph.name_to_vertex.find(root_frame_source.name);
      const auto & root_vertex_target = graph.name_to_vertex.find(root_frame_target.name);

      // Store joint direction for each root vertex
      std::vector<boost::default_color_type> colors(
        boost::num_vertices(graph.g), boost::default_color_type::white_color);
      internal::RecordJointDirectionVisitor::JointNameToDirection joint_direction_source;
      internal::RecordJointDirectionVisitor::JointNameToDirection joint_direction_target;
      boost::depth_first_search(
        graph.g, internal::RecordJointDirectionVisitor(&joint_direction_source), colors.data(),
        root_vertex_source->second);
      colors.assign(colors.size(), boost::default_color_type::white_color);
      boost::depth_first_search(
        graph.g, internal::RecordJointDirectionVisitor(&joint_direction_target), colors.data(),
        root_vertex_target->second);

      // Construct the mapping between source and target configuration and tangent vector.
      // If source model doesn't have a fixed base, we skip the first joint (usually a FF joint)
      // that can not be in the target model.
      std::size_t index_source = 1;
      if (!is_fixed_base_source)
      {
        index_source = 2;
      }
      for (; index_source < model_source.joints.size(); ++index_source)
      {
        ConfigurationMapping configuration;
        TangentMapping tangent;
        const std::string & joint_name = model_source.names[index_source];
        auto index_target = model_target.getJointId(joint_name);

        configuration.idx_qs_source = model_source.idx_qs[index_source];
        configuration.idx_qs_target = model_target.idx_qs[index_target];
        configuration.nq = model_source.nqs[index_source];

        tangent.idx_vs_source = model_source.idx_vs[index_source];
        tangent.idx_vs_target = model_target.idx_vs[index_target];
        tangent.nv = model_source.nvs[index_source];

        configuration_mapping.push_back(configuration);
        tangent_mapping.push_back(tangent);
        joint_mapping.emplace_back(
          model_source.joints[index_source],
          joint_direction_source.at(joint_name) == joint_direction_target.at(joint_name));
      }

      return ModelConfigurationConverter(
        configuration_mapping, tangent_mapping, joint_mapping, model_source.nq, model_source.nv,
        model_target.nq, model_target.nv);
    }

    // TODO: put in a .hxx
    // TODO: missing joints
    // TODO: tangent space
    // TODO: TU
    namespace internal
    {
      template<
        typename _Scalar,
        int _Options,
        template<typename, int> class JointCollectionTpl,
        typename ConfigVectorType1,
        typename ConfigVectorType2>
      struct ConfigurationConverterVisitor : public boost::static_visitor<void>
      {
        typedef _Scalar Scalar;
        enum
        {
          Options = _Options
        };

        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::ConfigurationMapping ConfigurationMapping;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;

        typedef void ReturnType;

        const Eigen::MatrixBase<ConfigVectorType1> & q_source;
        ConfigVectorType2 & q_target;
        const ConfigurationMapping & configuration;
        const JointMapping & joint;

        ConfigurationConverterVisitor(
          const Eigen::MatrixBase<ConfigVectorType1> & q_source,
          const Eigen::MatrixBase<ConfigVectorType2> & q_target,
          const ConfigurationMapping & configuration,
          const JointMapping & joint)
        : q_source(q_source)
        , q_target(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType2, q_target))
        , configuration(configuration)
        , joint(joint)
        {
        }

        /// Manage Revolute, Prismatic, Translation and Helical joint
        template<typename JointType>
        ReturnType operator()(const JointType &) const
        {
          q_target.segment(configuration.idx_qs_target, configuration.nq) =
            joint.direction_sign * q_source.segment(configuration.idx_qs_source, configuration.nq);
        }

        template<int axis>
        ReturnType operator()(const JointModelRevoluteUnboundedTpl<Scalar, Options, axis> &) const
        {
          // TODO
        }

        ReturnType
        operator()(const JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> &) const
        {
          // TODO
        }

        ReturnType operator()(const JointModelFreeFlyerTpl<Scalar, Options> &) const
        {
          // Copy tx, ty, tz, qx, qy, qz
          q_target.segment(configuration.idx_qs_target, 6) =
            joint.direction_sign * q_source.segment(configuration.idx_qs_source, 6);
          // Copy qw
          q_target[configuration.idx_qs_target + 6] = q_source[configuration.idx_qs_source + 6];
        }

        ReturnType operator()(const JointModelSphericalTpl<Scalar, Options> &) const
        {
          // Copy qx, qy, qz
          q_target.segment(configuration.idx_qs_target, 3) =
            joint.direction_sign * q_source.segment(configuration.idx_qs_source, 3);
          // Copy qw
          q_target[configuration.idx_qs_target + 3] = q_source[configuration.idx_qs_source + 3];
        }

        ReturnType operator()(const JointModelSphericalZYXTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
          }
          else
          {
          }
        }

        ReturnType operator()(const JointModelPlanarTpl<Scalar, Options> &) const
        {
          // TODO
        }

        ReturnType operator()(const JointModelUniversalTpl<Scalar, Options> &) const
        {
          q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source + 1];
          q_target[configuration.idx_qs_target + 1] = q_source[configuration.idx_qs_source];
        }

        ReturnType operator()(const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // TODO
        }

        ReturnType
        operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // TODO must know what kind of joint and add offset to visitor
        }
      };
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    template<typename ConfigVectorType1, typename ConfigVectorType2>
    void ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>::convertConfiguration(
      const Eigen::MatrixBase<ConfigVectorType1> & q_source,
      const Eigen::MatrixBase<ConfigVectorType2> & q_target) const
    {
      if (source_configuration_size != q_source.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "convertConfiguration - wrong source configuration size");
      }
      if (target_configuration_size != q_target.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "convertConfiguration - wrong target configuration size");
      }

      for (std::size_t i = 0; i < joint_mapping.size(); ++i)
      {
        const auto & configuration = configuration_mapping[i];
        const auto & joint = joint_mapping[i];
        boost::apply_visitor(
          internal::ConfigurationConverterVisitor<
            Scalar, Options, JointCollectionTpl, ConfigVectorType1, ConfigVectorType2>(
            q_source, q_target, configuration, joint),
          joint.joint);
      }
    }

  } // namespace graph

} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
