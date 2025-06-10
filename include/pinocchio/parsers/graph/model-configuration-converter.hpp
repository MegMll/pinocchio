//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
#define __pinocchio_parsers_graph_model_configuration_converter_hpp__

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include <Eigen/Core>

#include <vector>

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
      void convertConfigurationVector(
        const Eigen::MatrixBase<ConfigVectorType1> & q_source,
        const Eigen::MatrixBase<ConfigVectorType2> & q_target) const;

      /// Convert \p v_source tangent vector from source model to \p v_target tangent
      /// vector from target model.
      template<typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
      void convertTangentVector(
        const Eigen::MatrixBase<ConfigVectorType> & q_source,
        const Eigen::MatrixBase<TangentVectorType1> & v_source,
        const Eigen::MatrixBase<TangentVectorType2> & v_target) const;

      /// Contains configuration vector mapping between source and target model.
      /// This vector contains all flattened model joints (with composite joint contents).
      std::vector<ConfigurationMapping> configuration_mapping;
      /// Contains tangent vector mapping between source and target model.
      /// This vector contains all flattened model joints (with composite joint contents).
      std::vector<TangentMapping> tangent_mapping;
      /// Contains joint mapping between source and target model.
      /// This vector contains all flattened model joints (with composite joint contents).
      std::vector<JointMapping> joint_mapping;
      int source_configuration_size;
      int source_tangent_size;
      int target_configuration_size;
      int target_tangent_size;
    };

    /// Create a ModelConfigurationConverterTpl from \p model_source to \p model_target that
    /// have been created by the common \p graph.
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl> createConverter(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
      const ModelGraph & graph);

  } // namespace graph
} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/parsers/graph/model-configuration-converter.hxx"

#endif // ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
