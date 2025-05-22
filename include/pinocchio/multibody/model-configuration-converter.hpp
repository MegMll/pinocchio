//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_multibody_model_configuration_converter_hpp__
#define __pinocchio_multibody_model_configuration_converter_hpp__

#include "pinocchio/multibody/model-graph.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"
#include <boost/variant/detail/apply_visitor_unary.hpp>

namespace pinocchio
{

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
      JointModelVariant joint;
      Scalar direction_sign;
      bool forward;
    };

    ModelConfigurationConverterTpl() = default;
    ModelConfigurationConverterTpl(
      std::vector<ConfigurationMapping> configuration_mapping,
      std::vector<TangentMapping> tangent_mapping,
      std::vector<JointMapping> joint_mapping)
    : configuration_mapping(configuration_mapping)
    , tangent_mapping(tangent_mapping)
    , joint_mapping(joint_mapping)
    {
    }

    template<typename ConfigVectorType>
    void convertConfiguration(
      const Eigen::MatrixBase<ConfigVectorType> & q_source,
      const Eigen::MatrixBase<ConfigVectorType> & q_target) const;

    std::vector<ConfigurationMapping> configuration_mapping;
    std::vector<TangentMapping> tangent_mapping;
    std::vector<JointMapping> joint_mapping;
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl> createConverter(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
    const ModelGraph & graph)
  {
  }

  // TODO: put in a .hxx
  // TODO: construct mapping
  // TODO: missing joints
  // TODO: tangent space
  // TODO: TU
  template<
    typename _Scalar,
    int _Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
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

    const Eigen::MatrixBase<ConfigVectorType> & q_source;
    const Eigen::MatrixBase<ConfigVectorType> & q_target;
    const ConfigurationMapping & configuration;
    const JointMapping & joint;

    ConfigurationConverterVisitor(
      const Eigen::MatrixBase<ConfigVectorType> & q_source,
      const Eigen::MatrixBase<ConfigVectorType> & q_target,
      const ConfigurationMapping & configuration,
      const JointMapping & joint)
    : q_source(q_source)
    , q_target(q_target)
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

    ReturnType operator()(const JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> &) const
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
      if (joint.forward)
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

    ReturnType operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
    {
      // TODO must know what kind of joint and add offset to visitor
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  template<typename ConfigVectorType>
  void ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>::convertConfiguration(
    const Eigen::MatrixBase<ConfigVectorType> & q_source,
    const Eigen::MatrixBase<ConfigVectorType> & q_target) const
  {
    for (std::size_t i = 0; i < joint_mapping.size(); ++i)
    {
      const auto & configuration = configuration_mapping[i];
      const auto & joint = joint_mapping[i];
      boost::apply_visitor(
        ConfigurationConverterVisitor<Scalar, Options, JointCollectionTpl, ConfigVectorType>(
          q_source, q_target, configuration, joint),
        joint.joint);
    }
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_configuration_model_converter_hpp__
