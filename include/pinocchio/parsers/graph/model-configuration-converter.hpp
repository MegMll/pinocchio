//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
#define __pinocchio_parsers_graph_model_configuration_converter_hpp__

#include "pinocchio/parsers/graph/model-graph.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"

#include <Eigen/Geometry>

#include <boost/variant.hpp>

#include <stdexcept>
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

      /// Convert \p v_source tangent vector from source model to \p v_target tangent
      /// vector from target model.
      template<typename TangentVectorType1, typename TangentVectorType2>
      void convertTangent(
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
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "findRootBodyFrame - No BODY frame");
      }

      /// Compute the ModelConfigurationConverter mapping vector.
      /// This structure use recursive methods to handle composite joint flattening.
      template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
      struct CreateConverterAlgo
      {
        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::ConfigurationMapping ConfigurationMapping;
        typedef typename ModelConfigurationConverter::TangentMapping TangentMapping;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;
        typedef JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> JointModelComposite;
        typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;

        /// Add all the joints from a Model to the mapping.
        void addJointFromModel(
          const ModelTpl<Scalar, Options, JointCollectionTpl> & model_source,
          const ModelTpl<Scalar, Options, JointCollectionTpl> & model_target,
          const internal::RecordJointDirectionVisitor::JointNameToDirection &
            joint_direction_source,
          const internal::RecordJointDirectionVisitor::JointNameToDirection &
            joint_direction_target,
          std::size_t index_source)
        {
          for (; index_source < model_source.joints.size(); ++index_source)
          {
            const auto & joint_model_source = model_source.joints[index_source];
            const std::string & joint_name = model_source.names[index_source];
            auto index_target = model_target.getJointId(joint_name);
            bool same_direction =
              joint_direction_source.at(joint_name) == joint_direction_target.at(joint_name);
            const JointModelComposite * joint_composite_source =
              boost::get<JointModelComposite>(&joint_model_source);
            if (joint_composite_source == nullptr)
            {
              ConfigurationMapping configuration;
              TangentMapping tangent;

              configuration.idx_qs_source = model_source.idx_qs[index_source];
              configuration.idx_qs_target = model_target.idx_qs[index_target];
              configuration.nq = model_source.nqs[index_source];

              tangent.idx_vs_source = model_source.idx_vs[index_source];
              tangent.idx_vs_target = model_target.idx_vs[index_target];
              tangent.nv = model_source.nvs[index_source];

              configuration_mapping.push_back(configuration);
              tangent_mapping.push_back(tangent);
              joint_mapping.emplace_back(joint_model_source, same_direction);
            }
            else
            {
              const auto & joint_model_target = model_target.joints[index_target];
              const JointModelComposite & joint_composite_target =
                boost::get<JointModelComposite>(joint_model_target);
              addJointFromComposite(
                joint_composite_source->joints, joint_composite_target.joints, same_direction);
            }
          }
        }

        /// Add all the joints from a JointComposite to the mapping.
        void addJointFromComposite(
          const typename JointModelComposite::JointModelVector & joints_source,
          const typename JointModelComposite::JointModelVector & joints_target,
          bool same_direction)
        {
          if (same_direction)
          {
            for (std::size_t index_source = 0; index_source < joints_source.size(); ++index_source)
            {
              const auto & joint_model_source = joints_source[index_source];
              const auto & joint_model_target = joints_target[index_source];
              addJointModel(joint_model_source, joint_model_target, same_direction);
            }
          }
          else
          {
            for (std::size_t index_source = 0, index_target = joints_target.size() - 1;
                 index_source < joints_source.size(); ++index_source, --index_target)
            {
              const auto & joint_model_source = joints_source[index_source];
              const auto & joint_model_target = joints_target[index_target];
              addJointModel(joint_model_source, joint_model_target, same_direction);
            }
          }
        }

        /// Add a JointModel to the mapping.
        void addJointModel(
          const JointModel & joint_model_source,
          const JointModel & joint_model_target,
          bool same_direction)
        {
          const JointModelComposite * joint_composite_source =
            boost::get<JointModelComposite>(&joint_model_source);
          if (joint_composite_source == nullptr)
          {
            ConfigurationMapping configuration;
            TangentMapping tangent;
            configuration.idx_qs_source = joint_model_source.idx_q();
            configuration.idx_qs_target = joint_model_target.idx_q();
            configuration.nq = joint_model_source.nq();

            tangent.idx_vs_source = joint_model_source.idx_v();
            tangent.idx_vs_target = joint_model_target.idx_v();
            tangent.nv = joint_model_source.nv();

            configuration_mapping.push_back(configuration);
            tangent_mapping.push_back(tangent);
            joint_mapping.emplace_back(joint_model_source, same_direction);
          }
          else
          {
            const JointModelComposite & joint_composite_target =
              boost::get<JointModelComposite>(joint_model_target);
            addJointFromComposite(
              joint_composite_source->joints, joint_composite_target.joints, same_direction);
          }
        }

        std::vector<ConfigurationMapping> configuration_mapping;
        std::vector<TangentMapping> tangent_mapping;
        std::vector<JointMapping> joint_mapping;
      };
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
      internal::CreateConverterAlgo<Scalar, Options, JointCollectionTpl> algo;
      algo.addJointFromModel(
        model_source, model_target, joint_direction_source, joint_direction_target, index_source);

      return ModelConfigurationConverter(
        algo.configuration_mapping, algo.tangent_mapping, algo.joint_mapping, model_source.nq,
        model_source.nv, model_target.nq, model_target.nv);
    }

    // TODO: put in a .hxx
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

        typedef Eigen::Vector<Scalar, 2> Vector2;
        typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
        typedef Eigen::Vector<Scalar, 3> Vector3;
        typedef Eigen::Quaternion<Scalar> Quaternion;

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

        // Manage Revolute, Prismatic, Translation and Helical joint.
        template<typename JointType>
        ReturnType operator()(const JointType &) const
        {
          // Apply direction_sign on each configuration values.
          q_target.segment(configuration.idx_qs_target, configuration.nq) =
            joint.direction_sign * q_source.segment(configuration.idx_qs_source, configuration.nq);
        }

        template<int axis>
        ReturnType operator()(const JointModelRevoluteUnboundedTpl<Scalar, Options, axis> &) const
        {
          // Apply direction_sign on sinus to inverse the rotation
          q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source];
          q_target[configuration.idx_qs_target + 1] =
            joint.direction_sign * q_source[configuration.idx_qs_source + 1];
        }

        ReturnType
        operator()(const JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> &) const
        {
          // Apply direction_sign on sinus to inverse the rotation
          q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source];
          q_target[configuration.idx_qs_target + 1] =
            joint.direction_sign * q_source[configuration.idx_qs_source + 1];
        }

        ReturnType operator()(const JointModelFreeFlyerTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy tx, ty, tz, qx, qy, qz, qw
            q_target.template segment<7>(configuration.idx_qs_target) =
              q_source.template segment<7>(configuration.idx_qs_source);
          }
          else
          {
            // Apply inverse rotation on translation and copy inverse rotation
            Vector3 translation_source(q_source.template segment<3>(configuration.idx_qs_source));
            Quaternion rotation_source(
              q_source.template segment<4>(configuration.idx_qs_source + 3));
            Quaternion rotation_source_inv(rotation_source.inverse());

            Vector3 translation_target(-(rotation_source_inv * translation_source));
            q_target.template segment<3>(configuration.idx_qs_target) = translation_target;
            q_target.template segment<4>(configuration.idx_qs_target + 3) =
              rotation_source_inv.coeffs();
          }
        }

        ReturnType operator()(const JointModelSphericalTpl<Scalar, Options> &) const
        {
          // Copy qx, qy, qz with direction_sign apply to it
          q_target.template segment<3>(configuration.idx_qs_target) =
            joint.direction_sign * q_source.template segment<3>(configuration.idx_qs_source);
          // Copy qw
          q_target[configuration.idx_qs_target + 3] = q_source[configuration.idx_qs_source + 3];
        }

        ReturnType operator()(const JointModelSphericalZYXTpl<Scalar, Options> &) const
        {

          if (joint.same_direction)
          {
            // Copy zyx
            q_target.template segment<3>(configuration.idx_qs_target) =
              q_source.template segment<3>(configuration.idx_qs_source);
          }
          else
          {
            // Compute the inverse rotation and exctract the ZYX euler angles
            JointModelSphericalZYXTpl<Scalar, Options> jmodel;
            jmodel.setIndexes(0, 0, 0);
            JointDataSphericalZYXTpl<Scalar, Options> jdata;
            jmodel.calc(jdata, q_source.template segment<3>(configuration.idx_qs_source));
            q_target.template segment<3>(configuration.idx_qs_target) =
              jdata.M.rotation().transpose().eulerAngles(2, 1, 0);
          }
        }

        ReturnType operator()(const JointModelPlanarTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy x, y, cos_theta, sin_theta
            q_target.template segment<4>(configuration.idx_qs_target) =
              q_source.template segment<4>(configuration.idx_qs_source);
          }
          else
          {
            Scalar c_theta_source = q_source[configuration.idx_qs_source + 2];
            Scalar s_theta_source = q_source[configuration.idx_qs_source + 3];
            Matrix2 rotation_source_inv;
            rotation_source_inv << c_theta_source, s_theta_source, -s_theta_source, c_theta_source;
            q_target.template segment<2>(configuration.idx_qs_target) =
              -rotation_source_inv * q_source.template segment<2>(configuration.idx_qs_source);
            q_target[configuration.idx_qs_target + 2] = c_theta_source;
            q_target[configuration.idx_qs_target + 3] = -s_theta_source;
          }
        }

        ReturnType operator()(const JointModelUniversalTpl<Scalar, Options> &) const
        {
          if (joint.same_direction)
          {
            // Copy q
            q_target.template segment<2>(configuration.idx_qs_target) =
              q_source.template segment<2>(configuration.idx_qs_source);
          }
          else
          {
            // Axes are inversed so swap q
            q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source + 1];
            q_target[configuration.idx_qs_target + 1] = q_source[configuration.idx_qs_source];
          }
        }

        ReturnType operator()(const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // Nothing to do, q conversion is managed in mimicked joint.
        }

        ReturnType
        operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          assert(false && "This must never happened");
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
          std::invalid_argument, "convertConfiguration - wrong source configuration size");
      }
      if (target_configuration_size != q_target.size())
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "convertConfiguration - wrong target configuration size");
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

    namespace internal
    {
      template<
        typename _Scalar,
        int _Options,
        template<typename, int> class JointCollectionTpl,
        typename TangentVectorType1,
        typename TangentVectorType2>
      struct TangentConverterVisitor : public boost::static_visitor<void>
      {
        typedef _Scalar Scalar;
        enum
        {
          Options = _Options
        };

        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;
        typedef typename ModelConfigurationConverter::TangentMapping TangentMapping;
        typedef typename ModelConfigurationConverter::JointMapping JointMapping;

        typedef Eigen::Vector<Scalar, 2> Vector2;
        typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
        typedef Eigen::Vector<Scalar, 3> Vector3;
        typedef Eigen::Quaternion<Scalar> Quaternion;

        typedef void ReturnType;

        const Eigen::MatrixBase<TangentVectorType1> & v_source;
        TangentVectorType2 & v_target;
        const TangentMapping & tangent;
        const JointMapping & joint;

        TangentConverterVisitor(
          const Eigen::MatrixBase<TangentVectorType1> & v_source,
          const Eigen::MatrixBase<TangentVectorType2> & v_target,
          const TangentMapping & tangent,
          const JointMapping & joint)
        : v_source(v_source)
        , v_target(PINOCCHIO_EIGEN_CONST_CAST(TangentVectorType2, v_target))
        , tangent(tangent)
        , joint(joint)
        {
        }

        // Manage Revolute, RevoluteUnbounded, Prismatic, Translation and Helical joint.
        template<typename JointType>
        ReturnType operator()(const JointType &) const
        {
          // Apply direction_sign on each configuration values.
          v_target.segment(tangent.idx_vs_target, tangent.nv) =
            joint.direction_sign * v_source.segment(tangent.idx_vs_source, tangent.nv);
        }

        // ReturnType operator()(const JointModelFreeFlyerTpl<Scalar, Options> &) const
        // {
        //   if (joint.same_direction)
        //   {
        //     // Copy tx, ty, tz, qx, qy, qz, qw
        //     q_target.template segment<7>(configuration.idx_qs_target) =
        //       q_source.template segment<7>(configuration.idx_qs_source);
        //   }
        //   else
        //   {
        //     // Apply inverse rotation on translation and copy inverse rotation
        //     Vector3 translation_source(q_source.template
        //     segment<3>(configuration.idx_qs_source)); Quaternion rotation_source(
        //       q_source.template segment<4>(configuration.idx_qs_source + 3));
        //     Quaternion rotation_source_inv(rotation_source.inverse());
        //
        //     Vector3 translation_target(-(rotation_source_inv * translation_source));
        //     q_target.template segment<3>(configuration.idx_qs_target) = translation_target;
        //     q_target.template segment<4>(configuration.idx_qs_target + 3) =
        //       rotation_source_inv.coeffs();
        //   }
        // }
        //
        // ReturnType operator()(const JointModelSphericalTpl<Scalar, Options> &) const
        // {
        //   // Copy qx, qy, qz with direction_sign apply to it
        //   q_target.template segment<3>(configuration.idx_qs_target) =
        //     joint.direction_sign * q_source.template segment<3>(configuration.idx_qs_source);
        //   // Copy qw
        //   q_target[configuration.idx_qs_target + 3] = q_source[configuration.idx_qs_source + 3];
        // }
        //
        // ReturnType operator()(const JointModelSphericalZYXTpl<Scalar, Options> &) const
        // {
        //
        //   if (joint.same_direction)
        //   {
        //     // Copy zyx
        //     q_target.template segment<3>(configuration.idx_qs_target) =
        //       q_source.template segment<3>(configuration.idx_qs_source);
        //   }
        //   else
        //   {
        //     // Compute the inverse rotation and exctract the ZYX euler angles
        //     JointModelSphericalZYXTpl<Scalar, Options> jmodel;
        //     jmodel.setIndexes(0, 0, 0);
        //     JointDataSphericalZYXTpl<Scalar, Options> jdata;
        //     jmodel.calc(jdata, q_source.template segment<3>(configuration.idx_qs_source));
        //     q_target.template segment<3>(configuration.idx_qs_target) =
        //       jdata.M.rotation().transpose().eulerAngles(2, 1, 0);
        //   }
        // }
        //
        // ReturnType operator()(const JointModelPlanarTpl<Scalar, Options> &) const
        // {
        //   if (joint.same_direction)
        //   {
        //     // Copy x, y, cos_theta, sin_theta
        //     q_target.template segment<4>(configuration.idx_qs_target) =
        //       q_source.template segment<4>(configuration.idx_qs_source);
        //   }
        //   else
        //   {
        //     Scalar c_theta_source = q_source[configuration.idx_qs_source + 2];
        //     Scalar s_theta_source = q_source[configuration.idx_qs_source + 3];
        //     Matrix2 rotation_source_inv;
        //     rotation_source_inv << c_theta_source, s_theta_source, -s_theta_source,
        //     c_theta_source; q_target.template segment<2>(configuration.idx_qs_target) =
        //       -rotation_source_inv * q_source.template segment<2>(configuration.idx_qs_source);
        //     q_target[configuration.idx_qs_target + 2] = c_theta_source;
        //     q_target[configuration.idx_qs_target + 3] = -s_theta_source;
        //   }
        // }
        //
        // ReturnType operator()(const JointModelUniversalTpl<Scalar, Options> &) const
        // {
        //   if (joint.same_direction)
        //   {
        //     // Copy q
        //     q_target.template segment<2>(configuration.idx_qs_target) =
        //       q_source.template segment<2>(configuration.idx_qs_source);
        //   }
        //   else
        //   {
        //     // Axes are inversed so swap q
        //     q_target[configuration.idx_qs_target] = q_source[configuration.idx_qs_source + 1];
        //     q_target[configuration.idx_qs_target + 1] = q_source[configuration.idx_qs_source];
        //   }
        // }

        ReturnType operator()(const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          // Nothing to do, q conversion is managed in mimicked joint.
        }

        ReturnType
        operator()(const JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &) const
        {
          assert(false && "This must never happened");
        }
      };
    } // namespace internal

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    template<typename TangentVectorType1, typename TangentVectorType2>
    void ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>::convertTangent(
      const Eigen::MatrixBase<TangentVectorType1> & v_source,
      const Eigen::MatrixBase<TangentVectorType2> & v_target) const
    {
      if (source_tangent_size != v_source.size())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "convertTangent - wrong source tangent size");
      }
      if (target_tangent_size != v_target.size())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "convertTangent - wrong target tangent size");
      }

      for (std::size_t i = 0; i < joint_mapping.size(); ++i)
      {
        const auto & tangent = tangent_mapping[i];
        const auto & joint = joint_mapping[i];
        boost::apply_visitor(
          internal::TangentConverterVisitor<
            Scalar, Options, JointCollectionTpl, TangentVectorType1, TangentVectorType2>(
            v_source, v_target, tangent, joint),
          joint.joint);
      }
    }

  } // namespace graph

} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_model_configuration_converter_hpp__
