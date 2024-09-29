//
// Copyright (c) 2019-2021 INRIA
//

#ifndef __pinocchio_multibody_joint_mimic_hpp__
#define __pinocchio_multibody_joint_mimic_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include <boost/variant.hpp>
#include <boost/mpl/filter_view.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/placeholders.hpp>
#include <iostream>

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  struct ScaledJointMotionSubspaceTpl;

  template<typename _Scalar, int _Options>
  struct traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>
  {
    typedef JointMotionSubspaceTpl<Eigen::Dynamic, _Scalar, _Options> RefJointMotionSubspace;
    typedef typename traits<RefJointMotionSubspace>::Scalar Scalar;
    enum
    {
      Options = traits<RefJointMotionSubspace>::Options
    };
    enum
    {
      LINEAR = traits<RefJointMotionSubspace>::LINEAR,
      ANGULAR = traits<RefJointMotionSubspace>::ANGULAR
    };
    typedef typename traits<RefJointMotionSubspace>::JointMotion JointMotion;
    typedef typename traits<RefJointMotionSubspace>::JointForce JointForce;
    typedef typename traits<RefJointMotionSubspace>::DenseBase DenseBase;
    typedef typename traits<RefJointMotionSubspace>::MatrixReturnType MatrixReturnType;
    typedef typename traits<RefJointMotionSubspace>::ConstMatrixReturnType ConstMatrixReturnType;
  }; // traits ScaledJointMotionSubspaceTpl

  template<typename _Scalar, int _Options>
  struct SE3GroupAction<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>
  {
    typedef
      typename SE3GroupAction<typename traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::
                                RefJointMotionSubspace>::ReturnType ReturnType;
  };

  template<typename _Scalar, int _Options, typename MotionDerived>
  struct MotionAlgebraAction<ScaledJointMotionSubspaceTpl<_Scalar, _Options>, MotionDerived>
  {
    typedef typename MotionAlgebraAction<
      typename traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace,
      MotionDerived>::ReturnType ReturnType;
  };

  template<typename _Scalar, int _Options, typename ForceDerived>
  struct ConstraintForceOp<ScaledJointMotionSubspaceTpl<_Scalar, _Options>, ForceDerived>
  {
    typedef typename traits<
      ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace::Scalar Scalar;
    // typedef typename ConstraintForceOp<typename traits<ScaledJointMotionSubspaceTpl<_Scalar,
    // _Options>>::RefJointMotionSubspace,ForceDerived>::ReturnType OriginalReturnType;
    typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options> OriginalReturnType;

    typedef typename ScalarMatrixProduct<Scalar, OriginalReturnType>::type IdealReturnType;
    typedef Eigen::Matrix<
      Scalar,
      IdealReturnType::RowsAtCompileTime,
      IdealReturnType::ColsAtCompileTime,
      traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace::Options>
      ReturnType;
  };

  template<typename _Scalar, int _Options, typename ForceSet>
  struct ConstraintForceSetOp<ScaledJointMotionSubspaceTpl<_Scalar, _Options>, ForceSet>
  {
    typedef typename traits<
      ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace::Scalar Scalar;
    // typedef typename ConstraintForceSetOp<typename traits<ScaledJointMotionSubspaceTpl<_Scalar,
    // _Options>>::RefJointMotionSubspace, ForceSet>::ReturnType OriginalReturnType;
    typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options> OriginalReturnType;
    typedef typename ScalarMatrixProduct<Scalar, OriginalReturnType>::type IdealReturnType;
    typedef Eigen::Matrix<
      Scalar,
      traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace::NV,
      ForceSet::ColsAtCompileTime,
      traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace::Options
        | Eigen::RowMajor>
      ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct ScaledJointMotionSubspaceTpl
  : JointMotionSubspaceBase<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ScaledJointMotionSubspaceTpl)
    enum
    {
      NV = Eigen::Dynamic
    };
    typedef JointMotionSubspaceBase<ScaledJointMotionSubspaceTpl> Base;
    using Base::nv;

    typedef typename traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options>>::RefJointMotionSubspace
      RefJointMotionSubspace;
    typedef typename SE3GroupAction<RefJointMotionSubspace>::ReturnType SE3ActionReturnType;

    ScaledJointMotionSubspaceTpl()
    : ScaledJointMotionSubspaceTpl(1.0)
    {
    }

    explicit ScaledJointMotionSubspaceTpl(const Scalar & scaling_factor)
    : m_scaling_factor(scaling_factor)
    , m_constraint(0)
    {
    }

    ScaledJointMotionSubspaceTpl(
      const RefJointMotionSubspace & constraint, const Scalar & scaling_factor)
    : m_constraint(constraint)
    , m_scaling_factor(scaling_factor)
    {
    }

    ScaledJointMotionSubspaceTpl(const ScaledJointMotionSubspaceTpl & other)
    : m_constraint(other.m_constraint)
    , m_scaling_factor(other.m_scaling_factor)
    {
    }

    ScaledJointMotionSubspaceTpl & operator=(const ScaledJointMotionSubspaceTpl & other)
    {
      m_constraint = other.m_constraint;
      m_scaling_factor = other.m_scaling_factor;
      return *this;
    }

    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & v) const
    {

      assert(v.size() == nv());
      JointMotion jm = m_constraint * v;
      return m_scaling_factor * jm;
    }

    template<typename S1, int O1>
    SE3ActionReturnType se3Action(const SE3Tpl<S1, O1> & m) const
    {
      SE3ActionReturnType res = m_constraint.se3Action(m);
      return m_scaling_factor * res;
    }

    template<typename S1, int O1>
    SE3ActionReturnType se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      SE3ActionReturnType res = m_constraint.se3ActionInverse(m);
      return m_scaling_factor * res;
    }

    int nv_impl() const
    {
      return m_constraint.nv();
    }

    struct TransposeConst
    {
      const ScaledJointMotionSubspaceTpl & ref;
      TransposeConst(const ScaledJointMotionSubspaceTpl & ref)
      : ref(ref)
      {
      }

      template<typename Derived>
      typename ConstraintForceOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType
      operator*(const ForceDense<Derived> & f) const
      {
        // TODO: I don't know why, but we should a dense a return type, otherwise it failes at the
        // evaluation level;
        typedef
          typename ConstraintForceOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType ReturnType;
        return ReturnType(ref.m_scaling_factor * (ref.m_constraint.transpose() * f));
      }

      /// [CRBA]  MatrixBase operator* (RefConstraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        typedef typename ConstraintForceSetOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType
          ReturnType;
        return ReturnType(ref.m_scaling_factor * (ref.m_constraint.transpose() * F));
      }

    }; // struct TransposeConst

    TransposeConst transpose() const
    {
      return TransposeConst(*this);
    }

    const DenseBase & matrix_impl() const
    {
      S = m_scaling_factor * m_constraint.matrix_impl();
      return S;
    }

    DenseBase & matrix_impl()
    {
      S = m_scaling_factor * m_constraint.matrix_impl();
      return S;
    }

    template<typename MotionDerived>
    typename MotionAlgebraAction<ScaledJointMotionSubspaceTpl, MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename MotionAlgebraAction<ScaledJointMotionSubspaceTpl, MotionDerived>::ReturnType
        ReturnType;
      ReturnType res = m_scaling_factor * m_constraint.motionAction(m);
      return res;
    }

    inline const Scalar & scaling() const
    {
      return m_scaling_factor;
    }
    inline Scalar & scaling()
    {
      return m_scaling_factor;
    }

    inline const RefJointMotionSubspace & constraint() const
    {
      return m_constraint.derived();
    }
    inline RefJointMotionSubspace & constraint()
    {
      return m_constraint.derived();
    }

    bool isEqual(const ScaledJointMotionSubspaceTpl & other) const
    {
      return m_constraint == other.m_constraint && m_scaling_factor == other.m_scaling_factor;
    }

  protected:
    RefJointMotionSubspace m_constraint;
    Scalar m_scaling_factor;
    mutable DenseBase S;
  }; // struct ScaledJointMotionSubspaceTpl

  template<typename S1, int O1, typename S2, int O2>
  struct MultiplicationOp<InertiaTpl<S1, O1>, ScaledJointMotionSubspaceTpl<S2, O2>>
  {
    typedef InertiaTpl<S1, O1> Inertia;
    typedef ScaledJointMotionSubspaceTpl<S2, O2> Constraint;
    typedef typename Constraint::Scalar Scalar;

    // typedef typename MultiplicationOp<Inertia,typename Constraint::RefConstraint>::ReturnType
    // OriginalReturnType;
    typedef Eigen::Matrix<S2, 6, Eigen::Dynamic, O2> ReturnType;
    // typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type ReturnType;
    // typedef OriginalReturnType ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1, O1>, ScaledJointMotionSubspaceTpl<S2, O2>>
    {
      typedef InertiaTpl<S1, O1> Inertia;
      typedef ScaledJointMotionSubspaceTpl<S2, O2> Constraint;
      typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType ReturnType;

      static inline ReturnType run(const Inertia & Y, const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y * scaled_constraint.constraint());
      }
    };
  } // namespace impl

  template<typename M6Like, typename S2, int O2>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledJointMotionSubspaceTpl<S2, O2>>
  {
    typedef ScaledJointMotionSubspaceTpl<S2, O2> Constraint;
    typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType OriginalReturnType;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(OriginalReturnType) ReturnType;
  };

  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename S2, int O2>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledJointMotionSubspaceTpl<S2, O2>>
    {
      typedef ScaledJointMotionSubspaceTpl<S2, O2> Constraint;
      typedef
        typename MultiplicationOp<Eigen::MatrixBase<M6Like>, Constraint>::ReturnType ReturnType;

      static inline ReturnType
      run(const Eigen::MatrixBase<M6Like> & Y, const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y.derived() * scaled_constraint.constraint());
      }
    };
  } // namespace impl

  template<typename _Scalar, int _Options>
  struct JointCollectionMimicableTpl
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    // Joint Revolute
    typedef JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
    typedef JointModelRevoluteTpl<Scalar, Options, 1> JointModelRY;
    typedef JointModelRevoluteTpl<Scalar, Options, 2> JointModelRZ;

    // Joint Revolute Unaligned
    typedef JointModelRevoluteUnalignedTpl<Scalar, Options> JointModelRevoluteUnaligned;

    // Joint Revolute UBounded
    typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 0> JointModelRUBX;
    typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 1> JointModelRUBY;
    typedef JointModelRevoluteUnboundedTpl<Scalar, Options, 2> JointModelRUBZ;

    // Joint Revolute Unbounded Unaligned
    typedef JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options>
      JointModelRevoluteUnboundedUnaligned;

    // Joint Prismatic
    typedef JointModelPrismaticTpl<Scalar, Options, 0> JointModelPX;
    typedef JointModelPrismaticTpl<Scalar, Options, 1> JointModelPY;
    typedef JointModelPrismaticTpl<Scalar, Options, 2> JointModelPZ;

    // Joint Prismatic Unaligned
    typedef JointModelPrismaticUnalignedTpl<Scalar, Options> JointModelPrismaticUnaligned;

    // Joint Translation
    typedef JointModelTranslationTpl<Scalar, Options> JointModelTranslation;

    // Joint FreeFlyer
    typedef JointModelFreeFlyerTpl<Scalar, Options> JointModelFreeFlyer;

    typedef boost::variant<
      JointModelRX,
      JointModelRY,
      JointModelRZ,
      JointModelRevoluteUnaligned,
      JointModelPX,
      JointModelPY,
      JointModelPZ,
      JointModelPrismaticUnaligned,
      JointModelTranslation,
      JointModelRUBX,
      JointModelRUBY,
      JointModelRUBZ,
      JointModelRevoluteUnboundedUnaligned, 
      JointModelFreeFlyer>
      JointModelVariant;

    // Joint Revolute
    typedef JointDataRevoluteTpl<Scalar, Options, 0> JointDataRX;
    typedef JointDataRevoluteTpl<Scalar, Options, 1> JointDataRY;
    typedef JointDataRevoluteTpl<Scalar, Options, 2> JointDataRZ;

    // Joint Revolute Unaligned
    typedef JointDataRevoluteUnalignedTpl<Scalar, Options> JointDataRevoluteUnaligned;

    // Joint Revolute Unaligned
    typedef JointDataRevoluteUnboundedUnalignedTpl<Scalar, Options>
      JointDataRevoluteUnboundedUnaligned;

    // Joint Revolute UBounded
    typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 0> JointDataRUBX;
    typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 1> JointDataRUBY;
    typedef JointDataRevoluteUnboundedTpl<Scalar, Options, 2> JointDataRUBZ;

    // Joint Prismatic
    typedef JointDataPrismaticTpl<Scalar, Options, 0> JointDataPX;
    typedef JointDataPrismaticTpl<Scalar, Options, 1> JointDataPY;
    typedef JointDataPrismaticTpl<Scalar, Options, 2> JointDataPZ;

    // Joint Prismatic Unaligned
    typedef JointDataPrismaticUnalignedTpl<Scalar, Options> JointDataPrismaticUnaligned;

    // Joint Translation
    typedef JointDataTranslationTpl<Scalar, Options> JointDataTranslation;

    // Joint FreeFlyer
    typedef JointDataFreeFlyerTpl<Scalar, Options> JointDataFreeFlyer;

    typedef boost::variant<
      JointDataRX,
      JointDataRY,
      JointDataRZ,
      JointDataRevoluteUnaligned,
      JointDataPX,
      JointDataPY,
      JointDataPZ,
      JointDataPrismaticUnaligned,
      JointDataTranslation,
      JointDataRUBX,
      JointDataRUBY,
      JointDataRUBZ,
      JointDataRevoluteUnboundedUnaligned, 
      JointDataFreeFlyer>
      JointDataVariant;
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointMimicTpl;
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelMimicTpl;
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataMimicTpl;

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    typedef _Scalar Scalar;

    enum
    {
      Options = _Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic,
      NJ = Eigen::Dynamic
    };

    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef JointDataMimicTpl<Scalar, Options, JointCollectionTpl> JointDataDerived;
    typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelDerived;
    typedef ScaledJointMotionSubspaceTpl<Scalar, Options> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;

    typedef MotionTpl<Scalar, Options> Motion_t;
    typedef MotionTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> U_t;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> UD_t;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> TangentVector_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointDataMimicTpl<_Scalar, Options, JointCollectionTpl>>
  {
    typedef JointMimicTpl<_Scalar, Options, JointCollectionTpl> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointModelMimicTpl<_Scalar, Options, JointCollectionTpl>>
  {
    typedef JointMimicTpl<_Scalar, Options, JointCollectionTpl> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataMimicTpl
  : public JointDataBase<JointDataMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef JointDataBase<JointDataMimicTpl> Base;
    typedef JointMimicTpl<_Scalar, _Options, JointCollectionTpl> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);

    // typedef typename boost::make_variant_over<typename boost::mpl::filter_view<typename
    // JointDataVariant::types, is_mimicable<boost::mpl::_1>>::type>::type
    // MimicableJointDataVariant;
    typedef JointDataTpl<_Scalar, _Options, JointCollectionMimicableTpl> RefJointData;
    typedef typename RefJointData::JointDataVariant RefJointDataVariant;

    JointDataMimicTpl()
    : m_scaling((Scalar)0)
    , S((Scalar)0)
    {
      m_q_transform.resize(0);
      m_v_transform.resize(0);
    }

    // JointDataMimicTpl(const JointDataMimicTpl & other)
    // { *this = other; }

    JointDataMimicTpl(
      const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
      const Scalar & scaling,
      const Scalar & nq,
      const Scalar & nv)
    : m_scaling(scaling)
    , S(m_jdata_ref.S(), scaling)
    , m_jdata_ref(
        transferToVariant<JointDataTpl<Scalar, Options, JointCollectionTpl>, RefJointData>(jdata))
    {
      m_q_transform.resize(nq, 1);
      m_v_transform.resize(nv, 1);
    }

    // JointDataMimicTpl(const RefJointDataVariant & jdata,
    //                const Scalar & scaling,
    //                const Scalar & nq,
    //                const Scalar & nv)
    // : m_jdata_ref(jdata)
    // , m_scaling(scaling)
    // , S(m_jdata_ref.S(),scaling)
    // {

    // }

    JointDataMimicTpl(
      const RefJointData & jdata, const Scalar & scaling, const Scalar & nq, const Scalar & nv)
    : m_jdata_ref(jdata.derived())
    , m_scaling(scaling)
    , S(m_jdata_ref.S(), scaling)
    {
      m_q_transform.resize(nq, 1);
      m_v_transform.resize(nv, 1);
    }

    JointDataMimicTpl & operator=(const JointDataMimicTpl & other)
    {
      m_jdata_ref = other.m_jdata_ref;
      m_scaling = other.m_scaling;
      m_q_transform = other.m_q_transform;
      m_v_transform = other.m_v_transform;
      S = Constraint_t(m_jdata_ref.S(), other.m_scaling);
      return *this;
    }

    using Base::isEqual;
    bool isEqual(const JointDataMimicTpl & other) const
    {
      return Base::isEqual(other) && m_jdata_ref == other.m_jdata_ref
             && m_scaling == other.m_scaling && m_q_transform == other.m_q_transform
             && m_v_transform == other.m_v_transform;
    }

    static std::string classname()
    {
      return std::string("JointDataMimic");
    }

    std::string shortname() const
    {
      return classname();
    }

    // // Accessors
    ConstraintTypeConstRef S_accessor() const
    {
      return S;
    }
    ConstraintTypeRef S_accessor()
    {
      return S;
    }

    TansformTypeConstRef M_accessor() const
    {
      M_ = m_jdata_ref.M();
      return M_;
    }
    TansformTypeRef M_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      M_ = m_jdata_ref.M();
      return M_;
    }

    MotionTypeConstRef v_accessor() const
    {
      v_ = m_jdata_ref.v();
      return v_;
    }
    MotionTypeRef v_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      v_ = m_jdata_ref.v();
      return v_;
    }

    BiasTypeConstRef c_accessor() const
    {
      c_ = m_jdata_ref.c();
      return c_;
    }
    BiasTypeRef c_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      c_ = m_jdata_ref.c();
      return c_;
    }

    UTypeConstRef U_accessor() const
    {
      U_ = m_jdata_ref.U();
      return U_;
    }
    UTypeRef U_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      U_ = m_jdata_ref.U();
      return U_;
    }

    DTypeConstRef Dinv_accessor() const
    {
      Dinv_ = m_jdata_ref.Dinv();
      return Dinv_;
    }
    DTypeRef Dinv_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      Dinv_ = m_jdata_ref.Dinv();
      return Dinv_;
    }

    UDTypeConstRef UDinv_accessor() const
    {
      UDinv_ = m_jdata_ref.UDinv();
      return UDinv_;
    }
    UDTypeRef UDinv_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      UDinv_ = m_jdata_ref.UDinv();
      return UDinv_;
    }

    DTypeConstRef StU_accessor() const
    {
      StU_ = m_jdata_ref.StU();
      return StU_;
    }
    DTypeRef StU_accessor()
    {
      // assert(false && "Changes to non const ref on mimic joints won't be taken into account. Use
      // const ref");
      StU_ = m_jdata_ref.StU();
      return StU_;
    }

    friend struct JointModelMimicTpl<_Scalar, _Options, JointCollectionTpl>;

    const RefJointData & jdata() const
    {
      return m_jdata_ref;
    }
    RefJointData & jdata()
    {
      return m_jdata_ref;
    }

    const Scalar & scaling() const
    {
      return m_scaling;
    }
    Scalar & scaling()
    {
      return m_scaling;
    }

    ConfigVector_t & joint_q_accessor()
    {
      return m_q_transform;
    }
    const ConfigVector_t & joint_q_accessor() const
    {
      return m_q_transform;
    }

    TangentVector_t & joint_v_accessor()
    {
      return m_v_transform;
    }
    const TangentVector_t & joint_v_accessor() const
    {
      return m_v_transform;
    }

  protected:
    RefJointData m_jdata_ref;
    Scalar m_scaling;

    /// \brief Transform configuration vector
    ConfigVector_t m_q_transform;
    /// \brief Transform velocity vector.
    TangentVector_t m_v_transform;

  public:
    // data
    Constraint_t S;

  protected:
    /// \brief Buffer variable for accessors to return references
    mutable Transformation_t M_;
    mutable Motion_t v_;
    mutable Bias_t c_;
    mutable U_t U_;
    mutable D_t Dinv_;
    mutable UD_t UDinv_;
    mutable D_t StU_;

  }; // struct JointDataMimicTpl

  template<
    typename NewScalar,
    typename Scalar,
    int Options,
    template<typename S, int O>
    class JointCollectionTpl>
  struct CastType<NewScalar, JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  {
    typedef JointModelMimicTpl<NewScalar, Options, JointCollectionTpl> type;
  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelMimicTpl
  : public JointModelBase<JointModelMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef JointModelBase<JointModelMimicTpl> Base;
    typedef JointMimicTpl<_Scalar, _Options, JointCollectionTpl> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef JointModelTpl<Scalar, Options, JointCollectionMimicableTpl> JointModel;
    typedef typename JointModel::JointModelVariant JointModelVariant;
    // typedef typename boost::make_variant_over<typename boost::mpl::filter_view<typename
    // JointModelVariant::types, is_mimicable<boost::mpl::_1>>::type>::type
    // MimicableJointModelVariant; typedef JointModelTpl<_Scalar, _Options,
    // JointCollectionMimicableTpl> MimicableJointModel;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef InertiaTpl<Scalar, Options> Inertia;

    using Base::id;
    using Base::idx_j;
    using Base::idx_q;
    using Base::idx_v;
    using Base::nj;
    using Base::nq;
    using Base::nv;
    using Base::setIndexes;

    JointModelMimicTpl()
    {
    }

    JointModelMimicTpl(
      const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
      const Scalar & scaling,
      const Scalar & offset)
    : JointModelMimicTpl(jmodel, jmodel, scaling, offset)
    {
    }

    JointModelMimicTpl(
      const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel_mimicking,
      const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel_mimicked,
      const Scalar & scaling,
      const Scalar & offset)
    : m_scaling(scaling)
    , m_offset(offset)
    , m_jmodel_ref(
        transferToVariant<JointModelTpl<Scalar, Options, JointCollectionTpl>, JointModel>(
          jmodel_mimicking))
    {
      assert(jmodel_mimicking.nq() == jmodel_mimicked.nq());
      assert(jmodel_mimicking.nv() == jmodel_mimicked.nv());
      assert(jmodel_mimicking.nj() == jmodel_mimicked.nj());

      m_jmodel_ref.setIndexes(
        jmodel_mimicked.id(), jmodel_mimicked.idx_q(), jmodel_mimicked.idx_v(),
        jmodel_mimicked.idx_j());
    }

    template<typename JointModel>
    JointModelMimicTpl(
      const JointModelBase<JointModel> & jmodel, const Scalar & scaling, const Scalar & offset)
    : JointModelMimicTpl(jmodel, jmodel, scaling, offset)
    {
    }

    template<typename JointModelMimicking, typename JointModelMimicked>
    JointModelMimicTpl(
      const JointModelBase<JointModelMimicking> & jmodel_mimicking,
      const JointModelBase<JointModelMimicked> & jmodel_mimicked,
      const Scalar & scaling,
      const Scalar & offset)
    : m_jmodel_ref((JointModelVariant)jmodel_mimicking.derived())
    , m_scaling(scaling)
    , m_offset(offset)
    {
      assert(jmodel_mimicking.nq() == jmodel_mimicked.nq());
      assert(jmodel_mimicking.nv() == jmodel_mimicked.nv());
      assert(jmodel_mimicking.nj() == jmodel_mimicked.nj());

      m_jmodel_ref.setIndexes(
        jmodel_mimicked.id(), jmodel_mimicked.idx_q(), jmodel_mimicked.idx_v(),
        jmodel_mimicked.idx_j());
    }

    template<typename JointModelMimicking>
    JointModelMimicTpl(const JointModelBase<JointModelMimicking> & jmodel_mimicking,
                       const JointModelTpl<Scalar, Options, JointCollectionTpl > & jmodel_mimicked,
                       const Scalar & scaling,
                       const Scalar & offset)
    : m_jmodel_ref((JointModelVariant)jmodel_mimicking.derived())
    , m_scaling(scaling)
    , m_offset(offset)
    {
      assert(jmodel_mimicking.nq() == jmodel_mimicked.nq());
      assert(jmodel_mimicking.nv() == jmodel_mimicked.nv());
      assert(jmodel_mimicking.nj() == jmodel_mimicked.nj());

      m_jmodel_ref.setIndexes(jmodel_mimicked.id(), jmodel_mimicked.idx_q(), jmodel_mimicked.idx_v(), jmodel_mimicked.idx_j());
    }

    Base & base()
    {
      return *static_cast<Base *>(this);
    }
    const Base & base() const
    {
      return *static_cast<const Base *>(this);
    }

    inline int nq_impl() const
    {
      return 0;
    }
    inline int nv_impl() const
    {
      return 0;
    }
    inline int nj_impl() const
    {
      return m_jmodel_ref.nj();
    }

    inline int idx_q_impl() const
    {
      return m_jmodel_ref.idx_q();
    }
    inline int idx_v_impl() const
    {
      return m_jmodel_ref.idx_v();
    }

    void setIndexes_impl(JointIndex id, int q, int v, int j)
    {
      Base::i_id = id; // Only the id of the joint in the model is different.
      Base::i_q = m_jmodel_ref.idx_q();
      Base::i_v = m_jmodel_ref.idx_v();
      Base::i_j = j;
    }

    JointDataDerived createData() const
    {

      return JointDataDerived(
        m_jmodel_ref.createData(), scaling(), m_jmodel_ref.nq(), m_jmodel_ref.nv());
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return m_jmodel_ref.hasConfigurationLimit();
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return m_jmodel_ref.hasConfigurationLimitInTangent();
    }

    template<typename ConfigVector>
    EIGEN_DONT_INLINE void
    calc(JointDataDerived & jdata, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVectorAffineTransform<JointDerived>::Type AffineTransform;

      AffineTransform::run(
        qs.segment(m_jmodel_ref.idx_q(), m_jmodel_ref.nq()), m_scaling, m_offset,
        jdata.m_q_transform);

      m_jmodel_ref.calc(jdata.m_jdata_ref, qs);
    }

    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE void calc(
      JointDataDerived & jdata,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      typedef typename ConfigVectorAffineTransform<JointDerived>::Type AffineTransform;

      AffineTransform::run(
        qs.segment(m_jmodel_ref.idx_q(), m_jmodel_ref.nq()), m_scaling, m_offset,
        jdata.m_q_transform);
      jdata.m_v_transform = m_scaling * vs.segment(m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
      
      m_jmodel_ref.calc(jdata.m_jdata_ref, qs , vs);
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      // TODO: fixme
      assert(
        false
        && "Joint Mimic is not supported for aba yet. Remove it from your model if you want to use "
           "this function");
      m_jmodel_ref.calc_aba(
        data.m_jdata_ref, armature, PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I), update_I);
    }

    static std::string classname()
    {
      return std::string("JointModelMimic");
    }

    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar, JointModelMimicTpl>::type cast() const
    {
      typedef typename CastType<NewScalar, JointModelMimicTpl>::type ReturnType;

      ReturnType res(
        m_jmodel_ref.template cast<NewScalar>(), (NewScalar)m_scaling, (NewScalar)m_offset);
      res.setIndexes(id(), idx_q(), idx_v(), idx_j());
      return res;
    }

    const JointModel & jmodel() const
    {
      return m_jmodel_ref;
    }
    JointModel & jmodel()
    {
      return m_jmodel_ref;
    }

    const Scalar & scaling() const
    {
      return m_scaling;
    }
    Scalar & scaling()
    {
      return m_scaling;
    }

    const Scalar & offset() const
    {
      return m_offset;
    }
    Scalar & offset()
    {
      return m_offset;
    }

  protected:
    // data
    JointModel m_jmodel_ref;
    Scalar m_scaling, m_offset;

  public:
    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromDofSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), m_jmodel_ref.idx_q(), m_jmodel_ref.nq());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromDofSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), m_jmodel_ref.idx_q(), m_jmodel_ref.nq());
    }

    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromNqSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), m_jmodel_ref.idx_q(), 0);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromNqSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), m_jmodel_ref.idx_q(), 0);
    }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), m_jmodel_ref.idx_v(),  m_jmodel_ref.nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
    }

    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointVelCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointVelCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
    }

    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    joinVeltRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointVelRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv());
    }

    // /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the
    // matrix Mat
    // // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointVelBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(
        Mat.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv(),
        m_jmodel_ref.nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointVelBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(
        Mat.derived(), m_jmodel_ref.idx_v(), m_jmodel_ref.idx_v(), m_jmodel_ref.nv(),
        m_jmodel_ref.nv());
    }

  }; // struct JointModelMimicTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_constructor<
    ::pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_copy<::pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_constructor<
    ::pinocchio::JointDataMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_copy<::pinocchio::JointDataMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_mimic_hpp__
