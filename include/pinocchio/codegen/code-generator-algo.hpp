//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_codegen_code_generator_algo_hpp__
#define __pinocchio_codegen_code_generator_algo_hpp__

#include "pinocchio/codegen/code-generator-base.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

namespace pinocchio
{
  template<typename _Scalar>
  struct CodeGenRNEA : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenRNEA(
      const Model & model,
      const std::string & function_name = "rnea",
      const std::string & library_name = "cg_rnea_eval")
    : Base(model, model.nq + 2 * model.nv, model.nv, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_a = ADTangentVectorType(model.nv);
      ad_a.setZero();
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());

      dtau_dq = MatrixXs::Zero(model.nv, model.nq);
      dtau_dv = MatrixXs::Zero(model.nv, model.nv);
      dtau_da = MatrixXs::Zero(model.nv, model.nv);
    }

    virtual ~CodeGenRNEA()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_a = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::rnea(ad_model, ad_data, ad_q, ad_v, ad_a);

      ad_Y = ad_data.tau;

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    using Base::evalFunction;
    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & a)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;
      it += ad_model.nv;
      x.segment(it, ad_model.nv) = a;
      it += ad_model.nv;

      evalFunction(x);
      res = Base::y;
    }

    using Base::evalJacobian;
    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalJacobian(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & a)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;
      it += ad_model.nv;
      x.segment(it, ad_model.nv) = a;
      it += ad_model.nv;

      evalJacobian(x);
      it = 0;
      dtau_dq = Base::jac.middleCols(it, ad_model.nq);
      it += ad_model.nq;
      dtau_dv = Base::jac.middleCols(it, ad_model.nv);
      it += ad_model.nv;
      dtau_da = Base::jac.middleCols(it, ad_model.nv);
      it += ad_model.nv;
    }

    MatrixXs dtau_dq, dtau_dv, dtau_da;

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::jac;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q, ad_q_plus;
    ADTangentVectorType ad_dq, ad_v, ad_a;
  };

  template<typename _Scalar>
  struct CodeGenABA : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenABA(
      const Model & model,
      const std::string & function_name = "aba",
      const std::string & library_name = "cg_aba_eval")
    : Base(model, model.nq + 2 * model.nv, model.nv, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_tau = ADTangentVectorType(model.nv);
      ad_tau.setZero();
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());

      da_dq = MatrixXs::Zero(model.nv, model.nq);
      da_dv = MatrixXs::Zero(model.nv, model.nv);
      da_dtau = MatrixXs::Zero(model.nv, model.nv);
    }

    virtual ~CodeGenABA()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_tau = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::aba(ad_model, ad_data, ad_q, ad_v, ad_tau, Convention::WORLD);
      ad_Y = ad_data.ddq;

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    using Base::evalFunction;
    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & tau)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;
      it += ad_model.nv;
      x.segment(it, ad_model.nv) = tau;
      it += ad_model.nv;

      evalFunction(x);
      res = Base::y;
    }

    using Base::evalJacobian;
    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalJacobian(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & tau)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;
      it += ad_model.nv;
      x.segment(it, ad_model.nv) = tau;
      it += ad_model.nv;

      evalJacobian(x);

      it = 0;
      da_dq = Base::jac.middleCols(it, ad_model.nq);
      it += ad_model.nq;
      da_dv = Base::jac.middleCols(it, ad_model.nv);
      it += ad_model.nv;
      da_dtau = Base::jac.middleCols(it, ad_model.nv);
      it += ad_model.nv;
    }

    MatrixXs da_dq, da_dv, da_dtau;

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::jac;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q, ad_q_plus;
    ADTangentVectorType ad_dq, ad_v, ad_tau;
  };

  template<typename _Scalar>
  struct CodeGenCRBA : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenCRBA(
      const Model & model,
      const std::string & function_name = "crba",
      const std::string & library_name = "cg_crba_eval")
    : Base(model, model.nq, (model.nv * (model.nv + 1)) / 2, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());

      M = MatrixXs::Zero(model.nv, model.nq);

      Base::build_jacobian = false;
    }

    virtual ~CodeGenCRBA()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;

      pinocchio::crba(ad_model, ad_data, ad_q, pinocchio::Convention::WORLD);
      Eigen::DenseIndex it_Y = 0;

      for (Eigen::DenseIndex i = 0; i < ad_model.nv; ++i)
      {
        for (Eigen::DenseIndex j = i; j < ad_model.nv; ++j)
        {
          ad_Y[it_Y++] = ad_data.M(i, j);
        }
      }

      assert(it_Y == Base::getOutputDimension());

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType>
    void evalFunction(const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;

      Base::evalFunction(x);

      // fill M
      Eigen::DenseIndex it_Y = 0;
      for (Eigen::DenseIndex i = 0; i < ad_model.nv; ++i)
      {
        for (Eigen::DenseIndex j = i; j < ad_model.nv; ++j)
        {
          M(i, j) = Base::y[it_Y++];
        }
      }

      assert(it_Y == Base::getOutputDimension());
    }

    MatrixXs M;

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q;
  };

  template<typename _Scalar>
  struct CodeGenMinv : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenMinv(
      const Model & model,
      const std::string & function_name = "minv",
      const std::string & library_name = "cg_minv_eval")
    : Base(model, model.nq, (model.nv * (model.nv + 1)) / 2, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());

      Minv = MatrixXs::Zero(model.nv, model.nq);

      Base::build_jacobian = false;
    }

    virtual ~CodeGenMinv()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;

      pinocchio::computeMinverse(ad_model, ad_data, ad_q);
      Eigen::DenseIndex it_Y = 0;
      for (Eigen::DenseIndex i = 0; i < ad_model.nv; ++i)
      {
        for (Eigen::DenseIndex j = i; j < ad_model.nv; ++j)
        {
          ad_Y[it_Y++] = ad_data.Minv(i, j);
        }
      }

      assert(it_Y == Base::getOutputDimension());

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType>
    void evalFunction(const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;

      Base::evalFunction(x);

      // fill Minv
      Eigen::DenseIndex it_Y = 0;
      for (Eigen::DenseIndex i = 0; i < ad_model.nv; ++i)
      {
        for (Eigen::DenseIndex j = i; j < ad_model.nv; ++j)
        {
          Minv(i, j) = Base::y[it_Y++];
        }
      }
    }

    MatrixXs Minv;

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q;
  };

  template<typename _Scalar>
  struct CodeGenRNEADerivatives : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXs) RowMatrixXs;
    typedef typename Base::VectorXs VectorXs;

    typedef typename Base::ADData ADData;
    typedef typename ADData::MatrixXs ADMatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(ADMatrixXs) RowADMatrixXs;

    CodeGenRNEADerivatives(
      const Model & model,
      const std::string & function_name = "partial_rnea",
      const std::string & library_name = "cg_partial_rnea_eval")
    : Base(model, model.nq + 2 * model.nv, 3 * model.nv * model.nv, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_a = ADTangentVectorType(model.nv);
      ad_a.setZero();

      x = VectorXs::Zero(Base::getInputDimension());

      ad_dtau_dq = ADMatrixXs::Zero(model.nv, model.nv);
      ad_dtau_dv = ADMatrixXs::Zero(model.nv, model.nv);
      ad_dtau_da = ADMatrixXs::Zero(model.nv, model.nv);

      dtau_dq = MatrixXs::Zero(model.nv, model.nv);
      dtau_dv = MatrixXs::Zero(model.nv, model.nv);
      dtau_da = MatrixXs::Zero(model.nv, model.nv);

      Base::build_jacobian = false;
    }

    virtual ~CodeGenRNEADerivatives()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_a = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::computeRNEADerivatives(
        ad_model, ad_data, ad_q, ad_v, ad_a, ad_dtau_dq, ad_dtau_dv, ad_dtau_da);

      assert(ad_Y.size() == Base::getOutputDimension());

      Eigen::DenseIndex it_Y = 0;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dtau_dq;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dtau_dv;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dtau_da;
      it_Y += ad_model.nv * ad_model.nv;

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & a)
    {
      // fill x
      Eigen::DenseIndex it_x = 0;
      x.segment(it_x, ad_model.nq) = q;
      it_x += ad_model.nq;
      x.segment(it_x, ad_model.nv) = v;
      it_x += ad_model.nv;
      x.segment(it_x, ad_model.nv) = a;
      it_x += ad_model.nv;

      Base::evalFunction(x);

      // fill partial derivatives
      Eigen::DenseIndex it_y = 0;
      dtau_dq = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dtau_dv = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dtau_da = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
    }

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    ADMatrixXs ad_dtau_dq, ad_dtau_dv, ad_dtau_da;
    MatrixXs dtau_dq, dtau_dv, dtau_da;

    ADConfigVectorType ad_q;
    ADTangentVectorType ad_v, ad_a;
  };

  template<typename _Scalar>
  struct CodeGenABADerivatives : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXs) RowMatrixXs;
    typedef typename Base::VectorXs VectorXs;

    typedef typename Base::ADData ADData;
    typedef typename ADData::MatrixXs ADMatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(ADMatrixXs) RowADMatrixXs;

    CodeGenABADerivatives(
      const Model & model,
      const std::string & function_name = "partial_aba",
      const std::string & library_name = "cg_partial_aba_eval")
    : Base(model, model.nq + 2 * model.nv, 3 * model.nv * model.nv, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_tau = ADTangentVectorType(model.nv);
      ad_tau.setZero();

      x = VectorXs::Zero(Base::getInputDimension());

      ad_dddq_dq = ADMatrixXs::Zero(model.nv, model.nv);
      ad_dddq_dv = ADMatrixXs::Zero(model.nv, model.nv);
      ad_dddq_dtau = ADMatrixXs::Zero(model.nv, model.nv);

      dddq_dq = MatrixXs::Zero(model.nv, model.nv);
      dddq_dv = MatrixXs::Zero(model.nv, model.nv);
      dddq_dtau = MatrixXs::Zero(model.nv, model.nv);

      Base::build_jacobian = false;
    }

    virtual ~CodeGenABADerivatives()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_tau = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::computeABADerivatives(
        ad_model, ad_data, ad_q, ad_v, ad_tau, ad_dddq_dq, ad_dddq_dv, ad_dddq_dtau);

      assert(ad_Y.size() == Base::getOutputDimension());

      Eigen::DenseIndex it_Y = 0;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dddq_dq;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dddq_dv;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_dddq_dtau;
      it_Y += ad_model.nv * ad_model.nv;

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & tau)
    {
      // fill x
      Eigen::DenseIndex it_x = 0;
      x.segment(it_x, ad_model.nq) = q;
      it_x += ad_model.nq;
      x.segment(it_x, ad_model.nv) = v;
      it_x += ad_model.nv;
      x.segment(it_x, ad_model.nv) = tau;
      it_x += ad_model.nv;

      Base::evalFunction(x);

      // fill partial derivatives
      Eigen::DenseIndex it_y = 0;
      dddq_dq = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dddq_dv = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dddq_dtau = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
    }

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    ADMatrixXs ad_dddq_dq, ad_dddq_dv, ad_dddq_dtau;
    MatrixXs dddq_dq, dddq_dv, dddq_dtau;

    ADConfigVectorType ad_q;
    ADTangentVectorType ad_v, ad_tau;
  };

  template<typename _Scalar>
  struct CodeGenConstraintDynamicsDerivatives : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;
    typedef typename Base::ADScalar ADScalar;

    typedef typename Base::Model Model;

    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXs) RowMatrixXs;
    typedef typename Base::VectorXs VectorXs;

    typedef typename pinocchio::RigidConstraintModelTpl<Scalar, Base::Options> ContactModel;
    typedef Eigen::aligned_allocator<ContactModel> ConstraintModelAllocator;
    typedef typename std::vector<ContactModel, ConstraintModelAllocator> ContactModelVector;
    typedef typename pinocchio::RigidConstraintDataTpl<Scalar, Base::Options> ContactData;
    typedef Eigen::aligned_allocator<ContactData> ConstraintDataAllocator;
    typedef typename std::vector<ContactData, ConstraintDataAllocator> ContactDataVector;

    typedef typename pinocchio::RigidConstraintModelTpl<ADScalar, Base::Options> ADContactModel;
    typedef Eigen::aligned_allocator<ADContactModel> ADConstraintModelAllocator;
    typedef typename std::vector<ADContactModel, ADConstraintModelAllocator> ADContactModelVector;
    typedef typename pinocchio::RigidConstraintDataTpl<ADScalar, Base::Options> ADContactData;
    typedef Eigen::aligned_allocator<ADContactData> ADConstraintDataAllocator;
    typedef typename std::vector<ADContactData, ADConstraintDataAllocator> ADContactDataVector;

    typedef typename Base::ADData ADData;
    typedef typename ADData::MatrixXs ADMatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(ADMatrixXs) RowADMatrixXs;

    Eigen::DenseIndex constraintDim(const ContactModelVector & contact_models) const
    {
      Eigen::DenseIndex num_total_constraints = 0;
      for (typename ContactModelVector::const_iterator it = contact_models.begin();
           it != contact_models.end(); ++it)
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
          it->size() > 0, "The dimension of the constraint must be positive");
        num_total_constraints += it->size();
      }
      return num_total_constraints;
    }

    CodeGenConstraintDynamicsDerivatives(
      const Model & model,
      const ContactModelVector & contact_models,
      const std::string & function_name = "partial_constraintDynamics",
      const std::string & library_name = "cg_partial_constraintDynamics_eval")
    : Base(
        model,
        model.nq + 2 * model.nv,
        3 * model.nv * model.nv + 3 * constraintDim(contact_models) * model.nv,
        function_name,
        library_name)
    , nc(constraintDim(contact_models))
    , dddq_dq(model.nv, model.nv)
    , dddq_dv(model.nv, model.nv)
    , dddq_dtau(model.nv, model.nv)
    {
      dlambda_dq.resize(nc, model.nv);
      dlambda_dq.setZero();
      dlambda_dv.resize(nc, model.nv);
      dlambda_dv.setZero();
      dlambda_dtau.resize(nc, model.nv);
      dlambda_dtau.setZero();

      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_tau = ADTangentVectorType(model.nv);
      ad_tau.setZero();

      x = VectorXs::Zero(Base::getInputDimension());

      for (int k = 0; k < contact_models.size(); ++k)
      {
        ad_contact_models.push_back(contact_models[k].template cast<ADScalar>());
      }

      for (int k = 0; k < ad_contact_models.size(); ++k)
      {
        ad_contact_datas.push_back(ADContactData(ad_contact_models[k]));
      }

      pinocchio::initConstraintDynamics(ad_model, ad_data, ad_contact_models);
      Base::build_jacobian = false;
    }

    virtual ~CodeGenConstraintDynamicsDerivatives()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_tau = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::constraintDynamics(
        ad_model, ad_data, ad_q, ad_v, ad_tau, ad_contact_models, ad_contact_datas);
      pinocchio::computeConstraintDynamicsDerivatives(
        ad_model, ad_data, ad_contact_models, ad_contact_datas);
      assert(ad_Y.size() == Base::getOutputDimension());

      Eigen::DenseIndex it_Y = 0;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_data.ddq_dq;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_data.ddq_dv;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<RowADMatrixXs>(ad_Y.data() + it_Y, ad_model.nv, ad_model.nv) = ad_data.ddq_dtau;
      it_Y += ad_model.nv * ad_model.nv;
      Eigen::Map<ADMatrixXs>(ad_Y.data() + it_Y, nc, ad_model.nv) = ad_data.dlambda_dq;
      it_Y += nc * ad_model.nv;
      Eigen::Map<ADMatrixXs>(ad_Y.data() + it_Y, nc, ad_model.nv) = ad_data.dlambda_dv;
      it_Y += nc * ad_model.nv;
      Eigen::Map<ADMatrixXs>(ad_Y.data() + it_Y, nc, ad_model.nv) = ad_data.dlambda_dtau;
      it_Y += nc * ad_model.nv;
      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & tau)
    {
      // fill x
      Eigen::DenseIndex it_x = 0;
      x.segment(it_x, ad_model.nq) = q;
      it_x += ad_model.nq;
      x.segment(it_x, ad_model.nv) = v;
      it_x += ad_model.nv;
      x.segment(it_x, ad_model.nv) = tau;
      it_x += ad_model.nv;

      Base::evalFunction(x);

      // fill partial derivatives
      Eigen::DenseIndex it_y = 0;
      dddq_dq = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dddq_dv = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dddq_dtau = Eigen::Map<RowMatrixXs>(Base::y.data() + it_y, ad_model.nv, ad_model.nv);
      it_y += ad_model.nv * ad_model.nv;
      dlambda_dq = Eigen::Map<MatrixXs>(Base::y.data() + it_y, nc, ad_model.nv);
      it_y += nc * ad_model.nv;
      dlambda_dv = Eigen::Map<MatrixXs>(Base::y.data() + it_y, nc, ad_model.nv);
      it_y += nc * ad_model.nv;
      dlambda_dtau = Eigen::Map<MatrixXs>(Base::y.data() + it_y, nc, ad_model.nv);
      it_y += nc * ad_model.nv;
    }

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    Eigen::DenseIndex nc;
    ADContactModelVector ad_contact_models;
    ADContactDataVector ad_contact_datas;

    VectorXs x;
    MatrixXs dddq_dq, dddq_dv, dddq_dtau;
    MatrixXs dlambda_dq, dlambda_dv, dlambda_dtau;

    ADConfigVectorType ad_q;
    ADTangentVectorType ad_v, ad_tau;
  };

  template<typename _Scalar>
  struct CodeGenConstraintDynamics : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;
    typedef typename Base::ADScalar ADScalar;

    typedef typename Base::Model Model;

    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXs) RowMatrixXs;
    typedef typename Base::VectorXs VectorXs;

    typedef typename pinocchio::RigidConstraintModelTpl<Scalar, Base::Options> ContactModel;
    typedef Eigen::aligned_allocator<ContactModel> ConstraintModelAllocator;
    typedef typename std::vector<ContactModel, ConstraintModelAllocator> ContactModelVector;
    typedef typename pinocchio::RigidConstraintDataTpl<Scalar, Base::Options> ContactData;
    typedef Eigen::aligned_allocator<ContactData> ConstraintDataAllocator;
    typedef typename std::vector<ContactData, ConstraintDataAllocator> ContactDataVector;

    typedef typename pinocchio::RigidConstraintModelTpl<ADScalar, Base::Options> ADContactModel;
    typedef Eigen::aligned_allocator<ADContactModel> ADConstraintModelAllocator;
    typedef typename std::vector<ADContactModel, ADConstraintModelAllocator> ADContactModelVector;
    typedef typename pinocchio::RigidConstraintDataTpl<ADScalar, Base::Options> ADContactData;
    typedef Eigen::aligned_allocator<ADContactData> ADConstraintDataAllocator;
    typedef typename std::vector<ADContactData, ADConstraintDataAllocator> ADContactDataVector;

    typedef typename Base::ADData ADData;
    typedef typename ADData::MatrixXs ADMatrixXs;
    typedef typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(ADMatrixXs) RowADMatrixXs;

    Eigen::DenseIndex constraintDim(const ContactModelVector & contact_models) const
    {
      Eigen::DenseIndex num_total_constraints = 0;
      for (typename ContactModelVector::const_iterator it = contact_models.begin();
           it != contact_models.end(); ++it)
      {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
          it->size() > 0, "The dimension of the constraint must be positive");
        num_total_constraints += it->size();
      }
      return num_total_constraints;
    }

    CodeGenConstraintDynamics(
      const Model & model,
      const ContactModelVector & contact_models,
      const std::string & function_name = "constraintDynamics",
      const std::string & library_name = "cg_constraintDynamics_eval")
    : Base(
        model,
        model.nq + 2 * model.nv,
        model.nv + constraintDim(contact_models),
        function_name,
        library_name)
    , nc(constraintDim(contact_models))
    , da_dq(MatrixXs::Zero(model.nv, model.nq))
    , da_dv(MatrixXs::Zero(model.nv, model.nv))
    , da_dtau(MatrixXs::Zero(model.nv, model.nv))
    , ddq(model.nv)
    {
      lambda_c.resize(nc);
      lambda_c.setZero();
      dlambda_dq.resize(nc, model.nq);
      dlambda_dq.setZero();
      dlambda_dv.resize(nc, model.nv);
      dlambda_dv.setZero();
      dlambda_dtau.resize(nc, model.nv);
      dlambda_dtau.setZero();

      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      ad_tau = ADTangentVectorType(model.nv);
      ad_tau.setZero();

      x = VectorXs::Zero(Base::getInputDimension());

      for (int k = 0; k < contact_models.size(); ++k)
      {
        ad_contact_models.push_back(contact_models[k].template cast<ADScalar>());
      }

      for (int k = 0; k < ad_contact_models.size(); ++k)
      {
        ad_contact_datas.push_back(ADContactData(ad_contact_models[k]));
      }
      pinocchio::initConstraintDynamics(ad_model, ad_data, ad_contact_models);
    }

    virtual ~CodeGenConstraintDynamics()
    {
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;
      ad_tau = ad_X.segment(it, ad_model.nv);
      it += ad_model.nv;

      pinocchio::constraintDynamics(
        ad_model, ad_data, ad_q, ad_v, ad_tau, ad_contact_models, ad_contact_datas);
      ad_Y.head(ad_model.nv) = ad_data.ddq;
      ad_Y.segment(ad_model.nv, nc) = ad_data.lambda_c;

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & tau)
    {
      // fill x
      Eigen::DenseIndex it_x = 0;
      x.segment(it_x, ad_model.nq) = q;
      it_x += ad_model.nq;
      x.segment(it_x, ad_model.nv) = v;
      it_x += ad_model.nv;
      x.segment(it_x, ad_model.nv) = tau;
      it_x += ad_model.nv;

      Base::evalFunction(x);

      Eigen::DenseIndex it_y = 0;
      ddq = Base::y.segment(it_y, ad_model.nv);
      it_y += ad_model.nv;
      lambda_c = Base::y.segment(it_y, nc);
    }

    using Base::evalJacobian;
    template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
    void evalJacobian(
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVector1> & v,
      const Eigen::MatrixBase<TangentVector2> & a)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;
      it += ad_model.nv;
      x.segment(it, ad_model.nv) = a;
      it += ad_model.nv;

      evalJacobian(x);
      it = 0;
      da_dq = Base::jac.middleCols(it, ad_model.nq).topRows(ad_model.nv);
      dlambda_dq = Base::jac.middleCols(it, ad_model.nq).bottomRows(nc);
      it += ad_model.nq;
      da_dv = Base::jac.middleCols(it, ad_model.nv).topRows(ad_model.nv);
      dlambda_dv = Base::jac.middleCols(it, ad_model.nv).bottomRows(nc);
      it += ad_model.nv;
      da_dtau = Base::jac.middleCols(it, ad_model.nv).topRows(ad_model.nv);
      dlambda_dtau = Base::jac.middleCols(it, ad_model.nv).bottomRows(nc);
      it += ad_model.nv;
    }
    VectorXs lambda_c, ddq;
    MatrixXs da_dq, da_dv, da_dtau;
    MatrixXs dlambda_dq, dlambda_dv, dlambda_dtau;

  protected:
    using Base::ad_data;
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    ADContactModelVector ad_contact_models;
    ADContactDataVector ad_contact_datas;

    Eigen::DenseIndex nc;
    VectorXs x;

    ADConfigVectorType ad_q;
    ADTangentVectorType ad_v, ad_tau;
  };

  template<typename _Scalar>
  struct CodeGenIntegrate : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenIntegrate(
      const Model & model,
      const std::string & function_name = "integrate",
      const std::string & library_name = "cg_integrate_eval")
    : Base(model, model.nq + model.nv, model.nq, function_name, library_name)
    {
      ad_q = ADConfigVectorType(model.nq);
      ad_q = neutral(ad_model);
      ad_v = ADTangentVectorType(model.nv);
      ad_v.setZero();
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_v = ad_X.segment(it, ad_model.nv);
      pinocchio::integrate(ad_model, ad_q, ad_v, ad_Y);

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    using Base::evalFunction;
    template<typename ConfigVectorType1, typename TangentVector, typename ConfigVectorType2>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType1> & q,
      const Eigen::MatrixBase<TangentVector> & v,
      const Eigen::MatrixBase<ConfigVectorType2> & qout)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q;
      it += ad_model.nq;
      x.segment(it, ad_model.nv) = v;

      evalFunction(x);
      PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType2, qout) = Base::y;
    }

  protected:
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q;
    ADTangentVectorType ad_v;
  };

  template<typename _Scalar>
  struct CodeGenDifference : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenDifference(
      const Model & model,
      const std::string & function_name = "difference",
      const std::string & library_name = "cg_difference_eval")
    : Base(model, 2 * model.nq, model.nv, function_name, library_name)
    {
      ad_q0 = ADConfigVectorType(model.nq);
      ad_q0 = neutral(ad_model);
      ad_q1 = ADConfigVectorType(model.nq);
      ad_q1 = neutral(ad_model);
      x = VectorXs::Zero(Base::getInputDimension());
      res = VectorXs::Zero(Base::getOutputDimension());
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q0 = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_q1 = ad_X.segment(it, ad_model.nq);
      pinocchio::difference(ad_model, ad_q0, ad_q1, ad_Y);

      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    using Base::evalFunction;
    template<typename ConfigVectorType1, typename ConfigVectorType2, typename TangentVector>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType1> & q0,
      const Eigen::MatrixBase<ConfigVectorType2> & q1,
      const Eigen::MatrixBase<TangentVector> & v)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q0;
      it += ad_model.nq;
      x.segment(it, ad_model.nq) = q1;

      evalFunction(x);
      PINOCCHIO_EIGEN_CONST_CAST(TangentVector, v) = Base::y;
    }

  protected:
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    VectorXs res;

    ADConfigVectorType ad_q0;
    ADConfigVectorType ad_q1;
  };

  template<typename _Scalar>
  struct CodeGenDDifference : public CodeGenBase<_Scalar>
  {
    typedef CodeGenBase<_Scalar> Base;
    typedef typename Base::Scalar Scalar;

    typedef typename Base::Model Model;
    typedef typename Base::ADConfigVectorType ADConfigVectorType;
    typedef typename Base::ADTangentVectorType ADTangentVectorType;
    typedef typename Base::ADVectorXs ADVectorXs;
    typedef typename Base::ADMatrixXs ADMatrixXs;
    typedef typename Base::MatrixXs MatrixXs;
    typedef typename Base::VectorXs VectorXs;

    CodeGenDDifference(
      const Model & model,
      const std::string & function_name = "dDifference",
      const std::string & library_name = "cg_dDifference_eval")
    : Base(model, 2 * model.nq, 2 * model.nv * model.nv, function_name, library_name)
    {
      ad_q0 = neutral(ad_model);
      ad_q1 = neutral(ad_model);
      ad_J0 = ADMatrixXs::Zero(ad_model.nv, ad_model.nv);
      ad_J1 = ADMatrixXs::Zero(ad_model.nv, ad_model.nv);
      x = VectorXs::Zero(Base::getInputDimension());
    }

    void buildMap()
    {
      CppAD::Independent(ad_X);

      Eigen::DenseIndex it = 0;
      ad_q0 = ad_X.segment(it, ad_model.nq);
      it += ad_model.nq;
      ad_q1 = ad_X.segment(it, ad_model.nq);
      pinocchio::dDifference(ad_model, ad_q0, ad_q1, ad_J0, pinocchio::ARG0);
      pinocchio::dDifference(ad_model, ad_q0, ad_q1, ad_J1, pinocchio::ARG1);

      Eigen::Map<ADMatrixXs>(ad_Y.data(), 2 * ad_model.nv, ad_model.nv).topRows(ad_model.nv) =
        ad_J0;
      Eigen::Map<ADMatrixXs>(ad_Y.data(), 2 * ad_model.nv, ad_model.nv).bottomRows(ad_model.nv) =
        ad_J1;
      ad_fun.Dependent(ad_X, ad_Y);
      ad_fun.optimize("no_compare_op");
    }

    using Base::evalFunction;
    template<typename ConfigVectorType1, typename ConfigVectorType2, typename JacobianMatrix>
    void evalFunction(
      const Eigen::MatrixBase<ConfigVectorType1> & q0,
      const Eigen::MatrixBase<ConfigVectorType2> & q1,
      const Eigen::MatrixBase<JacobianMatrix> & J,
      const ArgumentPosition arg)
    {
      // fill x
      Eigen::DenseIndex it = 0;
      x.segment(it, ad_model.nq) = q0;
      it += ad_model.nq;
      x.segment(it, ad_model.nq) = q1;

      evalFunction(x);
      switch (arg)
      {
      case pinocchio::ARG0:
        PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix, J) =
          Eigen::Map<MatrixXs>(Base::y.data(), 2 * ad_model.nv, ad_model.nv).topRows(ad_model.nv);
        break;
      case pinocchio::ARG1:
        PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix, J) =
          Eigen::Map<MatrixXs>(Base::y.data(), 2 * ad_model.nv, ad_model.nv)
            .bottomRows(ad_model.nv);
        break;
      default:
        assert(false && "Wrong argument");
      }
    }

  protected:
    using Base::ad_fun;
    using Base::ad_model;
    using Base::ad_X;
    using Base::ad_Y;
    using Base::y;

    VectorXs x;
    ADConfigVectorType ad_q0;
    ADConfigVectorType ad_q1;
    ADMatrixXs ad_J0;
    ADMatrixXs ad_J1;
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_codegen_code_generator_algo_hpp__
