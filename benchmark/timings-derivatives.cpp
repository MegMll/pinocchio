//
// Copyright (c) 2018-2021 CNRS INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

template<typename Matrix1, typename Matrix2, typename Matrix3>
void rnea_fd(
  const pinocchio::Model & model,
  pinocchio::Data & data_fd,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a,
  const Eigen::MatrixBase<Matrix1> & _drnea_dq,
  const Eigen::MatrixBase<Matrix2> & _drnea_dv,
  const Eigen::MatrixBase<Matrix3> & _drnea_da)
{
  Matrix1 & drnea_dq = PINOCCHIO_EIGEN_CONST_CAST(Matrix1, _drnea_dq);
  Matrix2 & drnea_dv = PINOCCHIO_EIGEN_CONST_CAST(Matrix2, _drnea_dv);
  Matrix3 & drnea_da = PINOCCHIO_EIGEN_CONST_CAST(Matrix3, _drnea_da);

  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;

  VectorXd tau0 = rnea(model, data_fd, q, v, a);

  // dRNEA/dq
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    integrate(model, q, v_eps, q_plus);
    tau_plus = rnea(model, data_fd, q_plus, v, a);

    drnea_dq.col(k) = (tau_plus - tau0) / alpha;
    v_eps[k] -= alpha;
  }

  // dRNEA/dv
  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    tau_plus = rnea(model, data_fd, q, v_plus, a);

    drnea_dv.col(k) = (tau_plus - tau0) / alpha;
    v_plus[k] -= alpha;
  }

  // dRNEA/da
  drnea_da = pinocchio::crba(model, data_fd, q, pinocchio::Convention::WORLD);
  drnea_da.template triangularView<Eigen::StrictlyLower>() =
    drnea_da.transpose().template triangularView<Eigen::StrictlyLower>();
}

void aba_fd(
  const pinocchio::Model & model,
  pinocchio::Data & data_fd,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau,
  Eigen::MatrixXd & daba_dq,
  Eigen::MatrixXd & daba_dv,
  pinocchio::Data::RowMatrixXs & daba_dtau)
{
  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;

  VectorXd a0 = pinocchio::aba(model, data_fd, q, v, tau, pinocchio::Convention::LOCAL);

  // dABA/dq
  for (int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model, q, v_eps);
    a_plus = pinocchio::aba(model, data_fd, q_plus, v, tau, pinocchio::Convention::LOCAL);

    daba_dq.col(k) = (a_plus - a0) / alpha;
    v_eps[k] -= alpha;
  }

  // dABA/dv
  VectorXd v_plus(v);
  for (int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = pinocchio::aba(model, data_fd, q, v_plus, tau, pinocchio::Convention::LOCAL);

    daba_dv.col(k) = (a_plus - a0) / alpha;
    v_plus[k] -= alpha;
  }

  // dABA/dtau
  daba_dtau = computeMinverse(model, data_fd, q);
}

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
#ifdef NDEBUG
  const int NBT = 1000 * 100;
#else
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant) " << std::endl;
#endif

  Model model;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if (argc > 1)
    filename = argv[1];
  bool with_ff = true;

  if (argc > 2)
  {
    const std::string ff_option = argv[2];
    if (ff_option == "-no-ff")
      with_ff = false;
  }

  if (filename == "HS")
    buildModels::humanoidRandom(model, true);
  else if (with_ff)
    pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  //      pinocchio::urdf::buildModel(filename,JointModelRX(),model);
  else
    pinocchio::urdf::buildModel(filename, model);
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;

  Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qs(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qdots(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qddots(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) taus(NBT);

  for (size_t i = 0; i < NBT; ++i)
  {
    qs[i] = randomConfiguration(model, -qmax, qmax);
    qdots[i] = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

  PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(MatrixXd) drnea_dq(MatrixXd::Zero(model.nv, model.nv));
  PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(MatrixXd) drnea_dv(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd drnea_da(MatrixXd::Zero(model.nv, model.nv));

  MatrixXd daba_dq(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd daba_dv(MatrixXd::Zero(model.nv, model.nv));
  Data::RowMatrixXs daba_dtau(Data::RowMatrixXs::Zero(model.nv, model.nv));

  typedef Data::Tensor3x Tensor3x;
  //  typedef Eigen::Tensor<double, 3, Eigen::RowMajor> Tensor3x;
  Tensor3x dtau2_dq(model.nv, model.nv, model.nv);
  Tensor3x dtau2_dv(model.nv, model.nv, model.nv);
  Tensor3x dtau2_dqv(model.nv, model.nv, model.nv);
  Tensor3x dtau_dadq(model.nv, model.nv, model.nv);
  dtau2_dq.setZero();
  dtau2_dv.setZero();
  dtau2_dqv.setZero();
  dtau_dadq.setZero();

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "FK= \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeForwardKinematicsDerivatives(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "FK derivatives= \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "RNEA= \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeRNEADerivatives(
      model, data, qs[_smooth], qdots[_smooth], qddots[_smooth], drnea_dq, drnea_dv, drnea_da);
  }
  std::cout << "RNEA derivatives= \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    ComputeRNEASecondOrderDerivatives(
      model, data, qs[_smooth], qdots[_smooth], qddots[_smooth], dtau2_dq, dtau2_dv, dtau2_dqv,
      dtau_dadq);
  }
  std::cout << "RNEA derivatives SO= \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT / 100)
  {
    rnea_fd(
      model, data, qs[_smooth], qdots[_smooth], qddots[_smooth], drnea_dq, drnea_dv, drnea_da);
  }
  std::cout << "RNEA finite differences= \t";
  timer.toc(std::cout, NBT / 100);

  timer.tic();
  SMOOTH(NBT)
  {
    aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::LOCAL);
  }
  std::cout << "ABA= \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeABADerivatives(
      model, data, qs[_smooth], qdots[_smooth], taus[_smooth], daba_dq, daba_dv, daba_dtau);
  }
  std::cout << "ABA derivatives(q,v,tau)= \t";
  timer.toc(std::cout, NBT);

  {
    double total = 0;
    SMOOTH(NBT)
    {
      aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::WORLD);
      timer.tic();
      computeABADerivatives(model, data, daba_dq, daba_dv, daba_dtau);
      total += timer.toc(timer.DEFAULT_UNIT);
    }
    std::cout << "ABA derivatives() = \t\t" << (total / NBT) << " "
              << timer.unitName(timer.DEFAULT_UNIT) << std::endl;
  }

  timer.tic();
  SMOOTH(NBT / 100)
  {
    aba_fd(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], daba_dq, daba_dv, daba_dtau);
  }
  std::cout << "ABA finite differences= \t";
  timer.toc(std::cout, NBT / 100);

  timer.tic();
  SMOOTH(NBT)
  {
    computeMinverse(model, data, qs[_smooth]);
  }
  std::cout << "M.inverse(q) = \t\t\t";
  timer.toc(std::cout, NBT);

  {
    double total = 0;
    SMOOTH(NBT)
    {
      aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::WORLD);
      timer.tic();
      computeMinverse(model, data);
      total += timer.toc(timer.DEFAULT_UNIT);
    }
    std::cout << "M.inverse() from ABA = \t\t" << (total / NBT) << " "
              << timer.unitName(timer.DEFAULT_UNIT) << std::endl;
  }

  MatrixXd Minv(model.nv, model.nv);
  Minv.setZero();
  timer.tic();
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
    cholesky::decompose(model, data);
    cholesky::computeMinv(model, data, Minv);
  }
  std::cout << "Minv from Cholesky = \t\t";
  timer.toc(std::cout, NBT);

  std::cout << "--" << std::endl;
  return 0;
}
