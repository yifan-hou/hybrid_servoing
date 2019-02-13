#include "solvehfvc.h"

#include <iostream>
#include <algorithm>
#include <Eigen/LU>
#include <vector>

#include "eiquadprog.hpp"

using std::cout;
using std::endl;
using Eigen::VectorXf;
using Eigen::MatrixXf;


bool solvehfvc(const MatrixXf &Omega,
  const MatrixXf &Jac_phi_q_all,
  const MatrixXf &G, const VectorXf &b_G,
  const VectorXf &F,
  const MatrixXf &Aeq, const VectorXf &beq,
  const MatrixXf &A, const VectorXf &b_A,
  const int kDimActualized, const int kDimUnActualized,
  const int kDimSlidingFriction, const int kDimLambda,
  const int kNumSeeds,
  HFVC *action) {

// function [n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, ...
//         Jac_phi_q_all, G, b_G, F, Aeq, beq, A, b_A, dims, varargin)

  /* Size checking */
  const int kDimGeneralized = kDimActualized + kDimUnActualized;
  assert(Omega.rows() == Jac_phi_q_all.cols()); // dim q
  assert(Omega.cols() == kDimGeneralized);
  assert(Jac_phi_q_all.rows() == kDimLambda + kDimSlidingFriction);
  assert(G.rows() == b_G.rows());
  assert(G.cols() == kDimGeneralized);
  assert(F.rows() == kDimGeneralized);
  if (kDimSlidingFriction > 0) {
    assert(Aeq.rows() == beq.rows());
    assert(Aeq.cols() == kDimLambda + kDimGeneralized);
  }
  assert(A.rows() == b_A.rows());
  assert(A.cols() == kDimLambda + kDimGeneralized);

  MatrixXf Jac_phi_q;
  Jac_phi_q = Jac_phi_q_all.topRows(kDimLambda);

  cout << "Begin solving for velocity commands" << endl;
  cout << "  [1] Determine Possible Dimension of control" << endl;

  MatrixXf N = Jac_phi_q*Omega;
  MatrixXf NG(N.rows()+G.rows(), N.cols());
  NG << N, G;


  // matrix decomposition
  Eigen::FullPivLU<MatrixXf> lu_decomp_N(N);
  int rank_N = lu_decomp_N.rank();
  MatrixXf basis_N = lu_decomp_N.kernel(); // columns of basis_N forms a basis
                                           // of the null-space of N
  Eigen::FullPivLU<MatrixXf> lu_decomp_NG(NG);
  int rank_NG = lu_decomp_NG.rank();
  MatrixXf basis_NG = lu_decomp_NG.kernel(); // columns of basis_NG forms a basis
                                           // of the null-space of NG
  MatrixXf C_c(basis_NG.cols()+kDimUnActualized, basis_NG.rows());
  C_c << basis_NG.transpose(),
      MatrixXf::Identity(kDimUnActualized,kDimUnActualized),
      MatrixXf::Zero(kDimUnActualized,kDimActualized);
  Eigen::FullPivLU<MatrixXf>
      lu_decomp_C_c(C_c);
  MatrixXf basis_c = lu_decomp_C_c.kernel(); // columns of basis_C_c forms a basis
                                           // of the null-space of C_c

  int n_av = rank_NG - rank_N;
  int n_af = kDimActualized - n_av;

  cout << "   n_av = " << n_av << ", n_af = " << n_af << endl;
  assert(rank_N + kDimActualized >= kDimGeneralized);

  cout << "  [2] Solving for Directions by PGD" << endl;

  int NIter   = 300;
  int n_c     = rank_NG - kDimUnActualized;
  MatrixXf BB = basis_c.transpose()*basis_c;
  MatrixXf NN = basis_N*basis_N.transpose();

  std::vector<MatrixXf> k_all;
  float cost_all[kNumSeeds] = {0};
  for (int seed = 0; seed < kNumSeeds; ++seed)  {
    MatrixXf k  = MatrixXf::Random(n_c, n_av); // initial solution
    MatrixXf bck = basis_c*k;
    for (int i = 0; i < bck.cols(); i++) {
        float bck_col_norm = bck.col(i).norm();
        k.col(i) /= bck_col_norm;
    }
    MatrixXf g(n_c, n_av);
    float costs = 0;
    for (int iter = 0; iter < NIter; ++iter) {
      // compute gradient
      g = MatrixXf::Zero(n_c, n_av);
      costs = 0;
      for (int i = 0; i < n_av; ++i) {
        for (int j = 0; j < n_av; ++j) {
            if (i == j) continue;
            float tempcost = (k.col(i).transpose()*BB*k.col(j)).norm();
            costs += tempcost*tempcost;
            g.col(i) += 2.0f*(k.col(i).transpose()*BB*k.col(j))(0)*BB*k.col(j);
        }
        g.col(i) -= 2.0f*basis_c.transpose()*NN*basis_c*k.col(i);
        costs -= k.col(i).transpose()*basis_c.transpose()*NN*basis_c*k.col(i);
      }
      // descent
      k -= 1.0f*g;
      // project
      bck = basis_c*k;
      for (int i = 0; i < bck.cols(); i++) {
          float bck_col_norm = bck.col(i).norm();
          k.col(i) /= bck_col_norm;
      }
      // cout << "     cost: " << costs << ", grad: " << g.norm() << endl;
    }
    cost_all[seed] = costs;
    k_all.push_back(k);
  }
  const int SizeFloat = sizeof(cost_all) / sizeof(float);
  float *cost_best    = std::min_element(cost_all, cost_all + kNumSeeds);
  int min_id          = std::distance(cost_all, cost_best);
  MatrixXf k_best     = k_all[min_id];
  MatrixXf C_best     = (basis_c*k_best).transpose();
  // // double check
  // cout << "min id cost: " << cost_all[min_id] << endl;
  // cout << "min cost: " << *cost_best << endl;


  // R_a = [null(C_best(:, kDimUnActualized+1:end))';
  //         C_best(:, kDimUnActualized+1:end)];
  MatrixXf R_a(kDimActualized, kDimActualized);
  MatrixXf C_best_actualized = C_best.rightCols(kDimActualized);
  Eigen::FullPivLU<MatrixXf> lu_decomp_C_best_actualized(C_best_actualized);
  MatrixXf basis_C_best_actualized =
      lu_decomp_C_best_actualized.kernel(); // columns of basis_NG forms a basis
                // of the null-space of C_best_actualized
  R_a << basis_C_best_actualized.transpose(), C_best_actualized;
  MatrixXf T = MatrixXf::Zero(kDimGeneralized, kDimGeneralized);
  T.topLeftCorner(kDimUnActualized, kDimUnActualized) =
      MatrixXf::Identity(kDimUnActualized, kDimUnActualized);
  T.bottomRightCorner(kDimActualized,kDimActualized) = R_a;

  // b_NG = [zeros(size(N, 1), 1); b_G];
  VectorXf b_NG = VectorXf::Zero(N.rows() + b_G.rows());
  b_NG.tail(b_G.rows()) = b_G;

  // v_star = NG\b_NG;
  VectorXf v_star = NG.fullPivLu().solve(b_NG);
  // cout << "C_best: " << C_best.rows() << ", " << C_best.cols() << endl;
  // cout << C_best;

  VectorXf w_av = C_best*v_star;


  cout << "Begin Solving for force commands." << endl;
  // unactuated dimensions
  // H = [eye(kDimUnActualized), zeros(kDimUnActualized, kDimActualized)];
  MatrixXf H = MatrixXf::Zero(kDimUnActualized, kDimGeneralized);
  H.leftCols(kDimUnActualized) = MatrixXf::Identity(kDimUnActualized,
      kDimUnActualized);

  // Newton's laws
  MatrixXf T_inv = T.inverse();
  MatrixXf M_newton(kDimUnActualized+kDimGeneralized,
      kDimGeneralized + kDimLambda);
  if (kDimSlidingFriction > 0) {
    M_newton.resize(kDimUnActualized+kDimGeneralized+Aeq.rows(),
      kDimGeneralized);
    M_newton << MatrixXf::Zero(kDimUnActualized, kDimLambda), H*T_inv,
        T*Omega.transpose()*Jac_phi_q_all.transpose(),
        MatrixXf::Identity(kDimGeneralized, kDimGeneralized), Aeq;
  } else {
    M_newton << MatrixXf::Zero(kDimUnActualized, kDimLambda), H*T_inv,
        T*Omega.transpose()*Jac_phi_q_all.transpose(),
        MatrixXf::Identity(kDimGeneralized, kDimGeneralized);
  }
  VectorXf b_newton(M_newton.rows());
  if (kDimSlidingFriction > 0) {
    b_newton << VectorXf::Zero(H.rows()), -T*F, beq;
  } else {
    b_newton << VectorXf::Zero(H.rows()), -T*F;
  }

  MatrixXf M_free(M_newton.rows(), M_newton.cols() - n_af);
  M_free << M_newton.leftCols(kDimLambda+kDimUnActualized),
      M_newton.rightCols(n_av);
  MatrixXf M_eta_af = M_newton.middleCols(kDimLambda+kDimUnActualized,
      n_af);

  // prepare the QP
  // min 0.5 * x G0 x + g0 x
  // s.t.
  //     CE^T x + ce0 = 0
  //     CI^T x + ci0 >= 0
  // variables: [free_force, dual_free_force, eta_af]
  int n_free = kDimLambda + kDimUnActualized + n_av;
  int n_dual_free = M_newton.rows();
  Eigen::VectorXd Gdiag = Eigen::VectorXd::Zero(n_free+n_dual_free+n_af);
  for (int i = 0; i < n_af; ++i) Gdiag(n_free+n_dual_free+i) = 1.0;
  for (int i = 0; i < Gdiag.rows(); ++i) Gdiag(i) += 1e-3; // regularization

  Eigen::MatrixXd G0 = Gdiag.asDiagonal();
  Eigen::VectorXd g0 = Eigen::VectorXd::Zero(G0.rows());
  // A_temp = [A(:, 1:kDimLambda), A(:, kDimLambda+1:end)*T_inv];
  MatrixXf A_temp(A.rows(), A.cols());
  A_temp << A.leftCols(kDimLambda), A.rightCols(kDimGeneralized)*T_inv;

  // 0.5 x'Qx + f'x
  // Ax<b
  // A_lambda_eta_u = A_temp(:, 1:kDimLambda+kDimUnActualized);
  // A_eta_af = A_temp(:, kDimLambda+kDimUnActualized+1:kDimLambda+kDimUnActualized+n_af);
  // A_eta_av = A_temp(:, kDimLambda+kDimUnActualized+n_af+1:end);
  // qp.A = [A_lambda_eta_u A_eta_av zeros(size(A, 1), n_dual_free) A_eta_af];
  MatrixXf qpA(A.rows(), A.cols() + n_dual_free);
  qpA << A_temp.leftCols(kDimLambda+kDimUnActualized),
      A_temp.rightCols(n_av), MatrixXf::Zero(A.rows(), n_dual_free),
      A_temp.middleCols(kDimLambda+kDimUnActualized, n_af);
  VectorXf qpb = b_A;

  // Aeq x = beq
  // qpAeq = [2*eye(n_free), M_free', zeros(n_free, n_af);
  //           M_free, zeros(size(M_free, 1)), M_eta_af];
  // qpbeq = [zeros(n_free, 1); b_newton];
  MatrixXf qpAeq(n_free+M_free.rows(), A.cols() + n_dual_free);
  qpAeq << 2*MatrixXf::Identity(n_free,n_free), M_free.transpose(),
      MatrixXf::Zero(n_free, n_af), M_free,
      MatrixXf::Zero(M_free.rows(), M_free.rows()), M_eta_af;
  VectorXf qpbeq(n_free+b_newton.rows());
  qpbeq << VectorXf::Zero(n_free), b_newton;

  Eigen::VectorXd x = Eigen::VectorXd::Random(g0.rows());
  // cout << "QP:" << endl;
  // cout << "G0: " << G0 << endl;
  // cout << "g0: " << g0 << endl;
  // cout << "qpAeq: " << qpAeq << endl;
  // cout << "qpbeq: " << qpbeq << endl;
  // cout << "qpA: " << qpA << endl;
  // cout << "qpb: " << qpb << endl;
  double cost = solve_quadprog(G0, g0,
      qpAeq.transpose().cast<double>(), -qpbeq.cast<double>(),
      -qpA.transpose().cast<double>(), qpb.cast<double>(), x);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");
  cout << "QP Solved. cost = " << cost << ". x = " << x.format(OctaveFmt)
      << endl << endl;
  // eta_af = x(n_free + n_dual_free + 1:end);
  VectorXf eta_af = x.segment(n_free+n_dual_free, n_af).cast<float>();


  // check results
  VectorXf vel_command(kDimActualized);
  vel_command << VectorXf::Zero(n_af), w_av;
  cout<< " World frame velocity: " << (R_a.inverse()*vel_command).format(OctaveFmt)
      << endl;

  VectorXf force_command(kDimActualized);
  force_command << eta_af, VectorXf::Zero(n_av);
  cout << "  World frame force: " << (R_a.inverse()*force_command).format(OctaveFmt)
      << endl;

  cout << "  Equality violation: " << (qpbeq - qpAeq*x.cast<float>()).norm()
      << endl;

  VectorXf b_Ax = qpA*x.cast<float>() - qpb;
  float inequality_violation = 0;
  if (b_Ax.maxCoeff() > 0 ) inequality_violation = b_Ax.maxCoeff();
  cout << "  Inequality violation: " << inequality_violation
      << endl;

  action->n_av   = n_av;
  action->n_af   = n_af;
  action->R_a    = R_a;
  action->w_av   = w_av;
  action->eta_af = eta_af;

  return true;
}