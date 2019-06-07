#include "solvehfvc.h"

#include <iostream>
#include <algorithm>
#include <Eigen/LU>
#include <vector>

#include "eiquadprog.hpp"

using std::cout;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;


bool solvehfvc(const MatrixXd &N_ALL,
  const MatrixXd &G, const VectorXd &b_G,
  const VectorXd &F,
  const MatrixXd &Aeq, const VectorXd &beq,
  const MatrixXd &A, const VectorXd &b_A,
  const int kDimActualized, const int kDimUnActualized,
  const int kDimSlidingFriction, const int kDimLambda,
  const int kNumSeeds,
  HFVC *action) {

  /* Size checking */
  const int kDimGeneralized = kDimActualized + kDimUnActualized;
  const int kDimContactForce = kDimLambda + kDimSlidingFriction;
  assert(N_ALL.cols() == kDimGeneralized);
  assert(N_ALL.rows() == kDimContactForce);
  assert(G.rows() == b_G.rows());
  assert(G.cols() == kDimGeneralized);
  assert(F.rows() == kDimGeneralized);
  if (kDimSlidingFriction > 0) {
    assert(Aeq.rows() == beq.rows());
    assert(Aeq.cols() == kDimContactForce + kDimGeneralized);
  }
  assert(A.rows() == b_A.rows());
  assert(A.cols() == kDimContactForce + kDimGeneralized);

  cout << "Begin solving for velocity commands" << endl;
  cout << "  [1] Determine Possible Dimension of control" << endl;

  MatrixXd N = N_ALL.topRows(kDimLambda);
  MatrixXd NG(N.rows()+G.rows(), N.cols());
  NG << N, G;

  // matrix decomposition
  //  There are two things to care about Eigen::FullPivLU.
  //  Firstly, if the matrix to be decomposed is empty or all zero, the kernel()
  //    method will throw a run time error.
  //  Secondly, if the null space has zero dimension, the output is NOT a zero
  //    dimensional matrix, but a n x 1 vector with all zeros.
  Eigen::FullPivLU<MatrixXd> lu_decomp_N(N);
  int rank_N = lu_decomp_N.rank();
  MatrixXd basis_N; // columns of basis_N forms a basis of the null-space of N
  if (rank_N > 0)
      basis_N = lu_decomp_N.kernel();
  else
      basis_N = MatrixXd::Identity(kDimGeneralized, kDimGeneralized);

  Eigen::FullPivLU<MatrixXd> lu_decomp_NG(NG);
  int rank_NG = lu_decomp_NG.rank();
  assert(rank_NG > 0);

  int n_av = rank_NG - rank_N;
  int n_af = kDimActualized - n_av;
  cout << "   n_av = " << n_av << ", n_af = " << n_af << endl;
  assert(rank_N + kDimActualized >= kDimGeneralized);

  MatrixXd basis_c;
  if (rank_NG < kDimGeneralized) {
    // null_NG is not empty
    // so C_c is also not empty
    MatrixXd null_NG = lu_decomp_NG.kernel(); // columns of null_NG forms a basis
                                             // of the null-space of NG
    MatrixXd C_c(null_NG.cols()+kDimUnActualized, kDimGeneralized);
    C_c << null_NG.transpose(),
        MatrixXd::Identity(kDimUnActualized,kDimUnActualized),
        MatrixXd::Zero(kDimUnActualized,kDimActualized);
    Eigen::FullPivLU<MatrixXd> lu_decomp_C_c(C_c);
    basis_c = lu_decomp_C_c.kernel(); // columns of basis_C_c forms a basis
                                             // of the null-space of C_c
  } else if (kDimUnActualized > 0) {
    MatrixXd C_c(kDimUnActualized, kDimGeneralized);
    C_c << MatrixXd::Identity(kDimUnActualized,kDimUnActualized),
        MatrixXd::Zero(kDimUnActualized,kDimActualized);
    Eigen::FullPivLU<MatrixXd> lu_decomp_C_c(C_c);
    basis_c = lu_decomp_C_c.kernel(); // columns of basis_C_c forms a basis
  } else {
    basis_c = MatrixXd::Identity(kDimGeneralized, kDimGeneralized);
  }

  MatrixXd R_a(kDimActualized, kDimActualized);
  MatrixXd T(kDimGeneralized, kDimGeneralized);
  VectorXd w_av;
  if (n_av == 0) {
    cout << "  [2] No feasible velocity control can satisfy the goal" << endl;
    R_a = MatrixXd::Identity(kDimActualized, kDimActualized);
    T = MatrixXd::Identity(kDimGeneralized, kDimGeneralized);
    w_av = VectorXd(0);
  } else {
    cout << "  [2] Solving for Directions by PGD" << endl;

    assert(basis_c.norm() > 0.1);// this shouldn't happen
    int NIter   = 50;
    int n_c     = rank_NG - kDimUnActualized;
    MatrixXd BB = basis_c.transpose()*basis_c;
    MatrixXd NN = basis_N*basis_N.transpose();

    std::vector<MatrixXd> k_all;
    float cost_all[kNumSeeds] = {0};
    for (int seed = 0; seed < kNumSeeds; ++seed)  {
      MatrixXd k  = MatrixXd::Random(n_c, n_av); // initial solution
      MatrixXd bck = basis_c*k;
      for (int i = 0; i < bck.cols(); i++) {
          float bck_col_norm = bck.col(i).norm();
          k.col(i) /= bck_col_norm;
      }
      MatrixXd g(n_c, n_av);
      float costs = 0;
      for (int iter = 0; iter < NIter; ++iter) {
        // compute gradient
        g = MatrixXd::Zero(n_c, n_av);
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
        k -= 10.0f*g;
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
    float *cost_best    = std::min_element(cost_all, cost_all + kNumSeeds);
    int min_id          = std::distance(cost_all, cost_best);
    MatrixXd k_best     = k_all[min_id];
    MatrixXd C_best     = (basis_c*k_best).transpose();

    // R_a = [null(C_best(:, kDimUnActualized+1:end))';
    //         C_best(:, kDimUnActualized+1:end)];
    // For this decomposition, the input C_best_actualized won't be empty
    // because it has kDimActualized cols; its output basis_C_best_actualized
    // also won't be empty as we are conditioned on n_av > 0
    MatrixXd C_best_actualized = C_best.rightCols(kDimActualized);
    Eigen::FullPivLU<MatrixXd> lu_decomp_C_best_actualized(C_best_actualized);
    MatrixXd basis_C_best_actualized;
    int rank_C_best_actualized = lu_decomp_C_best_actualized.rank();
    assert(rank_C_best_actualized > 0);
    basis_C_best_actualized = // columns of basis_C_best_actualized forms the
        lu_decomp_C_best_actualized.kernel(); // null space of C_best_actualized
    R_a = MatrixXd::Zero(kDimActualized, kDimActualized);
    R_a << basis_C_best_actualized.transpose(), C_best_actualized;
    T = MatrixXd::Zero(kDimGeneralized, kDimGeneralized);
    T.topLeftCorner(kDimUnActualized, kDimUnActualized) =
        MatrixXd::Identity(kDimUnActualized, kDimUnActualized);
    T.bottomRightCorner(kDimActualized,kDimActualized) = R_a;

    // b_NG = [zeros(size(N, 1), 1); b_G];
    VectorXd b_NG = VectorXd::Zero(N.rows() + b_G.rows());
    b_NG.tail(b_G.rows()) = b_G;

    // v_star = NG\b_NG;
    VectorXd v_star = NG.fullPivLu().solve(b_NG);
    // cout << "C_best: " << C_best.rows() << ", " << C_best.cols() << endl;
    // cout << C_best;

    w_av = C_best*v_star;
  }

  cout << "Begin Solving for force commands." << endl;
  // unactuated dimensions
  // H = [eye(kDimUnActualized), zeros(kDimUnActualized, kDimActualized)];
  MatrixXd H = MatrixXd::Zero(kDimUnActualized, kDimGeneralized);
  H.leftCols(kDimUnActualized) = MatrixXd::Identity(kDimUnActualized,
      kDimUnActualized);
  MatrixXd T_inv = T.inverse();

  // Newton's laws
  MatrixXd M_newton_H(kDimUnActualized, kDimGeneralized + kDimContactForce);
  if (kDimUnActualized > 0)
    M_newton_H << MatrixXd::Zero(kDimUnActualized, kDimContactForce), H*T_inv;

  MatrixXd M_newton_N(kDimGeneralized, kDimGeneralized + kDimContactForce);
  if (kDimContactForce > 0)
    M_newton_N << T*N_ALL.transpose(),
            MatrixXd::Identity(kDimGeneralized, kDimGeneralized);
  else
    M_newton_N << MatrixXd::Identity(kDimGeneralized, kDimGeneralized);

  MatrixXd M_newton(kDimUnActualized+kDimGeneralized+Aeq.rows(),
      kDimGeneralized + kDimContactForce);
  M_newton << M_newton_H, M_newton_N, Aeq;

  VectorXd b_newton(M_newton.rows());
  b_newton << VectorXd::Zero(H.rows()), -T*F, beq;

  MatrixXd M_free(M_newton.rows(), M_newton.cols() - n_af);
  M_free << M_newton.leftCols(kDimContactForce+kDimUnActualized),
      M_newton.rightCols(n_av);
  MatrixXd M_eta_af = M_newton.middleCols(kDimContactForce+kDimUnActualized,
      n_af);

  // prepare the QP
  // min 0.5 * x G0 x + g0 x
  // s.t.
  //     CE^T x + ce0 = 0
  //     CI^T x + ci0 >= 0
  // variables: [free_force, dual_free_force, eta_af]

  // Cost function
  int n_free = kDimContactForce + kDimUnActualized + n_av;
  int n_dual_free = M_newton.rows();
  Eigen::VectorXd Gdiag = Eigen::VectorXd::Zero(n_free+n_dual_free+n_af);
  for (int i = 0; i < n_af; ++i) Gdiag(n_free+n_dual_free+i) = 1.0;
  for (int i = 0; i < Gdiag.rows(); ++i) Gdiag(i) += 1e-3; // regularization
  Eigen::MatrixXd G0 = Gdiag.asDiagonal();
  Eigen::VectorXd g0 = Eigen::VectorXd::Zero(G0.rows());

  // equality constraints
  // Aeq x = beq
  // qpAeq = [2*eye(n_free), M_free', zeros(n_free, n_af);
  //           M_free, zeros(size(M_free, 1)), M_eta_af];
  // qpbeq = [zeros(n_free, 1); b_newton];
  MatrixXd qpAeq(n_free+M_free.rows(), A.cols() + n_dual_free);
  qpAeq << 2*MatrixXd::Identity(n_free,n_free), M_free.transpose(),
      MatrixXd::Zero(n_free, n_af), M_free,
      MatrixXd::Zero(M_free.rows(), M_free.rows()), M_eta_af;
  VectorXd qpbeq(n_free+b_newton.rows());
  qpbeq << VectorXd::Zero(n_free), b_newton;


  // Inequality constraints
  // Ax<b
  // A_temp = [A(:, 1:kDimContactForce), A(:, kDimContactForce+1:end)*T_inv];
  // A_lambda_eta_u = A_temp(:, 1:kDimContactForce+kDimUnActualized);
  // A_eta_af = A_temp(:, kDimContactForce+kDimUnActualized+1:kDimContactForce+kDimUnActualized+n_af);
  // A_eta_av = A_temp(:, kDimContactForce+kDimUnActualized+n_af+1:end);
  // qp.A = [A_lambda_eta_u A_eta_av zeros(size(A, 1), n_dual_free) A_eta_af];
  MatrixXd qpA;
  if (A.rows() > 0) {
    MatrixXd A_temp(A.rows(), A.cols());
    A_temp << A.leftCols(kDimContactForce), A.rightCols(kDimGeneralized)*T_inv;
    qpA = MatrixXd(A.rows(), A.cols() + n_dual_free);
    qpA << A_temp.leftCols(kDimContactForce+kDimUnActualized),
        A_temp.rightCols(n_av), MatrixXd::Zero(A.rows(), n_dual_free),
        A_temp.middleCols(kDimContactForce+kDimUnActualized, n_af);
  } else {
    qpA = MatrixXd(0, A.cols() + n_dual_free);
  }
  VectorXd qpb = b_A;

  // solve the QP
  Eigen::VectorXd x = Eigen::VectorXd::Random(g0.rows());
  double cost = solve_quadprog(G0, g0,
      qpAeq.transpose().cast<double>(), -qpbeq.cast<double>(),
      -qpA.transpose().cast<double>(), qpb.cast<double>(), x);

  // read the results
  Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");
  cout << "  QP Solved. cost = " << cost << endl;
  // eta_af = x(n_free + n_dual_free + 1:end);
  VectorXd eta_af = x.segment(n_free+n_dual_free, n_af);

  cout << "   Equality violation: " << (qpbeq - qpAeq*x).norm()
      << endl;

  float inequality_violation = 0;
  if (qpA.rows() > 0) {
    VectorXd b_Ax = qpA*x - qpb;
    if (b_Ax.maxCoeff() > 0 ) inequality_violation = b_Ax.maxCoeff();
  }
  cout << "   Inequality violation: " << inequality_violation
      << endl;

  action->n_av   = n_av;
  action->n_af   = n_af;
  action->R_a    = R_a;
  action->w_av   = w_av;
  action->eta_af = eta_af;

  return true;
}