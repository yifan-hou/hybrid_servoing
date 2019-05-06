#include <apps/plane_engaging.h>

#include <Eigen/SVD>

#include "solvehfvc.h"
#include "utilities.h"
#include "yifanlibrary/TimerLinux.h"

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");

PlaneEngaging::PlaneEngaging(ForceControlHardware *robot,
        ForceControlController *controller) {
  robot_ = robot;
  controller_ = controller;
}

PlaneEngaging::~PlaneEngaging() {
}

bool PlaneEngaging::initialize(const std::string& file_name,
    const int main_loop_rate) {
  main_loop_rate_ = main_loop_rate;

  // parameters
  //  G, b_G
  ifstream fp;
  fp.open(file_name + "plane_engaging/para.txt");
  if (!fp) {
    cerr << "Unable to open parameter file.";
    return false;
  }

  int nRowG, nColG;

  fp >> nRowG >> nColG;
  G_.resize(nRowG, nColG);
  b_G_.resize(nRowG);
  for (int i = 0; i < nRowG; ++i)
    for (int j = 0; j < nColG; ++j)
      fp >> G_(i,j);

  for (int i = 0; i < nRowG; ++i) fp >> b_G_(i);

  fp.close();

  // double check
  cout << "G_: " << G_.format(MatlabFmt) << endl;
  cout << "b_G_: " << b_G_.format(MatlabFmt) << endl;

  return true;
}

bool PlaneEngaging::run() {
  MatrixXd f_data, v_data;
  f_data = MatrixXd::Zero(6, controller_->pool_size_);
  v_data = MatrixXd::Zero(6, controller_->pool_size_);

  controller_->reset();
  Timer time;
  int N_TRJ_ = 1000;
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    time.tic();
    /* Get new measurements */
    // already done in controller.update()

    /* Update weights of data */

    /* Estimate Natural Constraints from pool of data */
    for (int i = 0; i < controller_->f_queue_.size(); ++i) {
      f_data.col(i) = controller_->f_queue_[i];
      v_data.col(i) = controller_->v_queue_[i];
    }
    JacobiSVD<MatrixXd> svd_f(f_data, ComputeThinV);
    JacobiSVD<MatrixXd> svd_v(v_data, ComputeFullV);
    MatrixXd V_f = svd_f.computeV();
    MatrixXd V_v = svd_v.computeV();

    svd_f.singularValues();
    svd_v.singularValues();




    controller_->f_queue_[i]
    Vector3d p_WH, p_WO;
    Vector4d q_WO;
    p_WH = p_WH_traj_.col(fr);
    p_WO = p_WO_traj_.col(fr);
    q_WO = q_WO_traj_.col(fr);

    // Omega
    Matrix3d R_WO;
    R_WO = quat2m(Quaterniond(q_WO(0), q_WO(1), q_WO(2), q_WO(3)));
    MatrixXd E_qO(4, 3);
    E_qO << -q_WO(1), -q_WO(2), -q_WO(3),
            q_WO(0), -q_WO(3), q_WO(2),
            q_WO(3), q_WO(0), -q_WO(1),
            -q_WO(2), q_WO(1), q_WO(0);
    E_qO = 0.5*E_qO;
    MatrixXd Omega(10, 9);
    // Omega = blkdiag(R_WO, E_qO, eye(3));
    Omega = Eigen::Matrix<double, 10, 9>::Zero();
    Omega.block<3,3>(0,0) = R_WO;
    Omega.block<4,3>(3,3) = E_qO;
    Omega.block<3,3>(7,6) = Matrix3d::Identity();

    // Goal Description
    MatrixXd Adj_g_WO_inv(6, 6);
    // Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];
    Adj_g_WO_inv << R_WO.transpose(), -R_WO.transpose()*wedge(p_WO),
        Matrix3f::Zero(), R_WO.transpose();

    VectorXd t_OG(6), b_G(6);
    t_OG = Adj_g_WO_inv*t_WG_;
    b_G = t_OG;

    // external force
    VectorXd F(9);
    // F = R_WO'*F_WGO; zeros(3,1); F_WGH];
    F = VectorXd::Zero(9);
    F.head(3) = R_WO.transpose()*F_WGO_;
    F.tail(3) = F_WGH_;

    // Guard conditions
    //   hand contact is sticking; (force is in world frame)
    //   table contacts are sticking;
    //   hand contact normal force lower bound
    MatrixXd A(dimAx, dimAy);
    A = MatrixXd::Zero(dimAx, dimAy);
    MatrixXd zarray(1, 3);
    zarray << 0, 0, 1;
    for (int i = 0; i < kFrictionConeSides_; ++i) {
      A.block<1, 3>(i, 0) = (v_friction_directions_.col(i).transpose() -
          kFrictionCoefficientHand_*zarray)*R_WO.transpose();
      A.block<1, 3>(kFrictionConeSides_+i, 3) =
          v_friction_directions_.col(i).transpose() -
          kFrictionCoefficientTable_*zarray;
      A.block<1, 3>(2*kFrictionConeSides_, 6) =
          v_friction_directions_.col(i).transpose() -
          kFrictionCoefficientTable_*zarray;
    }
    A.block<1, 3>(3*kFrictionConeSides_, 0) = -zarray*R_WO.transpose();

    double J_array[90];
    jac_phi_q_block_tilting(p_WO.data(), q_WO.data(), p_WH.data(),
        p_OHC_.data(), p_WTC_all_.data(), p_OTC_all_.data(),
        J_array);

    // read the jacobian from results
    MatrixXd Jac_phi_q(9, 10);
    for(int i = 0; i<9; i++)
      for(int j = 0; j<10; j++)
        Jac_phi_q(i,j) = J_array[j*9 + i];

    // [n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, Jac_phi_q, ...
    //         G, b_G, F, [], [], A, b_A, dims, 'num_seeds', 1);
    HFVC action;
    MatrixXd Aeq(1, 1); // dummy
    VectorXd beq(1); // dummy
    int kDimActualized      = 3;
    int kDimUnActualized    = 6;
    int kDimSlidingFriction = 0;
    int kDimLambda          = 3*(1+2);
    int kNumSeeds           = 3;
    solvehfvc(Omega, Jac_phi_q, G, b_G, F, Aeq, beq,
      A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds,
      &action);

    double computation_time_ms = time.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;
    /*  Execute the hybrid action */
    double pose_set[7];
    robot_->getPose(pose_set); // get quaternion
    for (int i = 0; i < 3; ++i) pose_set[i] = 1000.0 * p_WH(i); // mm

    double force_set[6] = {0};
    for (int i = 0; i < action.n_af; ++i)  force_set[i] = action.eta_af(i);

    cout << "\nSolved. Solution Action:" << endl;
    cout << "n_af: " << action.n_af << endl;
    cout << "n_av: " << action.n_av << endl;
    // cout << "R_a: " << action.R_a << endl;
    // cout << "pose_set: " << pose_set[0] << "|"
    //                      << pose_set[1] << "|"
    //                      << pose_set[2] << " || "
    //                      << pose_set[3] << "|"
    //                      << pose_set[4] << "|"
    //                      << pose_set[5] << "|"
    //                      << pose_set[6] << endl;
    cout << "force_set: " << force_set[0] << "|"
                          << force_set[1] << "|"
                          << force_set[2] << " || "
                          << force_set[3] << "|"
                          << force_set[4] << "|"
                          << force_set[5] << endl;
    cout << "running time: " << kTimeStepSec_ << endl;
    cout << "Press Enter to begin motion!" << endl;
    // getchar();
    // cout << "motion begins:" << endl;
    ExecuteHFVC_ABBEGM(action.n_af, action.n_av,
        action.R_a, pose_set, force_set,
        HS_CONTINUOUS, kTimeStepSec_, main_loop_rate_,
        robot_, controller_);
  } // end for

}
