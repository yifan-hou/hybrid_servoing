#include <apps/block_tilting.h>

#include "solvehfvc.h"
#include "utilities.h"
#include "yifanlibrary/TimerLinux.h"

// Matlab CodeGen files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "jac_phi_q_block_tilting.h"
#include "jac_phi_q_block_tilting_types.h"
#include "jac_phi_q_block_tilting_terminate.h"
#include "jac_phi_q_block_tilting_initialize.h"


BlockTilting::BlockTilting(ForceControlHardware *robot,
        ForceControlController *controller) {
  robot_ = robot;
  controller_ = controller;
}

BlockTilting::~BlockTilting() {
}

bool BlockTilting::initialize(const std::string& file_name,
    const int main_loop_rate) {
  main_loop_rate_ = main_loop_rate;
  // read parameter file
  ifstream fp;
  fp.open(file_name + "block_tilting/para.txt");
  if (!fp) {
    cerr << "Unable to open parameter file.";
    return false;
  }

  double kObjectMass, kHandMass, kGravityConstant, kObjectEdgeLength;
  Vector3f p_WH0, p_WO0, kTiltDirection;

  fp >> N_TRJ_ >> kTimeStepSec_ >> kObjectMass >> kHandMass >> kGravityConstant
      >> kFrictionCoefficientTable_ >> kFrictionCoefficientHand_ >>
      kFrictionConeSides_ >> kMinNormalForce_ >> kObjectEdgeLength >>
      p_WH0(0) >> p_WH0(1) >> p_WH0(2) >>
      p_WO0(0) >> p_WO0(1) >> p_WO0(2) >>
      kTiltDirection(0) >> kTiltDirection(1) >> kTiltDirection(2);
  fp.close();

  // double check
  cout << "N_TRJ_: " << N_TRJ_ << endl;
  cout << "kTimeStepSec_: " << kTimeStepSec_ << endl;
  cout << "kObjectMass: " << kObjectMass << endl;
  cout << "kHandMass: " << kHandMass << endl;
  cout << "kGravityConstant: " << kGravityConstant << endl;
  cout << "kFrictionCoefficientTable_: " << kFrictionCoefficientTable_ << endl;
  cout << "kFrictionCoefficientHand_: " << kFrictionCoefficientHand_ << endl;
  cout << "kFrictionConeSides_: " << kFrictionConeSides_ << endl;
  cout << "kMinNormalForce_: " << kMinNormalForce_ << endl;
  cout << "kObjectEdgeLength: " << kObjectEdgeLength << endl;

  ifstream fp_p_WH, fp_p_WO, fp_q_WO;
  fp_p_WH.open(file_name + "block_tilting/p_WH.txt");
  fp_p_WO.open(file_name + "block_tilting/p_WO.txt");
  fp_q_WO.open(file_name + "block_tilting/q_WO.txt");
  if ((!fp_p_WO)||(!fp_p_WH)||(!fp_q_WO)) {
    cerr << "Unable to open trajectory file.";
    exit(1); // terminate with error
  }

  p_WH_traj_.resize(3, N_TRJ_);
  p_WO_traj_.resize(3, N_TRJ_);
  q_WO_traj_.resize(4, N_TRJ_);

  for (int i = 0; i < N_TRJ_; ++i) {
    fp_p_WH >> p_WH_traj_(0, i) >> p_WH_traj_(1, i) >> p_WH_traj_(2, i);
    fp_p_WO >> p_WO_traj_(0, i) >> p_WO_traj_(1, i) >> p_WO_traj_(2, i);
    fp_q_WO >> q_WO_traj_(0, i) >> q_WO_traj_(1, i) >> q_WO_traj_(2, i)
        >> q_WO_traj_(3, i);
  }
  fp_p_WH.close();
  fp_p_WO.close();
  fp_q_WO.close();

  Vector3d p_LineContact, kRotateAxis;
  p_LineContact = p_WO0;
  kRotateAxis   = Vector3d::UnitZ().cross(kTiltDirection);

  // goal obj twist in world frame
  t_WG_.resize(6);
  float kGoalVelocity = 1; // shouldn't matter
  t_WG_.head(3) = -kRotateAxis.cross(p_LineContact);
  t_WG_.tail(3) = kRotateAxis;
  t_WG_ = t_WG_*kGoalVelocity;

  p_OHC_ = p_WH0 - p_WO0;

  v_friction_directions_.resize(3, kFrictionConeSides_);
  for (int i = 0; i < kFrictionConeSides_; ++i) {
    v_friction_directions_(0, i) = sin(2.0f*PIf*float(i)/float(kFrictionConeSides_));
    v_friction_directions_(1, i) = cos(2.0f*PIf*float(i)/float(kFrictionConeSides_));
    v_friction_directions_(2, i) = 0.0f;
  }

  // contact point with table
  p_OTC_all_.resize(3, 2);
  p_WTC_all_.resize(3, 2);
  p_OTC_all_.col(0) = kObjectEdgeLength/2.0f*kRotateAxis;
  p_OTC_all_.col(1) = -kObjectEdgeLength/2.0f*kRotateAxis;
  p_WTC_all_.col(0) = p_WO0 + kObjectEdgeLength/2.0f*kRotateAxis;
  p_WTC_all_.col(1) = p_WO0 - kObjectEdgeLength/2.0f*kRotateAxis;

  F_WGO_ << 0, 0, -kObjectMass*kGravityConstant;
  F_WGH_ << 0, 0, -kHandMass*kGravityConstant;

  return true;
}

bool BlockTilting::run() {
  /* constants */

  // G = [eye(6) zeros(6, 3)];
  MatrixXd G(6, 9);
  G << MatrixXd::Identity(6, 6),
      MatrixXd::Zero(6, 3);

  int dimAx = kFrictionConeSides_*(1+2)+1;
  int dimAy = 3*(1+2)+9;

  VectorXd b_A(dimAx);
  b_A = VectorXd::Zero(dimAx);
  b_A(dimAx-1) = -kMinNormalForce_;

  controller_->reset();

  Timer time;
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    time.tic();
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
