#include "hybrid_servoing_tasks.h"

#include <iostream>
#include <fstream>

#include "solvehfvc.h"
#include "utilities.h"
#include "ZZJHand.h"
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

#define PIf 3.1416f

enum HYBRID_SERVO_MODE {
  HS_STOP_AND_GO,
  HS_CONTINUOUS
};

using std::cout;
using std::endl;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;
using Eigen::Matrix3f;
using Eigen::MatrixXf;


// Smoothly send a hybrid force-velocity action to the forcecontrol module.
// Note this function controls the 6D pose of the robot. For a particular
//    problem, the hfvc action may need to be transformed.
//
// Inputs
//  n_af, n_av, R_a: dimension and transformation of the 3D translation
//  pose_set: 7x1 goal pose (x y z qw qx qy qz), described in world frame.
//  force_set: 6x1 force vector, described in the transformed action space
//  mode: there are two modes:
//    HS_STOP_AND_GO will reset at begining, erase any existing offsets, move
//    from current robot pose to pose_set;
//    HS_CONTINUOUS will not reset at begining,keep existing offsets, move from
//        previous COMMAND (_pose_user_input) to pose_set.
//  running_time_s: time duration of the execution.
bool ExecuteHFVC_ABBEGM(const int n_af, const int n_av,
    const Matrix3f R_a, const float *pose_set, const float *force_set,
    HYBRID_SERVO_MODE mode, const float running_time_s,
    const int main_loop_rate,
    ForceControlHardware *robot, ForceControlController *controller) {
  if (mode == HS_STOP_AND_GO) {
    controller->reset();
  }
  controller->updateAxis(R_a, n_af);
  controller->setForce(force_set);

  float pose[7];
  if (mode == HS_STOP_AND_GO)
    robot->getPose(pose);
  else
    UT::copyArray(controller->_pose_user_input, pose, 7);

  int num_of_steps = int(running_time_s*float(main_loop_rate) + 0.5);

  float **pose_traj = new float*[num_of_steps];
  MotionPlanning(pose, pose_set, num_of_steps, pose_traj);
  // main loop
  ros::Duration period(EGM_PERIOD); // just for being compatible with ROS Control
  ros::Rate pub_rate(main_loop_rate);
  bool b_is_safe = true;
  for (int i = 0; i < num_of_steps; ++i) {
    // cout << "[Hybrid] update step " << i << " of " << num_of_steps;
    // cout << ", pose sent: " << pose_traj[i][0] << ", " << pose_traj[i][1];
    // cout << ", " << pose_traj[i][2] << endl;
    controller->setPose(pose_traj[i]);
    // !! after setPose, must call update before updateAxis
    // so as to set right value for pose_command

    ros::Time time_now = ros::Time::now();
    b_is_safe = controller->update(time_now, period);
    if(!b_is_safe) break;
    pub_rate.sleep();
  }
  for (int i = 0; i < num_of_steps; ++i)    delete [] pose_traj[i];
  delete [] pose_traj;

  return true;
}

ReOrienting::ReOrienting(ForceControlHardware *robot,
    ForceControlController *controller) {
  robot_        = robot;
  controller_   = controller;
  traj_is_read_ = false;
}

ReOrienting::~ReOrienting() {
  if (!rtype_) delete [] rtype_;
  if (!stuck_) delete [] stuck_;
}

bool ReOrienting::initialize(const std::string& file_name,
    const int main_loop_rate) {
  // --------------------------------------------------------
  // Read trajectory files
  // --------------------------------------------------------
  ROS_INFO("Reading Trajectory files..\n");

  ifstream f_para;
  f_para.open(file_name + "/para.txt");
  if (!f_para) {
    cerr << "Unable to open file for f_para'.";
    return false; // terminate with error
  }
  f_para >> N_TRJ_;
  cout << "N_TRJ: " << N_TRJ_ << endl;
  f_para >> grp_width_;
  f_para >> finger_radius_;
  f_para >> finger_thickness_;
  f_para >> open_finger_incre_for_pivot_mm_;
  f_para >> safe_Z_min_;
  f_para.close();

  // trajectories
  ifstream f_rtype, f_stuck, f_q_WG, f_p_WG;
  f_rtype.open(file_name + "/rtype.txt");
  f_stuck.open(file_name + "/stuck.txt");
  f_q_WG.open(file_name + "/q_WG.txt");
  f_p_WG.open(file_name + "/p_WG.txt");

  if (!f_rtype) {
    cerr << "Unable to open file for f_rtype'.";
    return false; // terminate with error
  }
  if (!f_stuck) {
    cerr << "Unable to open file for f_stuck'.";
    return false; // terminate with error
  }
  if (!f_q_WG) {
    cerr << "Unable to open file for f_q_WG'.";
    return false; // terminate with error
  }
  if (!f_p_WG) {
    cerr << "Unable to open file for p_WG'.";
    return false; // terminate with error
  }


  rtype_ = new bool[N_TRJ_];
  stuck_ = new bool[N_TRJ_];
  q_WG_.resize(4, N_TRJ_);
  p_WG_.resize(3, N_TRJ_);

  for (int i = 0; i < N_TRJ_; ++i) {
    f_rtype >> rtype_[i];
    f_stuck >> stuck_[i];
    f_q_WG >> q_WG_(0, i) >> q_WG_(1, i) >> q_WG_(2, i) >> q_WG_(3, i);
    f_p_WG >> p_WG_(0, i) >> p_WG_(1, i) >> p_WG_(2, i);

    // cout << i << " rtype: " << rtype[i] << " stuck: " << stuck[i];
    // cout << " qgrp: " << qgrp(0, i) << " " << qgrp(1, i) << " " << qgrp(2, i) << " " << qgrp(3, i);
    // cout << " grpz: " << grpz[i] << endl;
  }
  traj_is_read_ = true;
  main_loop_rate_ = main_loop_rate;

  f_rtype.close();
  f_stuck.close();
  f_q_WG.close();
  f_p_WG.close();
  return true;
}

bool ReOrienting::run() {
  if (!traj_is_read_) {
    ROS_ERROR("Error: trajectory is not read. Call initialize first.");
    return false;
  }
  traj_is_read_ = false;

  // grasp
  ZZJHand *hand = ZZJHand::Instance();
  if(hand->GraspFirmly() != MMC_SUCCESS) {
    hand->closeEpos();
    ROS_ERROR("Calling hand->GraspFirmly() failed.");
    return false;
  }

  // --------------------------------------------------------
  //  Main loop
  // --------------------------------------------------------
  ros::Duration period(EGM_PERIOD);
  ros::Rate pub_rate(main_loop_rate_);

  bool is_stuck = false;
  float pose_set[7], pose[7], force_set[6];
  robot_->getPose(pose);
  Vector3f xyz_delta;
  int mode = -1;
  controller_->reset();

  for (int fr = 1; fr < N_TRJ_; ++fr)  {
      /* ********************************************
       * There are three possible modes:
       *  1. rtype==0 (firm grasp) -- Hybrid servoing
       *  2. rtype==1, sticking == 0 -- velocity servoing
       *  3. rtype==1, sticking == 1 -- Hybrid servoing
      */
      int mode_new = 0;
      if (rtype_[fr] == false) mode_new = 1;
      else if(stuck_[fr] == false) mode_new = 2;
      else mode_new = 3;

      // Control the gripper
      if ( mode_new != mode ) {
        cout << "[New mode] Mode = " << mode_new << endl;
        // cout << "[New mode] Mode = " << mode_new << ", Press Enter to start..";
        // getchar();

        if (mode_new == 1) {
          // Switched to firm grasp
          if(hand->GraspFirmly() != MMC_SUCCESS) {
            ROS_ERROR("Failed to call hand->GraspFirmly().");
            hand->closeEpos();
            return -1;
          }
        }
        else {
          // Switched to Pivoting
          if(hand->Pivoting(open_finger_incre_for_pivot_mm_) != MMC_SUCCESS) {
            ROS_ERROR("Failed to call hand->Pivoting().");
            hand->closeEpos();
            return false;
          }
        }
      }

      // control the robot
      pose_set[0] = p_WG_(0, fr);
      pose_set[1] = p_WG_(1, fr);
      pose_set[2] = p_WG_(2, fr);
      pose_set[3] = q_WG_(0, fr);
      pose_set[4] = q_WG_(1, fr);
      pose_set[5] = q_WG_(2, fr);
      pose_set[6] = q_WG_(3, fr);

      // check collision with table
      Quaternionf qtemp;
      qtemp.w() = pose_set[3];
      qtemp.x() = pose_set[4];
      qtemp.y() = pose_set[5];
      qtemp.z() = pose_set[6];
      Vector3f grp_z  = qtemp._transformVector(Vector3f(0.0f, 0.0f, 1.0f));
      Vector3f grp_x  = qtemp._transformVector(Vector3f(1.0f, 0.0f, 0.0f));
      Vector3f grp_ax = Eigen::AngleAxis<float>(-22.5f*PI/180.0f, grp_z)._transformVector(grp_x);
      Vector3f grp(pose_set[0], pose_set[1], pose_set[2]);
      Vector3f gp1_bottom = grp + grp_ax*(grp_width_/2.0f + finger_thickness_) + grp_z*finger_radius_;
      Vector3f gp2_bottom = grp - grp_ax*(grp_width_/2.0f + finger_thickness_) + grp_z*finger_radius_;
      if ((gp1_bottom(2) < safe_Z_min_) || (gp2_bottom(2) < safe_Z_min_)) {
          cout << "The Z command is below safety limit. Gripper would hit table.";
          return false;
      }

      switch (mode_new) {
        case 1: {
          // Hybrid servoing
        }
        case 2: {
          // pure velocity control
          UT::setArray(force_set, 0, 6);
          controller_->updateAxis(Matrix3f::Identity(), 0);
          controller_->setForce(force_set);
          controller_->setPose(pose_set);
        }
        case 3: {
          // Hybrid servoing
        }
        default: {
          ROS_ERROR("Wrong mode_new: %d\n", mode_new);
          return false;
        }
      }

      mode = mode_new;

      ros::Time time_now = ros::Time::now();
      controller_->update(time_now, period);
      pub_rate.sleep();
  }

  // // write the current robot pose to file
  // robot.getPose(pose);
  // ofstream f_grp_pose;
  // f_grp_pose.open(GRP_POSE_FILE_PATH);
  // for (int i = 0; i < 7; ++i)
  //     f_grp_pose << pose[i] << " ";
  // f_grp_pose.close();


  ROS_INFO("[Regrasping] Run is done.");
  return true;
}


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

  float kObjectMass, kHandMass, kGravityConstant, kObjectEdgeLength;
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

  // cout << "\np_WH_traj_: " << endl;
  // for (int i = 0; i < N_TRJ_; ++i)
  //   cout << p_WH_traj_(0, i) << "|" << p_WH_traj_(1, i) << "|" <<
  //       p_WH_traj_(2, i) << endl;

  // cout << "\np_WO_traj_: " << endl;
  // for (int i = 0; i < N_TRJ_; ++i)
  //   cout << p_WO_traj_(0, i) << "|" << p_WO_traj_(1, i) << "|" <<
  //       p_WO_traj_(2, i) << endl;

  // cout << "\nq_WO_traj_: " << endl;
  // for (int i = 0; i < N_TRJ_; ++i)
  //   cout << q_WO_traj_(0, i) << "|" << q_WO_traj_(1, i) << "|" <<
  //       q_WO_traj_(2, i) << q_WO_traj_(3, i) << endl;

  // cout << "Press Enter to continue...";
  // getchar();

  Vector3f p_LineContact, kRotateAxis;
  p_LineContact = p_WO0;
  kRotateAxis   = Vector3f::UnitZ().cross(kTiltDirection);

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
  MatrixXf G(6, 9);
  G << MatrixXf::Identity(6, 6),
      MatrixXf::Zero(6, 3);

  int dimAx = kFrictionConeSides_*(1+2)+1;
  int dimAy = 3*(1+2)+9;

  VectorXf b_A(dimAx);
  b_A = VectorXf::Zero(dimAx);
  b_A(dimAx-1) = -kMinNormalForce_;

  controller_->reset();

  Timer time;
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    time.tic();
    Vector3f p_WH, p_WO;
    Vector4f q_WO;
    p_WH = p_WH_traj_.col(fr);
    p_WO = p_WO_traj_.col(fr);
    q_WO = q_WO_traj_.col(fr);

    // Omega
    Matrix3f R_WO;
    R_WO = quat2m(Quaternionf(q_WO(0), q_WO(1), q_WO(2), q_WO(3)));
    MatrixXf E_qO(4, 3);
    E_qO << -q_WO(1), -q_WO(2), -q_WO(3),
            q_WO(0), -q_WO(3), q_WO(2),
            q_WO(3), q_WO(0), -q_WO(1),
            -q_WO(2), q_WO(1), q_WO(0);
    E_qO = 0.5*E_qO;
    MatrixXf Omega(10, 9);
    // Omega = blkdiag(R_WO, E_qO, eye(3));
    Omega = Eigen::Matrix<float, 10, 9>::Zero();
    Omega.block<3,3>(0,0) = R_WO;
    Omega.block<4,3>(3,3) = E_qO;
    Omega.block<3,3>(7,6) = Matrix3f::Identity();

    // Goal Description
    MatrixXf Adj_g_WO_inv(6, 6);
    // Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];
    Adj_g_WO_inv << R_WO.transpose(), -R_WO.transpose()*wedge(p_WO),
        Matrix3f::Zero(), R_WO.transpose();

    VectorXf t_OG(6), b_G(6);
    t_OG = Adj_g_WO_inv*t_WG_;
    b_G = t_OG;

    // external force
    VectorXf F(9);
    // F = R_WO'*F_WGO; zeros(3,1); F_WGH];
    F = VectorXf::Zero(9);
    F.head(3) = R_WO.transpose()*F_WGO_;
    F.tail(3) = F_WGH_;

    // Guard conditions
    //   hand contact is sticking; (force is in world frame)
    //   table contacts are sticking;
    //   hand contact normal force lower bound
    MatrixXf A(dimAx, dimAy);
    A = MatrixXf::Zero(dimAx, dimAy);
    MatrixXf zarray(1, 3);
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

    float J_array[90];
    jac_phi_q_block_tilting(p_WO.data(), q_WO.data(), p_WH.data(),
        p_OHC_.data(), p_WTC_all_.data(), p_OTC_all_.data(),
        J_array);

    // read the jacobian from results
    MatrixXf Jac_phi_q(9, 10);
    for(int i = 0; i<9; i++)
      for(int j = 0; j<10; j++)
        Jac_phi_q(i,j) = J_array[j*9 + i];

    // [n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, Jac_phi_q, ...
    //         G, b_G, F, [], [], A, b_A, dims, 'num_seeds', 1);
    HFVC action;
    MatrixXf Aeq(1, 1); // dummy
    VectorXf beq(1); // dummy
    int kDimActualized = 3;
    int kDimUnActualized = 6;
    int kDimSlidingFriction = 0;
    int kDimLambda = 3*(1+2);
    int kNumSeeds = 5;
    solvehfvc(Omega, Jac_phi_q, G, b_G, F, Aeq, beq,
      A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds,
      &action);

    double computation_time_ms = time.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;
    /*  Execute the hybrid action */
    float pose_set[7];
    robot_->getPose(pose_set); // get quaternion
    for (int i = 0; i < 3; ++i) pose_set[i] = 1000.0f * p_WH(i); // mm

    float force_set[6];
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
    // cout << "Press Enter to begin motion!" << endl;
    // getchar();
    cout << "motion begins:" << endl;
    ExecuteHFVC_ABBEGM(action.n_af, action.n_av,
        action.R_a, pose_set, force_set,
        HS_CONTINUOUS, kTimeStepSec_, main_loop_rate_,
        robot_, controller_);
  } // end for

}


// LeveringUp::LeveringUp(ForceControlHardware *robot,
//         ForceControlController *controller) {
//   robot_ = robot;
//   controller_ = controller;
// }

// LeveringUp::~LeveringUp() {

// }


// bool LeveringUp::initialize(const std::string& file_name,
//     const int main_loop_rate) {
//   main_loop_rate_ = main_loop_rate;
//   // read parameter file
//   ifstream fp;
//   fp.open(file_name + "block_tilting/para.txt");
//   if (!fp) {
//     cerr << "Unable to open parameter file.";
//     return false;
//   }

//   float kObjectMass, kHandMass, kGravityConstant, kObjectEdgeLength;
//   Vector3f p_WH0, p_WO0, kTiltDirection;

//   fp >> N_TRJ_ >> kTimeStepSec_ >> kObjectMass >> kHandMass >> kGravityConstant
//       >> kFrictionCoefficientTable_ >> kFrictionCoefficientHand_ >>
//       kFrictionConeSides_ >> kMinNormalForce_ >> kObjectEdgeLength >>
//       p_WH0(0) >> p_WH0(1) >> p_WH0(2) >>
//       p_WO0(0) >> p_WO0(1) >> p_WO0(2) >>
//       kTiltDirection(0) >> kTiltDirection(1) >> kTiltDirection(2);
//   fp.close();

//   // double check
//   cout << "N_TRJ_: " << N_TRJ_ << endl;
//   cout << "kTimeStepSec_: " << kTimeStepSec_ << endl;
//   cout << "kObjectMass: " << kObjectMass << endl;
//   cout << "kHandMass: " << kHandMass << endl;
//   cout << "kGravityConstant: " << kGravityConstant << endl;
//   cout << "kFrictionCoefficientTable_: " << kFrictionCoefficientTable_ << endl;
//   cout << "kFrictionCoefficientHand_: " << kFrictionCoefficientHand_ << endl;
//   cout << "kFrictionConeSides_: " << kFrictionConeSides_ << endl;
//   cout << "kMinNormalForce_: " << kMinNormalForce_ << endl;
//   cout << "kObjectEdgeLength: " << kObjectEdgeLength << endl;

//   ifstream fp_p_WH, fp_p_WO, fp_q_WO;
//   fp_p_WH.open(file_name + "block_tilting/p_WH.txt");
//   fp_p_WO.open(file_name + "block_tilting/p_WO.txt");
//   fp_q_WO.open(file_name + "block_tilting/q_WO.txt");
//   if ((!fp_p_WO)||(!fp_p_WH)||(!fp_q_WO)) {
//     cerr << "Unable to open trajectory file.";
//     exit(1); // terminate with error
//   }

//   p_WH_traj_.resize(3, N_TRJ_);
//   p_WO_traj_.resize(3, N_TRJ_);
//   q_WO_traj_.resize(4, N_TRJ_);

//   for (int i = 0; i < N_TRJ_; ++i) {
//     fp_p_WH >> p_WH_traj_(0, i) >> p_WH_traj_(1, i) >> p_WH_traj_(2, i);
//     fp_p_WO >> p_WO_traj_(0, i) >> p_WO_traj_(1, i) >> p_WO_traj_(2, i);
//     fp_q_WO >> q_WO_traj_(0, i) >> q_WO_traj_(1, i) >> q_WO_traj_(2, i)
//         >> q_WO_traj_(3, i);
//   }
//   fp_p_WH.close();
//   fp_p_WO.close();
//   fp_q_WO.close();

//   // cout << "\np_WH_traj_: " << endl;
//   // for (int i = 0; i < N_TRJ_; ++i)
//   //   cout << p_WH_traj_(0, i) << "|" << p_WH_traj_(1, i) << "|" <<
//   //       p_WH_traj_(2, i) << endl;

//   // cout << "\np_WO_traj_: " << endl;
//   // for (int i = 0; i < N_TRJ_; ++i)
//   //   cout << p_WO_traj_(0, i) << "|" << p_WO_traj_(1, i) << "|" <<
//   //       p_WO_traj_(2, i) << endl;

//   // cout << "\nq_WO_traj_: " << endl;
//   // for (int i = 0; i < N_TRJ_; ++i)
//   //   cout << q_WO_traj_(0, i) << "|" << q_WO_traj_(1, i) << "|" <<
//   //       q_WO_traj_(2, i) << q_WO_traj_(3, i) << endl;

//   // cout << "Press Enter to continue...";
//   // getchar();

//   Vector3f p_LineContact, kRotateAxis;
//   p_LineContact = p_WO0;
//   kRotateAxis   = Vector3f::UnitZ().cross(kTiltDirection);

//   // goal obj twist in world frame
//   t_WG_.resize(6);
//   float kGoalVelocity = 1; // shouldn't matter
//   t_WG_.head(3) = -kRotateAxis.cross(p_LineContact);
//   t_WG_.tail(3) = kRotateAxis;
//   t_WG_ = t_WG_*kGoalVelocity;


//   p_OHC_ = p_WH0 - p_WO0;


//   v_friction_directions_.resize(3, kFrictionConeSides_);
//   for (int i = 0; i < kFrictionConeSides_; ++i) {
//     v_friction_directions_(0, i) = sin(2.0f*PIf*float(i)/float(kFrictionConeSides_));
//     v_friction_directions_(1, i) = cos(2.0f*PIf*float(i)/float(kFrictionConeSides_));
//     v_friction_directions_(2, i) = 0.0f;
//   }

//   // contact point with table
//   p_OTC_all_.resize(3, 2);
//   p_WTC_all_.resize(3, 2);
//   p_OTC_all_.col(0) = kObjectEdgeLength/2.0f*kRotateAxis;
//   p_OTC_all_.col(1) = -kObjectEdgeLength/2.0f*kRotateAxis;
//   p_WTC_all_.col(0) = p_WO0 + kObjectEdgeLength/2.0f*kRotateAxis;
//   p_WTC_all_.col(1) = p_WO0 - kObjectEdgeLength/2.0f*kRotateAxis;

//   F_WGO_ << 0, 0, -kObjectMass*kGravityConstant;
//   F_WGH_ << 0, 0, -kHandMass*kGravityConstant;

//   return true;
// }

// bool LeveringUp::run() {

// }