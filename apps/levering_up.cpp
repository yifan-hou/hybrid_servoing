#include <apps/levering_up.h>

#include "solvehfvc.h"
#include "utilities.h"
#include "yifanlibrary/TimerLinux.h"

// Matlab CodeGen files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "jac_phi_q_flip_against_corner.h"
#include "jac_phi_q_flip_against_corner_types.h"
#include "jac_phi_q_flip_against_corner_terminate.h"
#include "jac_phi_q_flip_against_corner_initialize.h"



LeveringUp::LeveringUp(ForceControlHardware *robot,
        ForceControlController *controller) {
  robot_ = robot;
  controller_ = controller;
}

LeveringUp::~LeveringUp() {

}


bool LeveringUp::initialize(const std::string& file_name,
    const int main_loop_rate) {
  main_loop_rate_ = main_loop_rate;
  // read parameter file
  ifstream fp;
  fp.open(file_name + "flip_against_corner/para.txt");
  if (!fp) {
    cerr << "Unable to open parameter file.";
    return false;
  }

  fp >> kGoalTheta_ >> kTimeStepSec_ >> kObjectMass_ >> kHandMass_ >>
      kGravityConstant_ >> kObjectLength_ >> kObjectThickness_ >>
      kFrictionCoefficientTable_ >> kFrictionCoefficientHand_ >>
      kFrictionCoefficientBin_ >> kMinNormalForce_ >> kMinNormalForceSliding_
      >> kMaxNormalForceSliding_ >> kGoalRotationVelocity_ >>
      p_WH0_(0) >> p_WH0_(1) >> p_WO0_(0) >> p_WO0_(1);
  fp.close();

  // double check
  cout << "kGoalTheta_: " << kGoalTheta_ << endl;
  cout << "kTimeStepSec: " << kTimeStepSec_ << endl;
  cout << "kObjectMass: " << kObjectMass_ << endl;
  cout << "kHandMass: " << kHandMass_ << endl;
  cout << "kGravityConstant: " << kGravityConstant_ << endl;
  cout << "kObjectLength: " << kObjectLength_ << endl;
  cout << "kObjectThickness: " << kObjectThickness_ << endl;
  cout << "kFrictionCoefficientTable_: " << kFrictionCoefficientTable_ << endl;
  cout << "kFrictionCoefficientHand_: " << kFrictionCoefficientHand_ << endl;
  cout << "kFrictionCoefficientBin_: " << kFrictionCoefficientBin_ << endl;
  cout << "kMinNormalForce_: " << kMinNormalForce_ << endl;
  cout << "kMinNormalForceSliding_: " << kMinNormalForceSliding_ << endl;
  cout << "kMaxNormalForceSliding_: " << kMaxNormalForceSliding_ << endl;
  cout << "kGoalRotationVelocity_: " << kGoalRotationVelocity_ << endl;
  cout << "p_WH0_: " << p_WH0_(0) << ", " << p_WH0_(1) << endl;
  cout << "p_WO0_: " << p_WO0_(0) << ", " << p_WO0_(1) << endl;

  return true;
}

bool LeveringUp::run() {
  // dimensions
  int kDimActualized      = 2;
  int kDimUnActualized    = 3;
  int kDimSlidingFriction = 2;
  int kDimLambda          = 4;
  int kNumSeeds           = 2;
  int kDimGeneralized = kDimActualized + kDimUnActualized;

  float a = p_WH0_(1) - p_WO0_(1) + kObjectThickness_*0.5f;
  float b = kObjectLength_;
  float l_diagonal = std::sqrt(kObjectThickness_*kObjectThickness_ +
      kObjectLength_*kObjectLength_);
  float angle_inner_sharp = std::asin(kObjectThickness_/l_diagonal);
  Vector2f p_OHC = p_WH0_ - p_WO0_;
  Vector2f p_OTC, p_OBC;
  p_OTC << -kObjectLength_*0.5, -kObjectThickness_*0.5;
  p_OBC << -kObjectLength_*0.5, kObjectThickness_*0.5;
  MatrixXd Omega = MatrixXd::Identity(5, 5);
  VectorXd F(5);
  F << 0, -kObjectMass_*kGravityConstant_, 0, 0, -kHandMass_*kGravityConstant_;

  controller_->reset();

  Timer time;
  for (int fr = 0; fr < 1000; ++fr) {
    time.tic();

    // compute object pose
    //   Ideally we should use perception; here we hack it by assuming the
    //   contact between the hand and the object is sticking, and solve for the
    //   object pose from hand pose

    // feedback
    float pose[7];
    robot_->getPose(pose); // get quaternion
    Vector2f p_WH;
    p_WH(0) = pose[1]*0.001f; // m
    p_WH(1) = pose[2]*0.001f;

    // 1. Solve for theta
    //   This is a a*sin(theta)+bsin(theta)=c problem
    float c = p_WH(1) - p_WO0_(1) + kObjectThickness_*0.5;
    float phi = std::atan2(a, b);
    float theta_plus_phi = std::asin(c/(std::sqrt(a*a+b*b)));
    float theta = theta_plus_phi - phi;
    cout << "a = " << a << ", b = " << b << ", theta = " << theta*180.0f/PIf
        << " degree" << endl;
    if (theta > kGoalTheta_) {
      cout << "Goal theta achieved. Exiting.." << endl;
      break;
    }
    assert(theta >= -5.0f*PIf/180.f);
    assert(theta <= 90.0f*PIf/180.f);

    // 2. solve for p_WO
    Vector2f p_temp, p_WO;
    p_temp <<
        kObjectThickness_*std::sin(theta)+l_diagonal*0.5*cos(theta+angle_inner_sharp),
        l_diagonal*0.5*std::sin(theta+angle_inner_sharp);
    p_WO = p_WO0_;
    p_WO(0) -= kObjectLength_*0.5;
    p_WO(1) -= kObjectThickness_*0.5;
    p_WO    += p_temp;

    Matrix3f R_WO3 = aa2mat(theta, Vector3f::UnitX());
    Matrix2f R_WO = R_WO3.block<2,2>(1,1);


    // goal
    float goal_velocity_z = kGoalRotationVelocity_*kObjectLength_*std::cos(theta);
    MatrixXd G(1, 5);
    G << 0, 1, 0, 0, 0;
    VectorXd b_G(1);
    b_G << goal_velocity_z;

    // Guard Conditions
    //   Inequality A*lambda<b_A
    //       hand contact is sticking; (force is in world frame)
    //       hand contact normal force lower bound
    //       table contact normal force lower bound
    //       table contact normal force upper bound
    //       binwall contact normal force lower bound
    //       binwall contact normal force upper bound

    //   Equality Aeq*lambda = beq
    //       table contact is sliding;
    //       bin wall contact is sliding;
    // lambda: f_why, f_whx, f_table_normal, f_binwall_normal,
    //         f_table_friction, f_binwall_friction
    MatrixXd A = MatrixXd::Zero(2 + 1 + 4,
        kDimLambda + kDimSlidingFriction + kDimGeneralized);
    Vector2f y, z;
    y << 1, 0;
    z << 0, 1;
    A.block<1, 2>(0, 0) = (z.transpose() -
        kFrictionCoefficientHand_*y.transpose())*R_WO.transpose();
    A.block<1, 2>(1, 0) = (-z.transpose() -
        kFrictionCoefficientHand_*y.transpose())*R_WO.transpose();
    A.block<1, 2>(2, 0) = -y.transpose()*R_WO.transpose();
    A(3, 2) = -1;
    A(4, 2) = 1;
    A(5, 3) = -1;
    A(6, 3) = 1;
    VectorXd b_A(7);
    b_A << 0, 0, -kMinNormalForce_, -kMinNormalForceSliding_,
        kMaxNormalForceSliding_, -kMinNormalForceSliding_,
        kMaxNormalForceSliding_;

    MatrixXd Aeq = MatrixXd::Zero(2,
        kDimLambda + kDimSlidingFriction + kDimGeneralized);
    // -y'*ftable = mu*z'*ftable
    Vector2f Aeq_temp_entries =
        kFrictionCoefficientTable_*z.transpose()+y.transpose();
    Aeq(0, 4) = Aeq_temp_entries(0);
    Aeq(0, 2) = Aeq_temp_entries(1);
    // z'*f = mu*y'*f
    Aeq_temp_entries = kFrictionCoefficientBin_*y.transpose()-z.transpose();
    Aeq(1, 3) = Aeq_temp_entries(0);
    Aeq(1, 5) = Aeq_temp_entries(1);
    VectorXd beq(2);
    beq << 0, 0;

    float J_array[30];
    jac_phi_q_flip_against_corner(p_WO.data(), theta, p_WH.data(), p_OHC.data(),
        p_OTC.data(), p_OBC.data(), J_array);

    // read the jacobian from results
    MatrixXd Jac_phi_q(6, 5);
    for(int i = 0; i<6; i++)
      for(int j = 0; j<5; j++)
        Jac_phi_q(i,j) = J_array[j*6 + i];

    HFVC action;
    solvehfvc(Omega, Jac_phi_q, G, b_G, F, Aeq, beq,
      A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds,
      &action);

    double computation_time_ms = time.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;

    // check results
    VectorXd vel_command(kDimActualized);
    vel_command << VectorXf::Zero(action.n_af), action.w_av;
    VectorXd vel_W = action.R_a.inverse()*vel_command;
    cout << " World frame velocity: " << vel_W.format(MatlabFmt)
         << endl;

    VectorXd force_command(kDimActualized);
    force_command << action.eta_af, VectorXd::Zero(action.n_av);
    cout << "  World frame force: " << (action.R_a.inverse()*force_command).format(MatlabFmt)
        << endl;


    /*  Execute the hybrid action */
    /*  Reformulate the 2D solution to 3D action */
    float pose_set[7];
    robot_->getPose(pose_set); // get quaternion
    pose_set[1] = pose_set[1] + vel_W(0)*kTimeStepSec_*1000.0f;
    pose_set[2] = pose_set[2] + vel_W(1)*kTimeStepSec_*1000.0f;

    float force_set[6] = {0};
    for (int i = 0; i < action.n_af; ++i)  force_set[i] = action.eta_af(i);

    Matrix3f R_a = Matrix3f::Zero();
    R_a.block<2, 2>(0, 1) = action.R_a;
    R_a(2, 0) = 1;
    int n_af = action.n_af;
    int n_av = action.n_av + 1;

    cout << "running time: " << kTimeStepSec_ << endl;
    // cout << "n_af: " << n_af << endl;
    // cout << "n_av: " << n_av << endl;
    // cout << "R_a: " << R_a.format(MatlabFmt) << endl;
    // cout << "force_set: " << force_set[0] << "|" << force_set[1] << "|" <<
    //     force_set[2] << endl;
    // cout << "Press Enter to begin motion!" << endl;
    // getchar();
    cout << "motion begins:" << endl;
    ExecuteHFVC_ABBEGM(n_af, n_av,
        R_a, pose_set, force_set,
        HS_CONTINUOUS, kTimeStepSec_, main_loop_rate_,
        robot_, controller_);
  }

}