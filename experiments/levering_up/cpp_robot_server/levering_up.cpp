#include "levering_up.h"

#include <fstream>
#include <solvehfvc.h>
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

// Matlab CodeGen files
#include <stddef.h>
#include <stdlib.h>
#include <rtwtypes.h>
#include <jacobian_levering_up.h>
#include <jacobian_levering_up_types.h>
#include <jacobian_levering_up_terminate.h>
#include <jacobian_levering_up_initialize.h>

#define PI 3.14159265

using namespace RUT;
using std::cout;
using std::cerr;
using std::endl;
using Eigen::Vector2d;
using Eigen::Matrix2d;


bool LeveringUpTaskServer::initLeveringUpTaskServer() {
  ROS_INFO_STREAM("Levering_up server is starting");
  if (_ros_handle_p == nullptr) {
    ROS_ERROR_STREAM("[LeveringUpTaskServer] You must call .init() before .initLeveringUpTaskServer().");
    exit(1);
  }

  _ros_handle_p->param(std::string("/task/goal_rotation_angle_deg"), _kGoalTheta, 45.0);
  if (!_ros_handle_p->hasParam("/task/goal_rotation_angle_deg"))
    ROS_WARN_STREAM("Parameter [/task/goal_rotation_angle_deg] not found");
  _ros_handle_p->param(std::string("/task/time_step"), _kTimeStepSec, 0.1);
  if (!_ros_handle_p->hasParam("/task/time_step"))
    ROS_WARN_STREAM("Parameter [/task/time_step] not found");
  _ros_handle_p->param(std::string("/task/object_mass"), _kObjectMass, 0.1);
  if (!_ros_handle_p->hasParam("/task/object_mass"))
    ROS_WARN_STREAM("Parameter [/task/object_mass] not found");
  _ros_handle_p->param(std::string("/task/hand_mass"), _kHandMass, 0.1);
  if (!_ros_handle_p->hasParam("/task/hand_mass"))
    ROS_WARN_STREAM("Parameter [/task/hand_mass] not found");
  _ros_handle_p->param(std::string("/task/object_length"), _kObjectLength, 0.1);
  if (!_ros_handle_p->hasParam("/task/object_length"))
    ROS_WARN_STREAM("Parameter [/task/object_length] not found");
  _ros_handle_p->param(std::string("/task/object_thickness"), _kObjectThickness, 0.1);
  if (!_ros_handle_p->hasParam("/task/object_thickness"))
    ROS_WARN_STREAM("Parameter [/task/object_thickness] not found");
  _ros_handle_p->param(std::string("/task/friction_coef/table_object"), _kFrictionCoefficientTable, 0.1);
  if (!_ros_handle_p->hasParam("/task/friction_coef/table_object"))
    ROS_WARN_STREAM("Parameter [/task/friction_coef/table_object] not found");
  _ros_handle_p->param(std::string("/task/friction_coef/hand_object"), _kFrictionCoefficientHand, 0.1);
  if (!_ros_handle_p->hasParam("/task/friction_coef/hand_object"))
    ROS_WARN_STREAM("Parameter [/task/friction_coef/hand_object] not found");
  _ros_handle_p->param(std::string("/task/friction_coef/bin_object"), _kFrictionCoefficientBin, 0.1);
  if (!_ros_handle_p->hasParam("/task/friction_coef/bin_object"))
    ROS_WARN_STREAM("Parameter [/task/friction_coef/bin_object] not found");
  _ros_handle_p->param(std::string("/task/min_normal_force_sticking"), _kMinNormalForceSticking, 0.1);
  if (!_ros_handle_p->hasParam("/task/min_normal_force_sticking"))
    ROS_WARN_STREAM("Parameter [/task/min_normal_force_sticking] not found");
  _ros_handle_p->param(std::string("/task/min_normal_force_sliding"), _kMinNormalForceSliding, 0.1);
  if (!_ros_handle_p->hasParam("/task/min_normal_force_sliding"))
    ROS_WARN_STREAM("Parameter [/task/min_normal_force_sliding] not found");
  _ros_handle_p->param(std::string("/task/max_normal_force_sliding"), _kMaxNormalForceSliding, 0.1);
  if (!_ros_handle_p->hasParam("/task/max_normal_force_sliding"))
    ROS_WARN_STREAM("Parameter [/task/max_normal_force_sliding] not found");
  _ros_handle_p->param(std::string("/task/goal_rotation_velocity"), _kGoalRotationVelocity, 0.1);
  if (!_ros_handle_p->hasParam("/task/goal_rotation_velocity"))
    ROS_WARN_STREAM("Parameter [/task/goal_rotation_velocity] not found");
  _ros_handle_p->param(std::string("/task/hand_height0"), _kHandHeight0, 0.0);
  if (!_ros_handle_p->hasParam("/task/hand_height0"))
    ROS_WARN_STREAM("Parameter [/task/hand_height0] not found");

  _kGoalTheta *= PI/180.0;

  return true;
}

bool LeveringUpTaskServer::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &LeveringUpTaskServer::SrvReset, (RobotBridge*)this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &LeveringUpTaskServer::SrvMoveTool, (RobotBridge*)this);
  ros::ServiceServer move_until_touch_service = _ros_handle_p->advertiseService("move_until_touch", &LeveringUpTaskServer::SrvMoveUntilTouch, (RobotBridge*)this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &LeveringUpTaskServer::SrvGetPose, (RobotBridge*)this);
  ros::ServiceServer execute_task_service     = _ros_handle_p->advertiseService("execute_task", &LeveringUpTaskServer::SrvExecuteTask, this);

  cout << endl << "[LeveringUpTaskServer] Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[LeveringUpTaskServer] Service servers stopped." << endl);
  return true;
}

bool LeveringUpTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  // Get current pose
  double pose[7];
  _robot.getPose(pose); // get quaternion
  Eigen::Vector2d p_WH0, p_WO0;
  p_WH0 << pose[1]/1000.0, pose[2]/1000;
  p_WO0[0] = p_WH0[0] - (6.5 + _kObjectLength/2);
  p_WO0[1] = p_WH0[1] - (_kHandHeight0 - _kObjectThickness/2);

  // dimensions
  int kDimActualized      = 2;
  int kDimUnActualized    = 3;
  int kDimSlidingFriction = 2;
  int kDimLambda          = 4;
  int kNumSeeds           = 2;
  int kPrintLevel         = 1;

  int kDimGeneralized = kDimActualized + kDimUnActualized;
  double a = p_WH0(1) - p_WO0(1) + _kObjectThickness*0.5f;
  double b = _kObjectLength;
  double l_diagonal = std::sqrt(_kObjectThickness*_kObjectThickness +
      _kObjectLength*_kObjectLength);
  double angle_inner_sharp = std::asin(_kObjectThickness/l_diagonal);
  Vector2d p_OHC = p_WH0 - p_WO0;
  Vector2d p_OTC, p_OBC;
  p_OTC << -_kObjectLength*0.5, -_kObjectThickness*0.5;
  p_OBC << -_kObjectLength*0.5, _kObjectThickness*0.5;
  MatrixXd Omega = MatrixXd::Identity(5, 5);
  VectorXd F(5);
  double kGravityConstant = 9.8;
  F << 0, -_kObjectMass*kGravityConstant, 0, 0, -_kHandMass*kGravityConstant;

  _controller.reset();

  Timer time;
  for (int fr = 0; fr < 1000; ++fr) {
    time.tic();

    // compute object pose
    //   Ideally we should use perception; here we hack it by assuming the
    //   contact between the hand and the object is sticking, and solve for the
    //   object pose from hand pose

    // feedback
    _robot.getPose(pose); // get quaternion
    Vector2d p_WH;
    p_WH(0) = pose[1]*0.001f; // m
    p_WH(1) = pose[2]*0.001f;

    // 1. Solve for theta
    //   This is a a*sin(theta)+bsin(theta)=c problem
    double c = p_WH(1) - p_WO0(1) + _kObjectThickness*0.5;
    double phi = std::atan2(a, b);
    double theta_plus_phi = std::asin(c/(std::sqrt(a*a+b*b)));
    double theta = theta_plus_phi - phi;
    cout << "a = " << a << ", b = " << b << ", theta = " << theta*180.0/PI
        << " degree" << endl;
    if (theta > _kGoalTheta) {
      cout << "Goal theta achieved. Exiting.." << endl;
      break;
    }
    assert(theta >= -5.0*PI/180.0);
    assert(theta <= 90.0*PI/180.0);

    // 2. solve for p_WO
    Vector2d p_temp, p_WO;
    p_temp <<
        _kObjectThickness*std::sin(theta)+l_diagonal*0.5*cos(theta+angle_inner_sharp),
        l_diagonal*0.5*std::sin(theta+angle_inner_sharp);
    p_WO = p_WO0;
    p_WO(0) -= _kObjectLength*0.5;
    p_WO(1) -= _kObjectThickness*0.5;
    p_WO    += p_temp;

    Matrix3d R_WO3 = aa2mat(theta, Vector3d::UnitX());
    Matrix2d R_WO = R_WO3.block<2,2>(1,1);

    // goal
    MatrixXd G(1, 5);
    G << 0, 0, 1, 0, 0;
    VectorXd b_G(1);
    b_G << _kGoalRotationVelocity;

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
    Vector2d y, z;
    y << 1, 0;
    z << 0, 1;
    A.block<1, 2>(0, 0) = (z.transpose() -
        _kFrictionCoefficientHand*y.transpose())*R_WO.transpose();
    A.block<1, 2>(1, 0) = (-z.transpose() -
        _kFrictionCoefficientHand*y.transpose())*R_WO.transpose();
    A.block<1, 2>(2, 0) = -y.transpose()*R_WO.transpose();
    A(3, 2) = -1;
    A(4, 2) = 1;
    A(5, 3) = -1;
    A(6, 3) = 1;
    VectorXd b_A(7);
    b_A << 0, 0, -_kMinNormalForceSticking, -_kMinNormalForceSliding,
        _kMaxNormalForceSliding, -_kMinNormalForceSliding,
        _kMaxNormalForceSliding;

    MatrixXd Aeq = MatrixXd::Zero(2,
        kDimLambda + kDimSlidingFriction + kDimGeneralized);
    // -y'*ftable = mu*z'*ftable
    Vector2d Aeq_temp_entries =
        _kFrictionCoefficientTable*z.transpose()+y.transpose();
    Aeq(0, 4) = Aeq_temp_entries(0);
    Aeq(0, 2) = Aeq_temp_entries(1);
    // z'*f = mu*y'*f
    Aeq_temp_entries = _kFrictionCoefficientBin*y.transpose()-z.transpose();
    Aeq(1, 3) = Aeq_temp_entries(0);
    Aeq(1, 5) = Aeq_temp_entries(1);
    VectorXd beq(2);
    beq << 0, 0;

    double J_array[30];
    jacobian_levering_up(p_WO.data(), theta, p_WH.data(), p_OHC.data(),
        p_OTC.data(), p_OBC.data(), J_array);

    // read the jacobian from results
    MatrixXd Jac_phi_q(6, 5);
    for(int i = 0; i<6; i++)
      for(int j = 0; j<5; j++)
        Jac_phi_q(i,j) = J_array[j*6 + i];

    HFVC action;
    solvehfvc(Jac_phi_q*Omega, G, b_G, F, Aeq, beq,
      A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds, kPrintLevel,
      &action);

    double computation_time_ms = time.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;

    /* Convert 2D world action to 3D tool action.
     * The 2D world is the y-z slice of the 3D world
     */
    // pose command
    Vector2d vel_command;
    vel_command << VectorXd::Zero(action.n_af), action.w_av;
    Vector6d vel_W;
    vel_W.block<2,1>(1, 0) = action.R_a.inverse()*vel_command;
    cout << " World frame velocity: " << vel_W.format(MatlabFmt) << endl;
    double pose_set[7];
    _robot.getPose(pose_set); // get quaternion
    pose_set[1] = pose_set[1] + vel_W(1)*_kTimeStepSec*1000.0f;
    pose_set[2] = pose_set[2] + vel_W(2)*_kTimeStepSec*1000.0f;

    // force command
    double force_set[6] = {0};
    for (int i = 0; i < action.n_af; ++i)  force_set[i] = action.eta_af(i);
    Vector2d force_command;
    force_command << action.eta_af, VectorXd::Zero(action.n_av);
    Vector6d force_W;
    force_W.block<2,1>(1, 0) = action.R_a.inverse()*force_command;
    cout << " World frame force: " << force_W.format(MatlabFmt) << endl;

    // transformation
    Matrix6d T_w = Matrix6d::Zero();
    T_w.block<2, 2>(0, 1) = action.R_a;
    T_w(2, 0) = 1;
    T_w(3, 3) = 1;
    T_w(4, 4) = 1;
    T_w(5, 5) = 1;
    Matrix4d SE3_WT_fb = RUT::posemm2SE3(pose);
    Matrix6d Adj_WT = RUT::SE32Adj(SE3_WT_fb);
    Matrix6d T_T = T_w * Adj_WT;

    // dimensions
    int n_af = action.n_af;
    int n_av = action.n_av + 4;


    /*  Execute the hybrid action */
    cout << "motion begins:" << endl;
    _controller.ExecuteHFVC(n_af, n_av,
        T_T, pose_set, force_set,
        HS_CONTINUOUS, _main_loop_rate);
  }

}