#include "tracking2d.h"

#include <fstream>
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

#define PI 3.14159265

using namespace RUT;
using std::cout;
using std::cerr;
using std::endl;
using Eigen::Vector2d;
using Eigen::Matrix2d;


bool Tracking2DTaskServer::initTracking2DTaskServer() {
  ROS_INFO_STREAM("Tracking2D server is starting");
  if (_ros_handle_p == nullptr) {
    ROS_ERROR_STREAM("[Tracking2DTaskServer] You must call .init() before .initTracking2DTaskServer().");
    exit(1);
  }

  _ros_handle_p->param(std::string("/task/translation_resolution"), _kTransResMM, 0.1);
  if (!_ros_handle_p->hasParam("/task/translation_resolution"))
    ROS_WARN_STREAM("Parameter [/task/translation_resolution] not found");
  _ros_handle_p->param(std::string("/task/translation_velocity"), _kTransVelMM, 5.0);
  if (!_ros_handle_p->hasParam("/task/translation_velocity"))
    ROS_WARN_STREAM("Parameter [/task/translation_velocity] not found");
  _ros_handle_p->param(std::string("/task/max_trans_per_frame"), _kTransMaxPerFrameMM, 10.0);
  if (!_ros_handle_p->hasParam("/task/max_trans_per_frame"))
    ROS_WARN_STREAM("Parameter [/task/max_trans_per_frame] not found");

  _ros_handle_p->param(std::string("/task/plan_offset/x"), _kPlanYOffsetX, 0.0);
  if (!_ros_handle_p->hasParam("/task/plan_offset/x"))
    ROS_WARN_STREAM("Parameter [/task/plan_offset/x] not found");
  _ros_handle_p->param(std::string("/task/plan_offset/y"), _kPlanYOffsetY, 0.0);
  if (!_ros_handle_p->hasParam("/task/plan_offset/y"))
    ROS_WARN_STREAM("Parameter [/task/plan_offset/y] not found");

  _ros_handle_p->param(std::string("/task/data_file_name"), _data_filename, "");
  if (!_ros_handle_p->hasParam("/task/data_file_name"))
    ROS_WARN_STREAM("Parameter [/task/data_file_name] not found");

  return true;
}

bool Tracking2DTaskServer::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &Tracking2DTaskServer::SrvReset, (RobotBridge*)this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &Tracking2DTaskServer::SrvMoveTool, (RobotBridge*)this);
  ros::ServiceServer move_until_touch_service = _ros_handle_p->advertiseService("move_until_touch", &Tracking2DTaskServer::SrvMoveUntilTouch, (RobotBridge*)this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &Tracking2DTaskServer::SrvGetPose, (RobotBridge*)this);
  ros::ServiceServer execute_task_service     = _ros_handle_p->advertiseService("execute_task", &Tracking2DTaskServer::SrvExecuteTask, this);

  cout << endl << "[Tracking2DTaskServer] Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[Tracking2DTaskServer] Service servers stopped." << endl);
  return true;
}

bool Tracking2DTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  std::cout << "[Tracking2DTaskServer] Reading data from " << _data_filename
      << std::endl;
  std::fstream fin;
  fin.open(_data_filename, std::ios::in);

  std::vector<VectorXd> one_traj;
  std::vector<string> row;
  std::string line, word, temp;
  while (fin >> temp) {
    row.clear();
    // Read a row
    getline(fin, line);
    stringstream s(line);
    while (getline(s, word, ', ')) {
      row.push_back(word);
    }
    // check whether to start a new trajectory or not
    double stability_margin = stof(row[0]);
    if (stability_margin < 0) {
      // save the old trajectory
      if (one_traj.size() > 0) {
        Eigen::MatrixXd one_traj_eigen(one_traj.size(), one_traj(0).rows());
        for (int i = 0; i < one_traj.size(); ++i) {
          one_traj_eigen.middleRows(i, 1) = one_traj[i].transpose();
        }
        motion_plans.push_back(one_traj_eigen);
      }
      // start a new trajectory
      one_traj.clear();
      Eigen::Vector2d vec;
      vec(0) = stof(row[1]);
      vec(1) = stof(row[2]);
      contact_normal_engaging.push_back(vec);
      vec(0) = stof(row[3]);
      vec(1) = stof(row[4]);
      contact_normal_disengaging.push_back(vec);
    } else {
      // add new line of data
      VectorXd vec(16);
      for (int i = 0; i < 16; ++i) {
        vec(i) = stof(row[i+1]);
      }
      one_traj.push_back(vec);
    }
  } // end while loop
  Eigen::MatrixXd one_traj_eigen(one_traj.size(), one_traj(0).rows());
  for (int i = 0; i < one_traj.size(); ++i) {
    one_traj_eigen.middleRows(i, 1) = one_traj[i].transpose();
  }
  motion_plans.push_back(one_traj_eigen);
  return true;
}

bool Tracking2DTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  // get these variables from motion plan
  int NFrames = 100;

  // Get current pose
  double pose[7];
  _robot.getPose(pose);
  _controller.reset();
  Timer time;
  for (int fr = 0; fr < NFrames; ++fr) {
    time.tic();

    // feedback
    _robot.getPose(pose);
    Matrix4d SE3_WT_fb = RUT::posemm2SE3(pose);

    Vector2d p_WH;
    p_WH(0) = pose[1]*0.001; // m
    p_WH(1) = pose[2]*0.001;

    // get from motion plan
    // HFVC action;
    int action_n_av;
    int action_n_af;
    Matrix3d R_a;
    VectorXd eta_af, w_av;

    Vector2d p_WT_goal;
    p_WT_goal[0] += _kPlanYOffsetX;
    p_WT_goal[1] += _kPlanYOffsetY;

    /**
     * Convert 2D world action to 3D tool action.
     * The 2D world is the y-z slice of the 3D world
     */
    // dimensions
    int n_af = action_n_af;
    int n_av = action_n_av + 4;
    // Compute Pose command
    // 1. Compute the generalized velocity
    Matrix3d R_inv = R_a.inverse();
    Vector3d vel_command;
    vel_command << VectorXd::Zero(action_n_af), w_av;
    Vector3d V3_T = R_inv*vel_command;
    // 2. Transform planar velocity to 3D tool frame
    //  This depends on the robot tool frame definition
    //  Here I assume toolZ points upwards, the three planar axes are
    //   [toolY, toolZ, toolRX]
    VectorXd V6_T = VectorXd::Zero(6);
    V6_T(1) = V3_T(0); // toolY = 2D x
    V6_T(2) = V3_T(1); // toolZ = 2D y
    V6_T(3) = V3_T(2); // toolRx = 2D R
    assert(fabs(V6_T(3)) < 1e-7); // it should not rotate
    // 3. Transform to world frame velocity
    Matrix6d Adj_WT = RUT::SE32Adj(SE3_WT_fb);
    VectorXd V_W = Adj_WT * V6_T;
    assert(fabs(V_W(0)) < 1e-7);
    cout << " World frame velocity: " << V_W.format(MatlabFmt) << endl;
    double pose_set[7];
    _robot.getPose(pose_set); // get quaternion
    // 4. Decide how far to move: move closest to the target WT
    V_W.normalize();
    double dist0 = 999, dist1;
    Vector2d p_WT_target;
    p_WT_target << pose[1], pose[2];
    int count = 0;
    int max_count = round(_kTransMaxPerFrameMM/_kTransResMM);
    for (count = 0; count < max_count; ++count) {
      p_WT_target[0] += V_W(1)*_kTransResMM;
      p_WT_target[1] += V_W(2)*_kTransResMM;
      dist1 = (p_WT_target - p_WT_goal).norm();
      if (dist1 > dist0) break;
      dist0 = dist1;
    }
    if (count >= max_count - 1) {
      std::cerr << "[Tracking2D] This frame goes too far!!!"
      " _kTransMaxPerFrameMM is violated." << std::endl;
      exit(-1);
    }
    double duration_s = count * _kTransResMM / _kTransVelMM;

    pose_set[1] = p_WT_target[0];
    pose_set[2] = p_WT_target[1];

    // Compute force command
    double force_set[6] = {0};
    for (int i = 0; i < action_n_af; ++i)  force_set[i] = eta_af(i);

    // Compute transformation
    /**
     * T =
     * vx vy  vz  rx  ry rz
       0  R01 R02 R03 0  0
       0  R11 R12 R13 0  0
       0  R21 R22 R23 0  0
       1  0   0   0   0  0
       0  0   0   0   1  0
       0  0   0   0   0  1
     */
    Matrix6d T_T = Matrix6d::Zero();
    T_T.block<3, 3>(0, 1) = R_a;
    T_T(3, 0) = 1;
    T_T(4, 4) = 1;
    T_T(5, 5) = 1;

    // print out world frame force for debug
    Vector3d F_command;
    F_command << eta_af, VectorXd::Zero(action_n_av);
    Vector3d F3_T = R_inv * F_command;
    VectorXd F6_T(6);
    F6_T(1) = F3_T(0); // toolY = 2D x
    F6_T(2) = F3_T(1); // toolZ = 2D y
    F6_T(3) = F3_T(2); // toolRx = 2D R
    assert(fabs(F6_T(3)) < 1e-7); // it should not rotate
    Matrix6d Adj_TW = RUT::SE32Adj(RUT::SE3Inv(SE3_WT_fb));
    Vector6d F_W = Adj_TW.transpose() * F6_T;
    assert(fabs(F_W(0)) < 1e-7);
    cout << " World frame wrench: " << F_W.format(MatlabFmt) << endl;
    cout << "Change of pose: " << pose_set[0] - pose[0] << ", "
        << pose_set[1] - pose[1] << ", "
        << pose_set[2] - pose[2] << endl;
    cout << "Duration: " << duration_s << " sec" << std::endl;
    /*  Execute the hybrid action */
    cout << "[Tracking2D] Press Enter to move\n";
    getchar();
    cout << "motion begins:" << endl;
    if (!_controller.ExecuteHFVC(n_af, n_av, T_T, pose_set, force_set,
        HS_CONTINUOUS, _main_loop_rate, duration_s)) {
      ROS_WARN_STREAM("[Tracking2DTaskServer] unsafe motion. Stopped prematurely.");
    }
  }
  return true;
}