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

  _ros_handle_p->param(std::string("/task/plan_offset/x"), _k2DTo3DOffsetX, 0.0);
  if (!_ros_handle_p->hasParam("/task/plan_offset/x"))
    ROS_WARN_STREAM("Parameter [/task/plan_offset/x] not found");
  _ros_handle_p->param(std::string("/task/plan_offset/y"), _k2DTo3DOffsetY, 0.0);
  if (!_ros_handle_p->hasParam("/task/plan_offset/y"))
    ROS_WARN_STREAM("Parameter [/task/plan_offset/y] not found");
  _ros_handle_p->param(std::string("/task/plan_offset/z"), _k2DTo3DOffsetZ, 0.0);
  if (!_ros_handle_p->hasParam("/task/plan_offset/z"))
    ROS_WARN_STREAM("Parameter [/task/plan_offset/z] not found");

  _ros_handle_p->param(std::string("/task/data_folder_path"), _data_folder_path, std::string());
  if (!_ros_handle_p->hasParam("/task/data_folder_path"))
    ROS_WARN_STREAM("Parameter [/task/data_folder_path] not found");
  _ros_handle_p->param(std::string("/task/data_file_name"), _data_filename, std::string());
  if (!_ros_handle_p->hasParam("/task/data_file_name"))
    ROS_WARN_STREAM("Parameter [/task/data_file_name] not found");

  _ros_handle_p->param(std::string("/task/XZ_plane"), _XZ_plane, true);
  if (!_ros_handle_p->hasParam("/task/XZ_plane"))
    ROS_WARN_STREAM("Parameter [/task/XZ_plane] not found");

  // // debug readMotionPlan
  // std_srvs::Empty::Request  req;
  // std_srvs::Empty::Response res;
  // SrvReadMotionPlan(req, res);
  return true;
}

bool Tracking2DTaskServer::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &Tracking2DTaskServer::SrvReset, (RobotBridge*)this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &Tracking2DTaskServer::SrvMoveTool, (RobotBridge*)this);
  ros::ServiceServer engage_service           = _ros_handle_p->advertiseService("engage", &Tracking2DTaskServer::SrvEngage, this);
  ros::ServiceServer preengage_service        = _ros_handle_p->advertiseService("pre_engage", &Tracking2DTaskServer::SrvPreEngage, this);
  ros::ServiceServer disengage_service        = _ros_handle_p->advertiseService("disengage", &Tracking2DTaskServer::SrvDisengage, this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &Tracking2DTaskServer::SrvGetPose, (RobotBridge*)this);
  ros::ServiceServer read_motion_plan_service = _ros_handle_p->advertiseService("read_motion_plan", &Tracking2DTaskServer::SrvReadMotionPlan, this);
  ros::ServiceServer execute_task_service     = _ros_handle_p->advertiseService("execute_task", &Tracking2DTaskServer::SrvExecuteTask, this);

  cout << endl << "[Tracking2DTaskServer] Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[Tracking2DTaskServer] Service servers stopped." << endl);
  return true;
}

bool Tracking2DTaskServer::SrvReadMotionPlan(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  std::string file = _data_folder_path + _data_filename;
  std::cout << "[Tracking2DTaskServer] Reading data from " << file << std::endl;
  std::fstream fin;
  fin.open(file);
  if (!fin.is_open()) {
    std::cout << "unable to open file!!" << std::endl;
    std::cerr << "Error: " << std::strerror(errno);
    exit(-1);
  }

  std::vector<VectorXd> one_traj;
  std::vector<std::string> row;
  std::string line, word;
  int counter = 0;
  while (getline(fin, line)) {
    row.clear();
    // Read a row
    std::stringstream s(line);
    while (getline(s, word, ',')) {
      row.push_back(word);
    }
    std::cout << "line " << ++counter << ", size " << row.size() << std::endl;
    std::cout << "contents: " << line << std::endl;
    std::cout << "reads: ";
    // check whether to start a new trajectory or not
    double stability_margin = std::stod(row[0]);
    if (stability_margin < 0) {
      // save this trajectory
      assert(one_traj.size() > 0);
      Eigen::MatrixXd one_traj_eigen(one_traj.size(), one_traj[0].rows());
      for (int i = 0; i < one_traj.size(); ++i) {
        one_traj_eigen.middleRows(i, 1) = one_traj[i].transpose();
      }
      _motion_plans_m.push_back(one_traj_eigen);
      Eigen::Vector2d vec;
      std::cout << " Info line. " << row[1] << ", " << row[2] << ", " << row[3] << ", " << row[4];
      vec(0) = std::stod(row[1]);
      vec(1) = std::stod(row[2]);
      _contact_normal_engaging.push_back(-vec);
      vec(0) = std::stod(row[3]);
      vec(1) = std::stod(row[4]);
      _contact_normal_disengaging.push_back(-vec);
      // start a new trajectory
      one_traj.clear();
    } else {
      // add new line of data
      VectorXd vec(17);
      for (int i = 0; i < 17; ++i) {
        std::cout << row[i] << ", ";
        vec(i) = std::stod(row[i]);
      }
      one_traj.push_back(vec);
    }
    std::cout << std::endl;
  } // end while loop
  if (one_traj.size() > 0) {
    std::cerr << "[Tracking2DTaskServer::SrvReadMotionPlan] Wrong data format! "
        "The last line must start with -1." << std::endl;
    exit(-1);
  }
  _traj_piece_count = 0;
  fin.close();

  std::cout << "_motion_plans_m: " << _motion_plans_m.size() << std::endl;
  std::cout << "_contact_normal_engaging: " << _contact_normal_engaging.size() << std::endl;
  std::cout << "_contact_normal_disengaging: " << _contact_normal_disengaging.size() << std::endl;
  std::cout << "motion plan 1:\n" << _motion_plans_m[0] << std::endl;
  std::cout << "[Tracking2DTaskServer] data is loaded. " << std::endl;
  return true;
}

bool Tracking2DTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  std::cout << "[Tracking2DTaskServer] Calling ExecuteTask. Running "
      " piece " << _traj_piece_count << std::endl;
  // get these variables from motion plan
  if (_traj_piece_count >= _motion_plans_m.size()) {
    std::cout << "[Tracking2DTaskServer] The plan is already finished."
        " Do nothing." << std::endl;
    return false;
  }
  int NFrames = _motion_plans_m[_traj_piece_count].rows();

  // Get current pose
  double pose[7];
  _controller.reset();
  Timer time;
  for (int fr = 0; fr < NFrames; ++fr) {
    time.tic();

    // feedback
    _robot.getPose(pose);
    Matrix4d SE3_WT_fb = RUT::posemm2SE3(pose);

    // get from motion plan
    Eigen::VectorXd plan = _motion_plans_m[_traj_piece_count].middleRows(fr, 1).transpose();
    // [margin, n_af, n_av, R_a (9), eta_af, w_av]
    int action_n_af = plan(1);
    int action_n_av = plan(2);
    Matrix3d R_a; // meter
    R_a << plan(3), plan(4), plan(5), plan(6), plan(7), plan(8), plan(9),
        plan(10), plan(11);
    VectorXd eta_af(action_n_af), w_av(action_n_av);
    for (int i = 0; i < action_n_af; ++i)
      eta_af(i) = plan(12+i);
    for (int i = 0; i < action_n_av; ++i)
      w_av(i) = plan(12+action_n_af+i);
    // debug hack
    // make the plan purely translational
    // 1. force (assume 2d)
    Eigen::Vector3d sum = (R_a.topRows(1)*eta_af(0) + R_a.middleRows(1,1)*eta_af(1)).transpose();
    sum(2) = 0; // rotational force set to zero
    eta_af(0) = sum(2);
    eta_af(1) = sum.head(2).norm();
    R_a(0,0) = 0;
    R_a(0,1) = 0;
    R_a(0,2) = 1;
    R_a.middleRows(1,1) = sum.normalized().transpose();
    // 2. velocity (assume 1d)
    Eigen::Vector3d v_row = R_a.bottomRows(1).transpose();
    v_row(2) = 0;
    R_a.bottomRows(1) = v_row.normalized().transpose();

    Vector3d p_WT_goal; // mm
    p_WT_goal << _k2DTo3DOffsetX, _k2DTo3DOffsetY, _k2DTo3DOffsetZ;
    if (_XZ_plane) {
      p_WT_goal[0] += plan(15)*1000.0;
      p_WT_goal[2] += plan(16)*1000.0;
    } else {
      p_WT_goal[1] += plan(16)*1000.0;
      p_WT_goal[2] += plan(16)*1000.0;
    }

    /**
     * Convert 2D world action to 3D tool action.
     * The 2D world is the y-z slice of the 3D world
     */
    // dimensions
    int n_af = action_n_af;
    int n_av = action_n_av + 3;
    // Compute Pose command
    // 1. Compute the generalized velocity
    Matrix3d R_inv = R_a.inverse();
    Vector3d vel_command;
    vel_command << VectorXd::Zero(action_n_af), w_av; // meter/s
    Vector3d V3_T = R_inv*vel_command; // meter/s
    // 2. Transform planar velocity to 3D tool frame
    //  This depends on the robot tool frame definition
    //  Here I assume toolZ points upwards, the three planar axes are
    //   [toolY, toolZ, toolRX]
    VectorXd V6_T = VectorXd::Zero(6); // meter/s
    // // debug hack
    // V3_T(2) = 0;
    if (_XZ_plane) {
      V6_T(0) = V3_T(0); // toolX = 2D x
      V6_T(2) = V3_T(1); // toolZ = 2D y
      V6_T(4) = - V3_T(2); // toolRy = - 2D R
    } else {
      V6_T(1) = V3_T(0); // toolY = 2D x
      V6_T(2) = V3_T(1); // toolZ = 2D y
      V6_T(3) = V3_T(2); // toolRx = 2D R
    }
    std::cout << "R_a:\n" << R_a << std::endl;
    std::cout << "w_av: " << w_av.transpose() << std::endl;
    std::cout << "V3_T: " << V3_T.transpose() << std::endl;
    std::cout << "V6_T: " << V6_T.transpose() << std::endl;
    assert(fabs(V3_T(2)) < 1e-7); // it should not rotate

    // 3. Transform to world frame velocity
    Matrix6d Adj_WT = RUT::SE32Adj(SE3_WT_fb); // meter/s
    VectorXd V_W = Adj_WT * V6_T; // unitless
    V_W.normalize();
    cout << " World frame velocity: " << V_W.transpose().format(MatlabFmt) << endl;
    double pose_set[7];
    _robot.getPose(pose_set); // get quaternion
    // 4. Decide how far to move: move closest to the target WT
    double dist0 = 999, dist1;
    Vector3d p_WT_target;
    p_WT_target << pose[0], pose[1], pose[2];
    int count = 0;
    int max_count = round(_kTransMaxPerFrameMM/_kTransResMM);
    for (count = 0; count < max_count; ++count) {
      p_WT_target[0] += V_W(0)*_kTransResMM;
      p_WT_target[1] += V_W(1)*_kTransResMM;
      p_WT_target[2] += V_W(2)*_kTransResMM;
      dist1 = (p_WT_target - p_WT_goal).norm();
      if (dist1 > dist0) break;
      dist0 = dist1;
    }
    // std::cout << "p_WT: " << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;
    // std::cout << "p_WT_goal: " << p_WT_goal.transpose() << std::endl;
    // std::cout << "p_WT_target: " << p_WT_target.transpose() << std::endl;
    if (count >= max_count - 1) {
      std::cerr << "[Tracking2D] This frame goes too far!!!"
      " _kTransMaxPerFrameMM is violated." << std::endl;
      exit(-1);
    }
    double duration_s = count * _kTransResMM / _kTransVelMM;

    pose_set[0] = p_WT_target[0];
    pose_set[1] = p_WT_target[1];
    pose_set[2] = p_WT_target[2];

    // Compute force command
    double force_set[6] = {0};
    for (int i = 0; i < action_n_af; ++i)  force_set[i] = eta_af(i);

    // Compute transformation
    Matrix6d T_T = Matrix6d::Zero();
    if (_XZ_plane) {
      /**
       * XZ Plane
       * T =
       * vx vy  vz  rx  ry rz
         R01 0 R02 -R03  0  0
         R11 0 R12 -R13  0  0
         R21 0 R22 -R23  0  0
         0   1   0   0   0  0
         0   0   0   0   1  0
         0   0   0   0   0  1
       */
      T_T.block<3, 1>(0, 0) = R_a.leftCols(1);
      T_T.block<3, 1>(0, 2) = R_a.middleCols(1,1);
      T_T.block<3, 1>(0, 3) = - R_a.rightCols(1);
      T_T(3, 1) = 1;
      T_T(4, 4) = 1;
      T_T(5, 5) = 1;
    } else {
      /**
       * YZ Plane
       * T =
       * vx vy  vz  rx  ry rz
         0  R01 R02 R03 0  0
         0  R11 R12 R13 0  0
         0  R21 R22 R23 0  0
         1  0   0   0   0  0
         0  0   0   0   1  0
         0  0   0   0   0  1
       */
      T_T.block<3, 3>(0, 1) = R_a;
      T_T(3, 0) = 1;
      T_T(4, 4) = 1;
      T_T(5, 5) = 1;
    }

    // print out world frame force for debug
    Vector3d F_command;
    F_command << eta_af, VectorXd::Zero(action_n_av);
    Vector3d F3_T = R_inv * F_command;
    VectorXd F6_T(6);
    // // debug hack
    // // ignore rotation
    // F3_T(2) = 0;
    assert(fabs(F3_T(2)) < 1e-7); // it should not rotate
    if (_XZ_plane) {
      F6_T(0) = F3_T(0); // toolX = 2D x
      F6_T(2) = F3_T(1); // toolZ = 2D y
      F6_T(4) = -F3_T(2); // toolRy = - 2D R
    } else {
      F6_T(1) = F3_T(0); // toolY = 2D x
      F6_T(2) = F3_T(1); // toolZ = 2D y
      F6_T(3) = F3_T(2); // toolRx = 2D R
    }
    // Matrix6d Adj_TW = RUT::SE32Adj(RUT::SE3Inv(SE3_WT_fb));
    // Vector6d F_W = Adj_TW.transpose() * F6_T;
    cout << " Tool frame wrench: " << F6_T.transpose().format(MatlabFmt) << endl;
    cout << "Change of pose: " << pose_set[0] - pose[0] << ", "
        << pose_set[1] - pose[1] << ", "
        << pose_set[2] - pose[2] << endl;
    cout << "Duration: " << duration_s << " sec" << std::endl;
    /*  Execute the hybrid action */
    if (_test_mode) {
      cout << "[Tracking2D] Press Enter to move\n";
      if (getchar() == 'q') {
        exit(1);
      }
    }
    cout << "motion begins:" << endl;
    if (_controller.ExecuteHFVC(n_af, n_av, T_T, pose_set, force_set,
        HS_CONTINUOUS, _main_loop_rate, duration_s)) {
      ROS_WARN_STREAM("[Tracking2DTaskServer] unsafe motion. Stopped prematurely.");
      return false;
    }
  }
  std::cout << "[Tracking2DTaskServer] SrvExecuteTask is done. " << std::endl;
  return true;
}

bool Tracking2DTaskServer::SrvPreEngage(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  std::cout << "[Tracking2DTaskServer] Calling SrvPreEngage. " << std::endl;
  if (_traj_piece_count >= _motion_plans_m.size()) {
    std::cout << "[Tracking2DTaskServer] The plan is already finished, "
        " or it is not loaded." << std::endl;
    return false;
  }
   // read normal
  Eigen::Vector2d v2_W = _contact_normal_engaging[_traj_piece_count];
  v2_W.normalize();
  Eigen::Vector3d v3_W;
  if (_XZ_plane) {
    v3_W << v2_W(0), 0, v2_W(1);
  } else {
    v3_W << 0, v2_W(0), v2_W(1);
  }
  // read initial frame
  Eigen::VectorXd plan = _motion_plans_m[_traj_piece_count].topRows(1).transpose();
  Vector2d p2_W; // mm
  p2_W << plan(15), plan(16);
  p2_W *= 1000.0;
  p2_W += v2_W*20.0;
  Vector3d p3_W;
  p3_W << _k2DTo3DOffsetX, _k2DTo3DOffsetY, _k2DTo3DOffsetZ;
  if (_XZ_plane) {
    p3_W(0) += p2_W(0);
    p3_W(2) += p2_W(1);
  } else {
    p3_W(1) += p2_W(0);
    p3_W(2) += p2_W(1);
  }

  double pose[7];
  _robot.getPose(pose); // get orientaton
  pose[0] = p3_W(0);
  pose[1] = p3_W(1);
  pose[2] = p3_W(2);

  std::cout << "engage prepare pose: " << p3_W[0] << ", " << p3_W[1] << ", "
      << p3_W[2] << std::endl;
  // std::cout << "Press Enter to begin." << std::endl;
  // getchar();
  // write to file
  std::ofstream fout;
  fout.open(_data_folder_path + "pose_set.txt", std::ios::out | std::ios::trunc);
  for (int i = 0; i < 7; ++i)
    fout << pose[i] << " ";
  fout.close();
  // call parent MoveUntilTouch
  bool flag = SrvMoveTool(req, res);
  std::cout << "SrvPreEngage is done." << std::endl;
  return flag;
}

bool Tracking2DTaskServer::SrvEngage(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  std::cout << "[Tracking2DTaskServer] Calling SrvEngage. " << std::endl;
  if (_traj_piece_count >= _motion_plans_m.size()) {
    std::cout << "[Tracking2DTaskServer] The plan is already finished, "
        " or it is not loaded." << std::endl;
    return false;
  }
   // read normal
  Eigen::Vector2d v2_W = _contact_normal_engaging[_traj_piece_count];
  v2_W.normalize();
  Eigen::Vector3d v3_W;
  if (_XZ_plane) {
    v3_W << v2_W(0), 0, v2_W(1);
  } else {
    v3_W << 0, v2_W(0), v2_W(1);
  }
  v3_W *= -10; // mm/s
  // write to file
  std::ofstream fout;
  fout.open(_data_folder_path + "velocity_set.txt", std::ios::out | std::ios::trunc);
  fout << v3_W(0) << v3_W(1) << v3_W(2);
  fout.close();
  std::cout << "Engaging direction: " << v3_W.transpose() << std::endl;
  // call parent MoveUntilTouch
  bool flag = SrvMoveUntilTouch(req, res);
  std::cout << "[Tracking2DTaskServer] SrvEngage is done. " << std::endl;
  return flag;
}

bool Tracking2DTaskServer::SrvDisengage(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  // read normal
  Eigen::Vector2d v2_W = _contact_normal_disengaging[_traj_piece_count];
  Eigen::Vector3d v3_W;
  if (_XZ_plane) {
    v3_W << v2_W(0), 0, v2_W(1);
  } else {
    v3_W << 0, v2_W(0), v2_W(1);
  }

  v3_W.normalize();
  v3_W *= 50; // mm

  // compute retract pose
  double pose[7];
  _robot.getPose(pose);
  std::cout << "current pose: " << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;
  std::cout << "offset: " << v3_W.transpose() << std::endl;
  pose[0] = pose[0] + v3_W(0);
  pose[1] = pose[1] + v3_W(1);
  pose[2] = pose[2] + v3_W(2);

  // write to file
  std::ofstream fout;
  fout.open(_data_folder_path + "pose_set.txt", std::ios::out | std::ios::trunc);
  for (int i = 0; i < 7; ++i)
    fout << pose[i] << " ";
  fout.close();
  // call parent MoveUntilTouch
  bool result = SrvMoveTool(req, res);
  _traj_piece_count ++; // advance to the next piece
  return result;
}