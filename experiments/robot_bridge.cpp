#include "robot_bridge.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <mutex>

#include <Eigen/Geometry>

#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

using namespace std;
using namespace RUT;
using namespace Eigen;

RobotBridge::RobotBridge() {
  _default_pose = new double[7];
}

RobotBridge::~RobotBridge() {
  delete [] _default_pose;
}

int RobotBridge::init(ros::NodeHandle *ros_handle, Clock::time_point time0,
    FTInterfaces *ft, RobotInterfaces *robot){
  _ros_handle_p = ros_handle;

  ROS_INFO_STREAM("robot_bridge server is starting");

  /**
   * Force control parameters
   */
  ros_handle->param(std::string("/force_control/main_loop_rate"), _main_loop_rate, 500);
  if (!ros_handle->hasParam("/force_control/main_loop_rate"))
    ROS_WARN_STREAM("Parameter [/force_control/main_loop_rate] not found");

  /**
   * Robot bridge parameters
   */
  ros_handle->param(std::string("/robot_bridge/force_touch_threshold"), _force_touch_threshold, 1.0);
  if (!ros_handle->hasParam("/robot_bridge/force_touch_threshold"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/force_touch_threshold] not found");
  ros_handle->param(std::string("/robot_bridge/pose_set_file_path"), _pose_set_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/pose_set_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/pose_set_file_path] not found");
  ros_handle->param(std::string("/robot_bridge/velocity_set_file_path"), _velocity_set_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/velocity_set_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/velocity_set_file_path] not found");
  ros_handle->param(std::string("/robot_bridge/pose_feedback_file_path"), _pose_feedback_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/pose_feedback_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/pose_feedback_file_path] not found");
  ros_handle->param(std::string("/robot_bridge/task_data_file_path"), _task_data_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/task_data_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/task_data_file_path] not found");
  ros_handle->param(std::string("/robot_bridge/hybrid_action_file_path"), _hybrid_action_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/hybrid_action_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/hybrid_action_file_path] not found");
  if (!ros_handle->hasParam("/robot_bridge/default_pose")) {
    ROS_WARN_STREAM("Parameter [/robot_bridge/default_pose] not found!");
    return -1;
  }
  ros_handle->param(std::string("/robot_bridge/default_pose/x"), _default_pose[0], 40.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/y"), _default_pose[1], 356.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/z"), _default_pose[2], 400.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/qw"), _default_pose[3], 0.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/qx"), _default_pose[4], 0.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/qy"), _default_pose[5], 1.0);
  ros_handle->param(std::string("/robot_bridge/default_pose/qz"), _default_pose[6], 0.0);

  /**
   * Trapezodial motion planning
   */
  ros_handle->param(string("/trapezodial/vel_max_translation"), _kVelMaxTrans, 0.0);
  ros_handle->param(string("/trapezodial/acc_max_translation"), _kAccMaxTrans, 0.0);
  ros_handle->param(string("/trapezodial/vel_max_rotation"), _kVelMaxRot, 0.0);
  ros_handle->param(string("/trapezodial/acc_max_rotation"), _kAccMaxRot, 0.0);
  if (!ros_handle->hasParam("/trapezodial/vel_max_translation"))
    ROS_WARN_STREAM("Parameter [/trapezodial/vel_max_translation] not found");
  if (!ros_handle->hasParam("/trapezodial/acc_max_translation"))
    ROS_WARN_STREAM("Parameter [/trapezodial/acc_max_translation] not found");
  if (!ros_handle->hasParam("/trapezodial/vel_max_rotation"))
    ROS_WARN_STREAM("Parameter [/trapezodial/vel_max_rotation] not found");
  if (!ros_handle->hasParam("/trapezodial/acc_max_rotation"))
    ROS_WARN_STREAM("Parameter [/trapezodial/acc_max_rotation] not found");

    // --------------------------------------------------------
    // Initialize robot and force controller
    // --------------------------------------------------------
    _robot.init(*ros_handle, time0, ft, robot); // robot must be initialized before controller
    _controller.init(*ros_handle, &_robot, time0);
    _controller.reset();
    return 0;
  }

bool RobotBridge::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &RobotBridge::SrvReset, this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &RobotBridge::SrvMoveTool, this);
  ros::ServiceServer move_until_touch_service = _ros_handle_p->advertiseService("move_until_touch", &RobotBridge::SrvMoveUntilTouch, this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &RobotBridge::SrvGetPose, this);
  ros::ServiceServer hybrid_servo_service     = _ros_handle_p->advertiseService("hybrid_servo", &RobotBridge::SrvHybridServo, this);

  cout << endl << "[robot_bridge] Initialization is done. Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
  return true;
}

bool RobotBridge::SrvReset(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _mtx.lock();
  ROS_INFO("[robot_bridge] Calling SrvReset!\n");

  double pose[7];
  _robot.getPose(pose);

  MatrixXd pose_traj;
  RUT::MotionPlanningTrapezodial(pose, _default_pose, _kAccMaxTrans,
      _kVelMaxTrans, _kAccMaxRot, _kVelMaxRot, (double)_main_loop_rate,
      &pose_traj);
  int num_of_steps = pose_traj.cols();

  cout << "Moving to reset location.." << endl;
  ros::Rate pub_rate(_main_loop_rate);
  for (int i = 0; i < num_of_steps; ++i) {
    _robot.setPose(pose_traj.col(i).data());
    pub_rate.sleep();
  }

  cout << "[robot_bridge] Reset is done." << endl;

  _mtx.unlock();
  return true;
}

bool RobotBridge::SrvMoveTool(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _mtx.lock();
  ROS_INFO("[robot_bridge] Calling SrvMoveTool!\n");

  /**
   * Read target pose from file.
   * TODO(Yifan): use pose message instead
   */
  ROS_INFO_STREAM("Reading pose from:\n" << _pose_set_file_path << "\n");

  ifstream fp;
  fp.open(_pose_set_file_path);

  if (!fp) {
    cerr << "Unable to open file for fp'.";
        exit(1); // terminate with error
  }
  double pose_set[7];
  cout << "Pose_set: ";
  for (int i = 0; i < 7; ++i) {
    fp >> pose_set[i];
    cout << pose_set[i] << ", ";
  }
  cout << endl;
  fp.close();

  /**
   * Move the robot to the target
   */
  double pose[7];
  _robot.getPose(pose);

  MatrixXd pose_traj;

  int num_of_steps = round(double(_main_loop_rate) * 1.0);
  RUT::MotionPlanningLinear(pose, pose_set, num_of_steps, &pose_traj);

  // RUT::MotionPlanningTrapezodial(pose, pose_set, _kAccMaxTrans, _kVelMaxTrans,
  //   _kAccMaxRot, _kVelMaxRot, (double)_main_loop_rate, &pose_traj);
  // int num_of_steps = pose_traj.cols();

  ros::Rate pub_rate(_main_loop_rate);
  for (int i = 0; i < num_of_steps; ++i) {
    _robot.setPose(pose_traj.col(i).data());
    pub_rate.sleep();
  }
  cout << "[robot_bridge] MoveTool finished. " << endl;

  _mtx.unlock();
  return true;
}


bool RobotBridge::SrvMoveUntilTouch(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _mtx.lock();
  ROS_INFO("[robot_bridge] Calling SrvMoveUntilTouch!\n");

  /**
   * Read velocity command
   */
  ROS_INFO("Reading velocity from the file..\n");

  ifstream fp;
  fp.open(_velocity_set_file_path);

  if (!fp) {
    cerr << "Unable to open file for fp'.";
    exit(1); // terminate with error
  }
  Vector3d v_set;
  cout << "velocity_set: ";
  for (int i = 0; i < 3; ++i) {
    fp >> v_set[i];
    cout << v_set[i] << ", ";
  }
  cout << endl;
  fp.close();

  /**
   * Beging engaging
   */

  double pose[7];
  _robot.getPose(pose);
  Vector3d p_set, v_delta;
  p_set << pose[0], pose[1], pose[2];
  v_delta = v_set/double(_main_loop_rate);
  ros::Rate pub_rate(_main_loop_rate);

  double wrench[6];
  double wrench_safety_check[3] = {0, 0, 0};
  int safety_count = 0;
  while (true) {
    // check force feedback
    int code = _robot.getWrench(wrench);
    cout << "wrench: "<< wrench[0] << ", " << wrench[1] << ", " << wrench[2]
        << endl;
    double force_mag = sqrt(wrench[0]*wrench[0] + wrench[1]*wrench[1] +
        wrench[2]*wrench[2]);
    double torque_mag = sqrt(wrench[3]*wrench[3] + wrench[4]*wrench[4] +
        wrench[5]*wrench[5]);
    if ((force_mag > _force_touch_threshold) || (torque_mag > 0.4f)) {
      cout << "[MoveUntilTouch] Touched!" << endl;
      _robot.getPose(pose);
      _robot.setPose(pose);
      pub_rate.sleep();
      _robot.setPose(pose);
      break;
    }

    if (code == 2) {
      cout << "[MoveUntilTouch] FT sensor stopped!!" << endl;
      break;
    }
    wrench_safety_check[0] = wrench[0];
    wrench_safety_check[1] = wrench[1];
    wrench_safety_check[2] = wrench[2];

    p_set += v_delta;
    pose[0] = p_set(0);
    pose[1] = p_set(1);
    pose[2] = p_set(2);

    _robot.setPose(pose);
    pub_rate.sleep();
  }

  cout << "[robot_bridge] MoveUntilTouch finished. " << endl;

  _mtx.unlock();
  return true;
}

bool RobotBridge::SrvGetPose(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _mtx.lock();
  ROS_INFO("[robot_bridge] Calling SrvGetPose!\n");

  double pose[7];
  _robot.getPose(pose);

  ROS_INFO("Writing pose to the file..\n");

  ofstream fp;
  fp.open(_pose_feedback_file_path);

  if (!fp) {
    cerr << "Unable to open file for fp'.";
    exit(1); // terminate with error
  }
  cout << "Pose now: ";
  for (int i = 0; i < 7; ++i) {
    fp << pose[i] << " ";
    cout << pose[i] << ", ";
  }
  cout << endl;
  fp.close();

  cout << "[robot_bridge] GetPose finished. " << endl;

  _mtx.unlock();
  return true;
}

bool RobotBridge::SrvHybridServo(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _mtx.lock();
  ROS_INFO("[robot_bridge] Calling SrvHybridServo!\n");

  /**
   * Read target pose from file.
   */
  ROS_INFO("Reading hybrid action from the file..\n");

  ifstream fp;
  fp.open(_hybrid_action_file_path);

  if (!fp) {
    cerr << "Unable to open file for fp'.";
        exit(1); // terminate with error
  }
  int n_af, n_av;
  double duration_s;
  double pose_set[7], force_set[6];
  Matrix6d R_a = Matrix6d::Zero();
  fp >> n_af >> n_av >> duration_s;
  for (int i = 0; i < 7; ++i) fp >> pose_set[i];
  for (int i = 0; i < 6; ++i) fp >> force_set[i];
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      fp >> R_a(i, j);
  fp.close();

  // cout << "The hybrid action:\n";
  // cout << "n_af: " << n_af << ", n_av: " << n_av << ", duration: " << duration_s << endl;
  // cout << "pose_set: ";
  // for (int i = 0; i < 7; ++i) cout << pose_set[i] << " ";
  // cout << endl << "force_set: ";
  // for (int i = 0; i < 6; ++i) cout << force_set[i] << " ";
  // cout << endl << "R_a:" << endl;
  // cout << R_a.format(MatlabFmt) << endl;
  // cout << "Press Enter to start.";
  // getchar();

  /**
   * Execute the HFVC
   */

  _controller.ExecuteHFVC(n_af, n_av, R_a, pose_set, force_set,
    HS_STOP_AND_GO, _main_loop_rate, duration_s);

  cout << "[robot_bridge] HybridServo finished. " << endl;

  _mtx.unlock();
  return true;
}
