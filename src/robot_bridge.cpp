#include <iostream>
#include <fstream>
#include <unistd.h>
#include <mutex>

#include <Eigen/Geometry>
#include <std_srvs/Empty.h>

#include "RobotUtilities/utilities.h"

// const string _pose_set_file_path = "/usr0/home/yifanh/Git/"
//     "hybrid-force-velocity-control/results/pose_set.txt";
// const string _velocity_set_file_path = "/usr0/home/yifanh/Git/"
//     "hybrid-force-velocity-control/results/velocity_set.txt";
// const string _pose_feedback_file_path = "/usr0/home/yifanh/Git/"
//     "hybrid-force-velocity-control/results/pose_feedback.txt";
// const string _task_data_file_path = "/usr0/home/yifanh/Git/"
//     "hybrid-force-velocity-control/results/";

using namespace std;
using namespace Eigen;

int RobotBridge::init(ros::NodeHandle *ros_handle, Clock time0){
  ros_handle_p = ros_handle;

  ROS_INFO_STREAM("robot_bridge server is starting");

  /**
   * Force control parameters
   */
  ros_handle->param(std::string("/forcecontrol/main_loop_rate"), _main_loop_rate, 500);
  if (!ros_handle->hasParam("/forcecontrol/main_loop_rate"))
    ROS_WARN_STREAM("Parameter [/forcecontrol/main_loop_rate] not found");

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
  // ros_handle->param(std::string("/robot_bridge/pose_feedback_file_path"), _pose_feedback_file_path, std::string("/tmp"));
  // if (!ros_handle->hasParam("/robot_bridge/pose_feedback_file_path"))
  //   ROS_WARN_STREAM("Parameter [/robot_bridge/pose_feedback_file_path] not found");
  ros_handle->param(std::string("/robot_bridge/task_data_file_path"), _task_data_file_path, std::string("/tmp"));
  if (!ros_handle->hasParam("/robot_bridge/task_data_file_path"))
    ROS_WARN_STREAM("Parameter [/robot_bridge/task_data_file_path] not found");
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
    // Initialize robot and forcecontroller
    // --------------------------------------------------------
    _robot.init(*ros_handle, time0); // robot must be initialized before controller
    _controller.init(*ros_handle, &robot, time0);
    _controller.reset();
    // --------------------------------------------------------
    // Establish Services
    // --------------------------------------------------------
    ros::ServiceServer reset_service            = ros_handle->advertiseService("reset", RobotBridge::SrvReset, this);
    ros::ServiceServer move_tool_service        = ros_handle->advertiseService("move_tool", RobotBridge::SrvMoveTool, this);
    // ros::ServiceServer move_hybrid_service      = ros_handle->advertiseService("move_hybrid", RobotBridge::SrvHybridServo, this);
    ros::ServiceServer move_until_touch_service = ros_handle->advertiseService("move_until_touch", RobotBridge::SrvMoveUntilTouch, this);
    ros::ServiceServer get_pose_service         = ros_handle->advertiseService("get_pose", RobotBridge::SrvGetPose, this);

    cout << endl << "[robot_bridge] Initialization is done. Service servers are listening.." << endl;
    ros::spin();

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);

    return 0;
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
    _robot.setControl(pose_traj.col(i).data());
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
  ROS_INFO("Reading pose from the file..\n");

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
  UT::MotionPlanningTrapezodial(pose, pose_set, _kAccMaxTrans, _kVelMaxTrans,
    _kAccMaxRot, _kVelMaxRot, (double)_main_loop_rate, &pose_traj);
  int num_of_steps = pose_traj.cols();

  ros::Rate pub_rate(_main_loop_rate);
  for (int i = 0; i < num_of_steps; ++i) {
    _robot.setControl(pose_traj.col(i).data());
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
  Eigen::Vector3d v_set;
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
  Eigen::Vector3d p_set, v_delta;
  p_set << pose[0], pose[1], pose[2];
  v_delta = v_set/double(main_loop_rate);
  ros::Rate pub_rate(main_loop_rate);

  double wrench[6];
  double wrench_safety_check[3] = {0, 0, 0};
  int safety_count = 0;
  while (true) {
    // check force feedback
    _robot.getWrench(wrench);
    cout << "wrench: "<< wrench[0] << ", " << wrench[1] << ", " << wrench[2]
        << endl;
    double force_mag = sqrt(wrench[0]*wrench[0] + wrench[1]*wrench[1] +
        wrench[2]*wrench[2]);
    double torque_mag = sqrt(wrench[3]*wrench[3] + wrench[4]*wrench[4] +
        wrench[5]*wrench[5]);
    if ((force_mag > _force_touch_threshold) || (torque_mag > 0.4f)) {
      cout << "[MoveUntilTouch] Touched!" << endl;
      break;
    }

    double change_of_force = abs(wrench[0] - wrench_safety_check[0]) +
        abs(wrench[1] - wrench_safety_check[1]) +
        abs(wrench[2] - wrench_safety_check[2]);
    if (change_of_force < 1e-4) safety_count ++;
    else safety_count = 0;

    if (safety_count > 20) {
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

    _robot.setControl(pose);
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
  fp.open(_POSE_FEEDBACK_FILE_PATH);

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

// bool SrvHybridServo(std_srvs::Empty::Request  &req,
//     std_srvs::Empty::Response &res) {
//   _mtx.lock();
//   ROS_INFO("[robot_bridge] Calling SrvHybridServo!\n");

//   PlaneEngaging task(&robot, &controller);
//   // LeveringUp task(&robot, &controller);
//   ROS_INFO("Reading pose from the file..\n");

//   task.initialize(_task_data_file_path, main_loop_rate, *ros_handle_p);
//   task.run();

//   cout << "[robot_bridge] SrvHybridServo finished. " << endl;

//   _mtx.unlock();
//   return true;
// }
