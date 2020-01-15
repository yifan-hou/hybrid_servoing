#include "diamond.h"

#include <fstream>
#include <cstdlib>  // For srand() and rand()

#include <Eigen/SVD>
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

#include <solvehfvc.h>

#define PI 3.14159265

using namespace RUT;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using Eigen::Vector2d;
using Eigen::Matrix2d;

bool DiamondTaskServer::initDiamondTaskServer() {
  ROS_INFO_STREAM("Diamond server is starting");
  if (_ros_handle_p == nullptr) {
    ROS_ERROR_STREAM("[DiamondTaskServer] You must call .init() before .initDiamondTaskServer().");
    exit(1);
  }

  _ros_handle_p->param(string("/task/v_singular_value_threshold"),
      _v_singular_value_threshold, 0.1);
  _ros_handle_p->param(string("/task/f_singular_value_threshold"),
      _f_singular_value_threshold, 0.1);
  if (!_ros_handle_p->hasParam("/task/v_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter [/task/v_singular_value_threshold] not found");
  if (!_ros_handle_p->hasParam("/task/f_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter [/task/f_singular_value_threshold] not found");

  _ros_handle_p->param(std::string("/task/time_step"), _kTimeStepSec, 0.1);
  if (!_ros_handle_p->hasParam("/task/time_step"))
    ROS_WARN_STREAM("Parameter [/task/time_step] not found");
  _ros_handle_p->param(std::string("/task/number_of_time_steps"), _kNumOfTimeSteps, 30);
  if (!_ros_handle_p->hasParam("/task/number_of_time_steps"))
    ROS_WARN_STREAM("Parameter [/task/number_of_time_steps] not found");
  _ros_handle_p->param(std::string("/task/goal_veloicty_meter"), _kGoalVelocityM, 0.01);
  if (!_ros_handle_p->hasParam("/task/goal_veloicty_meter"))
    ROS_WARN_STREAM("Parameter [/task/goal_veloicty_meter] not found");
  _ros_handle_p->param(std::string("/task/engage_step_length"), _kEngageStepLength, 0.001);
  if (!_ros_handle_p->hasParam("/task/engage_step_length"))
    ROS_WARN_STREAM("Parameter [/task/engage_step_length] not found");
  _ros_handle_p->param(std::string("/task/min_normal_force"), _kNormalForceMin, 4.0);
  if (!_ros_handle_p->hasParam("/task/min_normal_force"))
    ROS_WARN_STREAM("Parameter [/task/min_normal_force] not found");

  _F_W(0) = 0;
  _F_W(1) = 1;
  _F_W(2) = 0;

  return true;
}

bool DiamondTaskServer::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &DiamondTaskServer::SrvReset, (RobotBridge*)this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &DiamondTaskServer::SrvMoveTool, (RobotBridge*)this);
  ros::ServiceServer move_until_touch_service = _ros_handle_p->advertiseService("move_until_touch", &DiamondTaskServer::SrvMoveUntilTouch, (RobotBridge*)this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &DiamondTaskServer::SrvGetPose, (RobotBridge*)this);
  ros::ServiceServer hybrid_servo_service     = _ros_handle_p->advertiseService("hybrid_servo", &DiamondTaskServer::SrvHybridServo, (RobotBridge*)this);
  // ros::ServiceServer execute_task_service     = _ros_handle_p->advertiseService("execute_task", &DiamondTaskServer::SrvExecuteTask, this);

  cout << endl << "[DiamondTaskServer] Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[DiamondTaskServer] Service servers stopped." << endl);
  return true;
}

bool DiamondTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  cout << "Calling execution. To be implemented." << endl;
  return true;
}