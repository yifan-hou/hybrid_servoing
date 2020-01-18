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