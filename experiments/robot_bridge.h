#include <chrono>
#include <mutex>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <force_control/force_control_hardware.h>
#include <force_control/force_control_controller.h>

typedef std::chrono::high_resolution_clock Clock;

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");

class RobotBridge {
public:
  RobotBridge();
  ~RobotBridge();

  int init(ros::NodeHandle *ros_handle, Clock::time_point time0, FTInterfaces *ft,
      RobotInterfaces *robot);
  bool hostServices();

  // service call-backs
  bool SrvReset(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);
  bool SrvMoveTool(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);
  bool SrvMoveUntilTouch(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);
  bool SrvGetPose(std_srvs::Empty::Request  &req,
       std_srvs::Empty::Response &res);
  bool SrvHybridServo(std_srvs::Empty::Request  &req,
       std_srvs::Empty::Response &res); // to be implemented


  // parameters from parameter server
  int _main_loop_rate;
  double _force_touch_threshold;
  double *_default_pose;
  double _kVelMaxTrans;
  double _kAccMaxTrans;
  double _kVelMaxRot;
  double _kAccMaxRot;
  bool _test_mode;
  std::string _pose_set_file_path;
  std::string _velocity_set_file_path;
  std::string _task_data_file_path;
  std::string _pose_feedback_file_path;
  std::string _hybrid_action_file_path;

  // Handles
  ros::NodeHandle *_ros_handle_p = nullptr;
  ForceControlHardware _robot;
  ForceControlController _controller;

  // misc
  std::mutex _mtx;

};