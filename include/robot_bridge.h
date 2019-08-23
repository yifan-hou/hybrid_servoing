typedef std::chrono::high_resolution_clock Clock;

class RobotBridge {
public:
  RobotBridge();
  ~RobotBridge();

  int init(ros::NodeHandle *ros_handle, Clock time0);

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
       std_srvs::Empty::Response &res);


  // parameters from parameter server
  int _main_loop_rate;
  double _force_touch_threshold;
  double _default_pose[7];
  double _kVelMaxTrans;
  double _kAccMaxTrans;
  double _kVelMaxRot;
  double _kAccMaxRot;
  std::string _pose_set_file_path;
  std::string _velocity_set_file_path;
  std::string _task_data_file_path;
  // std::string POSE_FEEDBACK_FILE_PATH;

  // Handles
  ros::NodeHandle *_ros_handle_p;
  ForceControlHardware _robot;
  ForceControlController _controller;

  // misc
  mutex _mtx;

};