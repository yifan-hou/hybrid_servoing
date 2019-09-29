#pragma once

#include "robot_bridge.h"

class EngagingTaskServer : public RobotBridge {
public:
  EngagingTaskServer(){}
  ~EngagingTaskServer(){}

  bool initEngagingTaskServer();
  bool hostServices();

  // service callback
  bool SrvCompliantEngage(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

private:
  // state
  Eigen::Vector3d _F_W; // direction of resistance force

  // parameters
  int _pool_size;
  double _kTimeStepSec;
  double _kGoalVelocityM; // m/s
  double _kNormalForceMax;
  double _kEngagingForce;
  Eigen::Matrix<double, 6, 6> force_scale_matrix_inv_;
  Eigen::Vector3d _goal_W1;
  Eigen::Vector3d _goal_W2;
  std::string _timesteps_file_path;

  /**
   * The data from force control is scaled. Here we need to know the scale to
   * recover the original data
   */
  Eigen::Matrix<double, 6, 6> _force_scale_matrix_inv;
  double _v_singular_value_threshold;
  double _f_singular_value_threshold;

};