#pragma once

#include "robot_bridge.h"

class EngagingTaskServer : public RobotBridge {
public:
  EngagingTaskServer(){}
  ~EngagingTaskServer(){}

  bool initEngagingTaskServer();
  bool hostServices();

  // service callback
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

private:
  // state
  Eigen::Vector3d _F_W; // direction of resistance force

  // parameters
  int _pool_size;
  double _kTimeStepSec;
  int _kNumOfTimeSteps;
  double _kGoalVelocityM; // m/s
  double _kNormalForceMin;
  Eigen::Matrix<double, 6, 6> force_scale_matrix_inv_;

  /**
   * The data from force control is scaled. Here we need to know the scale to
   * recover the original data
   */
  Eigen::Matrix<double, 6, 6> _force_scale_matrix_inv;
  double _v_singular_value_threshold;
  double _f_singular_value_threshold;

};