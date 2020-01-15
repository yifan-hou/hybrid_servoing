#pragma once

#include "robot_bridge.h"

class DiamondTaskServer : public RobotBridge {
public:
  DiamondTaskServer(){}
  ~DiamondTaskServer(){}

  bool initDiamondTaskServer();
  bool hostServices();

  // service callback
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

private:
  // state
  Eigen::Vector3d _F_W; // direction of resistance force

  // parameters
  double _kTimeStepSec;
  int _kNumOfTimeSteps;
  double _kGoalVelocityM; // m/s
  double _kNormalForceMin;
  double _kEngageStepLength;
  /**
   * The data from force_control is scaled. Here we need to know the scale to
   * recover the original data
   */
  // Eigen::Matrix<double, 6, 6> _force_scale_matrix_inv;
  double _v_singular_value_threshold;
  double _f_singular_value_threshold;

};