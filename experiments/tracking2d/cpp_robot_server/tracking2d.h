#pragma once

#include "robot_bridge.h"

class Tracking2DTaskServer : public RobotBridge {
public:
  Tracking2DTaskServer(){}
  ~Tracking2DTaskServer(){}

  bool initTracking2DTaskServer();
  bool hostServices();

  // service callback
  bool SrvReadMotionPlan(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res)
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

private:

  // parameters
  // see yaml file for explanation
  double _kTransResMM;
  double _kTransVelMM;
  double _kTransMaxPerFrameMM;
  double _kPlanYOffsetX;
  double _kPlanYOffsetY;
  std::string _data_filename;
  // data
  std::vector<Eigen::MatrixXd> motion_plans;
  std::vector<Eigen::Vector2d> contact_normal_engaging;
  std::vector<Eigen::Vector2d> contact_normal_disengaging;
};