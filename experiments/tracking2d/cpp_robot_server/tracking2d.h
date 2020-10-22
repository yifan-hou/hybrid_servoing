#pragma once

#include "robot_bridge.h"

class Tracking2DTaskServer : public RobotBridge {
public:
  Tracking2DTaskServer(){}
  ~Tracking2DTaskServer(){}

  bool initTracking2DTaskServer();
  bool hostServices();

  // service callback
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
};