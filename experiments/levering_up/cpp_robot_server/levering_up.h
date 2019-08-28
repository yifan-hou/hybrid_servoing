#pragma once

#include "robot_bridge.h"

class LeveringUpTaskServer : public RobotBridge {
public:
  LeveringUpTaskServer(){}
  ~LeveringUpTaskServer(){}

  bool initLeveringUpTaskServer();
  bool hostServices();

  // service callback
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

private:

  // parameters
  double _kGoalThetaDeg;
  double _kTimeStepSec;
  double _kObjectMass;
  double _kHandMass;
  double _kGravityConstant;
  double _kObjectLength;
  double _kObjectThickness;
  double _kFrictionCoefficientTable;
  double _kFrictionCoefficientHand;
  double _kFrictionCoefficientBin;
  double _kMinNormalForceSticking;
  double _kMinNormalForceSliding;
  double _kMaxNormalForceSliding;
  double _kGoalRotationVelocity;
  double _kHandHeight0;
};