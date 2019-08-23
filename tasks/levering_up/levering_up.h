#pragma once

#include <robot_bridge.h>

class LeveringUpTaskServer : public RobotBridge {
public:
  LeveringUpTaskServer();
  ~LeveringUpTaskServer();

  bool initLeveringUp();
  bool run();

private:
  LeveringUpTaskServer(){}

  // parameters
  int _main_loop_rate;
  float _kGoalTheta;
  float _kTimeStepSec;
  float _kObjectMass;
  float _kHandMass;
  float _kGravityConstant;
  float _kObjectLength;
  float _kObjectThickness;
  float _kFrictionCoefficientTable;
  float _kFrictionCoefficientHand;
  float _kFrictionCoefficientBin;
  float _kMinNormalForce;
  float _kMinNormalForceSliding;
  float _kMaxNormalForceSliding;
  float _kGoalRotationVelocity;
  Eigen::Vector2f _p_WH0;
  Eigen::Vector2f _p_WO0;
};