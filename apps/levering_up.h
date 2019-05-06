#pragma once

#include <hybrid_servoing_tasks.h>

class LeveringUp : public HybridServoingTask {
public:
  LeveringUp(ForceControlHardware *robot,
    ForceControlController *controller);
  ~LeveringUp() override;

  bool initialize(const std::string& file_name,
    const int main_loop_rate) override;
  bool run() override;

private:
  LeveringUp(){}
    // task description variables
  ForceControlHardware *robot_;
  ForceControlController *controller_;

  float kGoalTheta_;
  float kTimeStepSec_;
  float kObjectMass_;
  float kHandMass_;
  float kGravityConstant_;
  float kObjectLength_;
  float kObjectThickness_;
  float kFrictionCoefficientTable_;
  float kFrictionCoefficientHand_;
  float kFrictionCoefficientBin_;
  float kMinNormalForce_;
  float kMinNormalForceSliding_;
  float kMaxNormalForceSliding_;
  float kGoalRotationVelocity_;
  Eigen::Vector2f p_WH0_;
  Eigen::Vector2f p_WO0_;
};