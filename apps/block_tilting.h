#pragma once

#include <hybrid_servoing_tasks.h>


class BlockTilting : public HybridServoingTask {
public:
  BlockTilting(ForceControlHardware *robot,
    ForceControlController *controller);
  ~BlockTilting() override;

  bool initialize(const std::string& file_name,
    const int main_loop_rate) override;
  bool run() override;

private:
  BlockTilting(){}
    // task description variables
  ForceControlHardware *robot_;
  ForceControlController *controller_;

  int N_TRJ_, kFrictionConeSides_;
  float kFrictionCoefficientTable_, kMinNormalForce_, kFrictionCoefficientHand_;
  float kTimeStepSec_;
  Eigen::Vector3f F_WGO_, F_WGH_;
  Eigen::Vector3f p_OHC_;
  Eigen::MatrixXf v_friction_directions_;
  Eigen::MatrixXf p_OTC_all_, p_WTC_all_;
  Eigen::VectorXf t_WG_;
  Eigen::MatrixXf p_WH_traj_;
  Eigen::MatrixXf p_WO_traj_;
  Eigen::MatrixXf q_WO_traj_;
};