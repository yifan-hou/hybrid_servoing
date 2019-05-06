#pragma once

#include <deque>
#include <hybrid_servoing_tasks.h>


class PlaneEngaging : public HybridServoingTask {
public:
  PlaneEngaging(ForceControlHardware *robot,
    ForceControlController *controller);
  ~PlaneEngaging() override;

  bool initialize(const std::string& file_name,
    const int main_loop_rate) override;
  bool run() override;

private:
  PlaneEngaging(){}
  ForceControlHardware *robot_;
  ForceControlController *controller_;

  /* task description variables */
  MatrixXd G_;
  VectorXd b_G_;
  double kTimeStepSec_;
  double v_singular_value_threshold_;
  double f_singular_value_threshold_;

  // MatrixXd N_; // current estimation of N


};