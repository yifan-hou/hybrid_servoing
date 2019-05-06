#pragma once

#include <hybrid_servoing_tasks.h>


class ReOrienting : public HybridServoingTask {
public:
  ReOrienting(ForceControlHardware *robot,
    ForceControlController *controller);
  ~ReOrienting() override;

  bool initialize(const std::string& file_name,
    const int main_loop_rate) override;
  bool run() override;

private:
  ReOrienting(){}
  // trajectory description variables
  bool *rtype_;
  bool *stuck_;
  Eigen::MatrixXf q_WG_;
  Eigen::MatrixXf p_WG_;
  // parameters
  int N_TRJ_;
  float grp_width_;
  float finger_radius_;
  float finger_thickness_;
  float open_finger_incre_for_pivot_mm_;
  float safe_Z_min_;

  bool traj_is_read_;
  ForceControlHardware *robot_;
  ForceControlController *controller_;
};
