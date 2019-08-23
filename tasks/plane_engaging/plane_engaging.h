#pragma once

#include <deque>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <apps/hybrid_servoing_tasks.h>

class PlaneEngaging : public HybridServoingTask {
public:
  PlaneEngaging(ForceControlHardware *robot,
    ForceControlController *controller);
  ~PlaneEngaging() override;

  bool initialize(const std::string& file_name,
    const int main_loop_rate, ros::NodeHandle& root_nh) override;
  bool run() override;

private:
  PlaneEngaging(){}
  ForceControlHardware *robot_;
  ForceControlController *controller_;

  /* task description variables */
  int N_TRJ_;
  Eigen::MatrixXd G_;
  Eigen::VectorXd b_G_;
  double v_singular_value_threshold_;
  double f_singular_value_threshold_;
  Eigen::Matrix<double, 6, 6> force_scale_matrix_inv_;

  /* MISC */
  std::string folder_path_;
};