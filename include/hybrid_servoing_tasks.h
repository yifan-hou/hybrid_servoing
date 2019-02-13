#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>
#include <forcecontrol/utilities.h>


class HybridServoingTask {
public:
  virtual ~HybridServoingTask() {}

  virtual bool initialize(const std::string& file_name,
    const int main_loop_rate) = 0;
  virtual bool run() = 0;

  int main_loop_rate_;
    // setters & getters
    // void SetMainLoopRate(const int rate) {main_loop_rate_ = rate; }
    // int GetMainLoopRate() const { return main_loop_rate_; }
protected:
  HybridServoingTask() {}

private:
};



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





// class LeveringUp : public HybridServoingTask {
// public:
//   LeveringUp(ForceControlHardware *robot,
//     ForceControlController *controller);
//   ~LeveringUp() override;

//   bool initialize(const std::string& file_name,
//     const int main_loop_rate) override;
//   bool run() override;

// private:
//   LeveringUp(){}
//     // task description variables
//   ForceControlHardware *robot_;
//   ForceControlController *controller_;

//   int N_TRJ_;
//   float kTimeStepSec_;
//   Eigen::MatrixXf p_WH_traj_;
//   Eigen::MatrixXf p_WO_traj_;
//   Eigen::MatrixXf q_WO_traj_;
// };

