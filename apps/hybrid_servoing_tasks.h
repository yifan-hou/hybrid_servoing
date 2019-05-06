/// The base class for all tasks.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <yifanlibrary/utilities.h>

#include <forcecontrol/forcecontrol_hardware.h>
#include <forcecontrol/forcecontrol_controller.h>

#define PIf 3.1416f

const static Eigen::IOFormat MatlabFmt(
    Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");


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
