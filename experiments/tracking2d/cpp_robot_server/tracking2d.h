#pragma once

#include "robot_bridge.h"

class Tracking2DTaskServer : public RobotBridge {
public:
  Tracking2DTaskServer(){}
  ~Tracking2DTaskServer(){}

  bool initTracking2DTaskServer();
  bool hostServices();

  // service callback

  /**
   * Read the whole, multi-pieces motion plan from file, reset piece counter
   * to zero.
   * File name is specified in config yaml.
   */
  bool SrvReadMotionPlan(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);
  /**
   * Execute the current piece of motion plan, advance to the next piece.
   */
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);
  /**
   * Wrapper for parent SrvMoveUntilTouch. Compute direction from motion plan.
   */
  bool SrvPreEngage(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);
  bool SrvEngage(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);
  bool SrvDisengage(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);
private:

  // parameters
  // see yaml file for explanation
  double _kTransResMM;
  double _kTransVelMM;
  double _kTransMaxPerFrameMM;
  double _k2DTo3DOffsetX;
  double _k2DTo3DOffsetY;
  double _k2DTo3DOffsetZ;
  std::string _data_folder_path;
  std::string _data_filename;
  bool _XZ_plane;
  // data
  std::vector<Eigen::MatrixXd> _motion_plans;
  std::vector<Eigen::Vector2d> _contact_normal_engaging;
  std::vector<Eigen::Vector2d> _contact_normal_disengaging;
  // internal data
  int _traj_piece_count;
  // misc
};