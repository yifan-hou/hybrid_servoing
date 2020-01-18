#pragma once

#include "robot_bridge.h"

class DiamondTaskServer : public RobotBridge {
public:
  DiamondTaskServer(){}
  ~DiamondTaskServer(){}

  bool initDiamondTaskServer();
  bool hostServices();

  // service callback
  bool SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res);

};