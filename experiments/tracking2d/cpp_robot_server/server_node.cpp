#include "tracking2d.h"

#include <RobotUtilities/utilities.h>
#include <ati_netft/ati_netft.h>
#include <abb_egm/abb_egm.h>
#include <ur_socket/ur_socket.h>

using namespace std;
using namespace RUT;

int main(int argc, char* argv[]) {
  ROS_INFO_STREAM("Tracking2D task server node starting");
  ros::init(argc, argv, "tracking2d_node");
  ros::NodeHandle hd;

  Clock::time_point time0 = std::chrono::high_resolution_clock::now();

  ATINetft ati;
  cout << "[test] initializing ft sensor:\n";
  ati.init(hd, time0);
  cout << "[test] initializing robot:\n";
  // URSocket *robot = URSocket::Instance();
  // robot->init(hd, time0);
  ABBEGM *robot = ABBEGM::Instance();
  robot->init(hd, time0);

  Tracking2DTaskServer task_server;
  task_server.init(&hd, time0, &ati, robot);
  task_server.initTracking2DTaskServer();

  task_server.hostServices();

  ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
  return 0;
}