#include "step_crawling.h"

#include <RobotUtilities/utilities.h>
#include <ati_netft/ati_netft.h>
#include <abb_egm/abb_egm.h>
#include <ur_socket/ur_socket.h>

using namespace std;
using namespace RUT;

int main(int argc, char* argv[]) {
  ROS_INFO_STREAM("Step crawling task server node starting");
  ros::init(argc, argv, "step_crawling_node");
  ros::NodeHandle hd;

  Clock::time_point time0 = std::chrono::high_resolution_clock::now();

  ATINetft ati;
  cout << "[test] initializing ft sensor:\n";
  ati.init(hd, time0);
  cout << "[test] initializing robot:\n";
  URSocket *robot = URSocket::Instance();
  robot->init(hd, time0);

  StepCrawlingTaskServer task_server;
  task_server.init(&hd, time0, &ati, robot);
  task_server.initStepCrawlingTaskServer();

  task_server.hostServices();

  ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
  return 0;
}