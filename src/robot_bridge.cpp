#include <iostream>
#include <fstream>
#include <unistd.h>
#include <mutex>

#include <Eigen/Geometry>
#include <std_srvs/Empty.h>

#include "yifanlibrary/utilities.h"

#include "apps/plane_engaging.h"


const string POSE_SET_FILE_PATH = "/usr0/home/yifanh/Git/"
        "hybrid-force-velocity-control/results/pose_set.txt";
const string VELOCITY_SET_FILE_PATH = "/usr0/home/yifanh/Git/"
        "hybrid-force-velocity-control/results/velocity_set.txt";
const string POSE_FEEDBACK_FILE_PATH = "/usr0/home/yifanh/Git/"
        "hybrid-force-velocity-control/results/pose_feedback.txt";
const string HYBRID_SERVO_FILE_PATH = "/usr0/home/yifanh/Git/"
        "hybrid-force-velocity-control/results/";

using namespace std;
using namespace Eigen;

// parameters from parameter server
int main_loop_rate;
double force_touch_threshold;
double default_pose[7];

double kVelMaxTrans;
double kAccMaxTrans;
double kVelMaxRot;
double kAccMaxRot;

// other global variables
ros::NodeHandle *ros_handle_p;
mutex mtx;
ForceControlHardware robot;
ForceControlController controller;


bool SrvReset(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
    mtx.lock();
    ROS_INFO("[robot_bridge] Calling SrvReset!\n");

    double pose[7];
    robot.getPose(pose);

    MatrixXd pose_traj;
    UT::MotionPlanningTrapezodial(pose, default_pose, kAccMaxTrans, kVelMaxTrans,
            kAccMaxRot, kVelMaxRot, (double)main_loop_rate, &pose_traj);
    int num_of_steps = pose_traj.cols();

    cout << "Moving to reset location.." << endl;
    ros::Rate pub_rate(main_loop_rate);
    for (int i = 0; i < num_of_steps; ++i) {
        robot.setControl(pose_traj.col(i).data());
        pub_rate.sleep();
    }

    cout << "[robot_bridge] Reset is done." << endl;

    mtx.unlock();
    return true;
}

bool SrvMoveTool(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
    mtx.lock();
    ROS_INFO("[robot_bridge] Calling SrvMoveTool!\n");

    // --------------------------------------------------------
    // Read
    // --------------------------------------------------------
    ROS_INFO("Reading pose from the file..\n");

    ifstream fp;
    fp.open(POSE_SET_FILE_PATH);

    if (!fp)
    {
        cerr << "Unable to open file for fp'.";
        exit(1); // terminate with error
    }
    double pose_set[7];
    cout << "Pose_set: ";
    for (int i = 0; i < 7; ++i) {
        fp >> pose_set[i];
        cout << pose_set[i] << ", ";
    }
    cout << endl;
    fp.close();

    // --------------------------------------------------------
    // Move
    // --------------------------------------------------------
    double pose[7];
    robot.getPose(pose);

    MatrixXd pose_traj;
    UT::MotionPlanningTrapezodial(pose, pose_set, kAccMaxTrans, kVelMaxTrans,
            kAccMaxRot, kVelMaxRot, (double)main_loop_rate, &pose_traj);
    int num_of_steps = pose_traj.cols();

    ros::Rate pub_rate(main_loop_rate);
    for (int i = 0; i < num_of_steps; ++i) {
        robot.setControl(pose_traj.col(i).data());
        pub_rate.sleep();
    }
    cout << "[robot_bridge] MoveTool finished. " << endl;

    mtx.unlock();
    return true;
}


bool SrvMoveUntilTouch(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
    mtx.lock();
    ROS_INFO("[robot_bridge] Calling SrvMoveUntilTouch!\n");

    // --------------------------------------------------------
    // Read
    // --------------------------------------------------------
    ROS_INFO("Reading velocity from the file..\n");

    ifstream fp;
    fp.open(VELOCITY_SET_FILE_PATH);

    if (!fp)
    {
        cerr << "Unable to open file for fp'.";
        exit(1); // terminate with error
    }
    Eigen::Vector3d v_set;
    cout << "velocity_set: ";
    for (int i = 0; i < 3; ++i) {
        fp >> v_set[i];
        cout << v_set[i] << ", ";
    }
    cout << endl;
    fp.close();

    // --------------------------------------------------------
    // Move
    // --------------------------------------------------------
    double pose[7];
    robot.getPose(pose);
    Eigen::Vector3d p_set, v_delta;
    p_set << pose[0], pose[1], pose[2];
    v_delta = v_set/double(main_loop_rate);
    ros::Rate pub_rate(main_loop_rate);

    double wrench[6];
    double wrench_safety_check[3] = {0, 0, 0};
    int safety_count = 0;
    for (;;)
    {
        // check force feedback
        robot.getWrench(wrench);
        cout << "wrench: "<< wrench[0] << ", " << wrench[1] << ", " << wrench[2] << endl;
        double force_mag = sqrt(wrench[0]*wrench[0] +
                wrench[1]*wrench[1] +
                wrench[2]*wrench[2]);
        double torque_mag = sqrt(wrench[3]*wrench[3] +
                wrench[4]*wrench[4] +
                wrench[5]*wrench[5]);
        if ((force_mag > force_touch_threshold) || (torque_mag > 0.4f)) {
            cout << "[MoveUntilTouch] Touched!" << endl;
            break;
        }

        double change_of_force = abs(wrench[0] - wrench_safety_check[0]) +
                abs(wrench[1] - wrench_safety_check[1]) +
                abs(wrench[2] - wrench_safety_check[2]);
        if (change_of_force < 1e-4) safety_count ++;
        else safety_count = 0;

        if (safety_count > 20)
        {
            cout << "[MoveUntilTouch] FT sensor stopped!!" << endl;
            break;
        }
        wrench_safety_check[0] = wrench[0];
        wrench_safety_check[1] = wrench[1];
        wrench_safety_check[2] = wrench[2];

        p_set += v_delta;
        pose[0] = p_set(0);
        pose[1] = p_set(1);
        pose[2] = p_set(2);

        robot.setControl(pose);
        pub_rate.sleep();
    }

    cout << "[robot_bridge] MoveUntilTouch finished. " << endl;

    mtx.unlock();
    return true;
}


bool SrvGetPose(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
    mtx.lock();
    ROS_INFO("[robot_bridge] Calling SrvGetPose!\n");

    // --------------------------------------------------------
    // Read
    // --------------------------------------------------------
    double pose[7];
    robot.getPose(pose);

    ROS_INFO("Writing pose to the file..\n");

    ofstream fp;
    fp.open(POSE_FEEDBACK_FILE_PATH);

    if (!fp)
    {
        cerr << "Unable to open file for fp'.";
        exit(1); // terminate with error
    }
    cout << "Pose now: ";
    for (int i = 0; i < 7; ++i) {
        fp << pose[i] << " ";
        cout << pose[i] << ", ";
    }
    cout << endl;
    fp.close();

    cout << "[robot_bridge] GetPose finished. " << endl;

    mtx.unlock();
    return true;
}

// bool SrvMoveHybrid(std_srvs::Empty::Request  &req,
//                std_srvs::Empty::Response &res) {
//     mtx.lock();
//     ROS_INFO("[robot_bridge] Calling SrvHybridServo!\n");

//     // --------------------------------------------------------
//     // Read
//     // --------------------------------------------------------
//     ROS_INFO("Reading pose from the file..\n");

//     ifstream fp;
//     fp.open(HYBRID_SERVO_FILE_PATH);

//     if (!fp)
//     {
//         cerr << "Unable to open file for fp'.";
//         exit(1); // terminate with error
//     }

//     int n_av;
//     float linear_velocity;
//     float pose_set[7], array_R_a[9], force_set[6];
//     Eigen::Matrix3f R_a;

//     fp >> n_av;
//     cout << "n_av: " << n_av << endl;
//     fp >> linear_velocity;
//     cout << "linear_velocity: " << linear_velocity << " mm/s" << endl;
//     cout << "Pose_set:  ";
//     for (int i = 0; i < 7; ++i) {
//         fp >> pose_set[i];
//         cout << pose_set[i] << ", ";
//     }
//     cout << endl;
//     // make sure units are mm
//     assert(fabs(pose_set[0]) + fabs(pose_set[1]) + fabs(pose_set[2]) > 10);

//     cout << "R_a: ";
//     for (int i = 0; i < 9; ++i) {
//         fp >> array_R_a[i];
//     }
//     R_a << array_R_a[0], array_R_a[1], array_R_a[2],
//            array_R_a[3], array_R_a[4], array_R_a[5],
//            array_R_a[6], array_R_a[7], array_R_a[8];
//     cout << R_a << endl;

//     cout << "force_set: ";
//     for (int i = 0; i < 6; ++i) {
//         fp >> force_set[i];
//         cout << force_set[i] << ", ";
//     }
//     cout << endl;
//     fp.close();

//     int n_af = 3 - n_av;
//     // --------------------------------------------------------
//     // Move
//     // --------------------------------------------------------
//     controller.reset();
//     controller.updateAxis(R_a, n_af);
//     controller.setForce(force_set);
//     // controller.setPose(pose_set); // !! after setPose, must call update before updateAxis
//     //                               // so as to set right value for pose_command

//     float pose[7];
//     robot.getPose(pose);

//     float distance = sqrt( (pose[0] - pose_set[0])*(pose[0] - pose_set[0]) +
//             (pose[1] - pose_set[1])*(pose[1] - pose_set[1]) +
//             (pose[2] - pose_set[2])*(pose[2] - pose_set[2]));
//     float time_s = distance/linear_velocity;
//     int num_of_steps = int(time_s*float(main_loop_rate) + 0.5);

//     float **pose_traj = new float*[num_of_steps];
//     MotionPlanning(pose, pose_set, num_of_steps, pose_traj);
//     // main loop
//     ros::Duration period(EGM_PERIOD); // just for being compatible with ROS Control
//     ros::Rate pub_rate(main_loop_rate);
//     bool b_is_safe = true;
//     for (int i = 0; i < num_of_steps; ++i)
//     {
//         cout << "[Hybrid] update step " << i << " of " << num_of_steps;
//         cout << ", pose sent: " << pose_traj[i][0] << ", " << pose_traj[i][1];
//         cout << ", " << pose_traj[i][2] << endl;
//         // robot.setControl(pose_traj[i]);
//         controller.setPose(pose_traj[i]);

//         ros::Time time_now = ros::Time::now();
//         b_is_safe = controller.update(time_now, period);
//         if(!b_is_safe) break;
//         pub_rate.sleep();
//     }
//     for (int i = 0; i < num_of_steps; ++i)    delete [] pose_traj[i];
//     delete [] pose_traj;

//     if (b_is_safe)
//         cout << "[robot_bridge] MoveHybrid finished. " << endl;
//     else
//         cout << "[robot_bridge] MoveHybrid halted: forces too big. " << endl;


//     mtx.unlock();
//     return true;
// }

bool SrvHybridServo(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
    mtx.lock();
    ROS_INFO("[robot_bridge] Calling SrvHybridServo!\n");

    // --------------------------------------------------------
    // Read
    // --------------------------------------------------------
    PlaneEngaging task(&robot, &controller);
    // LeveringUp task(&robot, &controller);
    ROS_INFO("Reading pose from the file..\n");

    task.initialize(HYBRID_SERVO_FILE_PATH, main_loop_rate, *ros_handle_p);
    task.run();

    cout << "[robot_bridge] SrvHybridServo finished. " << endl;

    mtx.unlock();
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_bridge_node");
    ros::NodeHandle ros_handle;
    ros_handle_p = &ros_handle;

    ROS_INFO_STREAM("robot_bridge server is starting");

    ros_handle.param(std::string("/main_loop_rate"), main_loop_rate, 500);
    if (!ros_handle.hasParam("/main_loop_rate"))
        ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << main_loop_rate);

    ros_handle.param(std::string("/force_touch_threshold"), force_touch_threshold, 1.0);
    if (!ros_handle.hasParam("/force_touch_threshold"))
        ROS_WARN_STREAM("Parameter [/force_touch_threshold] not found, using default: " << force_touch_threshold);

    // Speed limit for motion planning
    ros_handle.param(string("/vel_max_translation"), kVelMaxTrans, 0.0);
    ros_handle.param(string("/acc_max_translation"), kAccMaxTrans, 0.0);
    ros_handle.param(string("/vel_max_rotation"), kVelMaxRot, 0.0);
    ros_handle.param(string("/acc_max_rotation"), kAccMaxRot, 0.0);
    if (!ros_handle.hasParam("/vel_max_translation"))
        ROS_WARN_STREAM("Parameter [/vel_max_translation] not found, using default: " << kVelMaxTrans);
    if (!ros_handle.hasParam("/acc_max_translation"))
        ROS_WARN_STREAM("Parameter [/acc_max_translation] not found, using default: " << kAccMaxTrans);
    if (!ros_handle.hasParam("/vel_max_rotation"))
        ROS_WARN_STREAM("Parameter [/vel_max_rotation] not found, using default: " << kVelMaxRot);
    if (!ros_handle.hasParam("/acc_max_rotation"))
        ROS_WARN_STREAM("Parameter [/acc_max_rotation] not found, using default: " << kAccMaxRot);

    if (!ros_handle.hasParam("/robot_bridge/default_pose"))
    {
        ROS_WARN_STREAM("Parameter [/robot_bridge/default_pose] not found!");
        return -1;
    }
    ros_handle.param(std::string("/robot_bridge/default_pose/x"), default_pose[0], 40.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/y"), default_pose[1], 356.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/z"), default_pose[2], 400.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/qw"), default_pose[3], 0.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/qx"), default_pose[4], 0.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/qy"), default_pose[5], 1.0);
    ros_handle.param(std::string("/robot_bridge/default_pose/qz"), default_pose[6], 0.0);

    // --------------------------------------------------------
    // Initialize robot and forcecontroller
    // --------------------------------------------------------
    std::chrono::high_resolution_clock::time_point TheTime0;
    TheTime0 = std::chrono::high_resolution_clock::now();

    robot.init(ros_handle, TheTime0); // robot must be initialized before controller
    controller.init(ros_handle, &robot, TheTime0);
    controller.reset();
    // --------------------------------------------------------
    // Establish Services
    // --------------------------------------------------------
    ros::ServiceServer reset_service            = ros_handle.advertiseService("reset", SrvReset);
    ros::ServiceServer move_tool_service        = ros_handle.advertiseService("move_tool", SrvMoveTool);
    // ros::ServiceServer move_hybrid_service      = ros_handle.advertiseService("move_hybrid", SrvMoveHybrid);
    ros::ServiceServer move_hybrid_service      = ros_handle.advertiseService("move_hybrid", SrvHybridServo);
    ros::ServiceServer move_until_touch_service = ros_handle.advertiseService("move_until_touch", SrvMoveUntilTouch);
    ros::ServiceServer get_pose_service         = ros_handle.advertiseService("get_pose", SrvGetPose);

    cout << endl << "[robot_bridge] Initialization is done. Service servers are listening.." << endl;
    ros::spin();

    ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);

    return 0;
}
