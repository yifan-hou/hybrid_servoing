#include "step_crawling.h"

#include <fstream>
#include <cstdlib>  // For srand() and rand()

#include <Eigen/SVD>
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

#include <solvehfvc.h>

#define PI 3.14159265

using namespace RUT;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using Eigen::Vector2d;
using Eigen::Matrix2d;

bool StepCrawlingTaskServer::initStepCrawlingTaskServer() {
  ROS_INFO_STREAM("Step crawling server is starting");
  if (_ros_handle_p == nullptr) {
    ROS_ERROR_STREAM("[StepCrawlingTaskServer] You must call .init() before .initStepCrawlingTaskServer().");
    exit(1);
  }

  _ros_handle_p->param(string("/task/v_singular_value_threshold"),
      _v_singular_value_threshold, 0.1);
  _ros_handle_p->param(string("/task/f_singular_value_threshold"),
      _f_singular_value_threshold, 0.1);
  if (!_ros_handle_p->hasParam("/task/v_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter [/task/v_singular_value_threshold] not found");
  if (!_ros_handle_p->hasParam("/task/f_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter [/task/f_singular_value_threshold] not found");

  _ros_handle_p->param(std::string("/task/time_step"), _kTimeStepSec, 0.1);
  if (!_ros_handle_p->hasParam("/task/time_step"))
    ROS_WARN_STREAM("Parameter [/task/time_step] not found");
  _ros_handle_p->param(std::string("/task/number_of_time_steps"), _kNumOfTimeSteps, 30);
  if (!_ros_handle_p->hasParam("/task/number_of_time_steps"))
    ROS_WARN_STREAM("Parameter [/task/number_of_time_steps] not found");
  _ros_handle_p->param(std::string("/task/goal_veloicty_meter"), _kGoalVelocityM, 0.01);
  if (!_ros_handle_p->hasParam("/task/goal_veloicty_meter"))
    ROS_WARN_STREAM("Parameter [/task/goal_veloicty_meter] not found");
  _ros_handle_p->param(std::string("/task/engage_step_length"), _kEngageStepLength, 0.001);
  if (!_ros_handle_p->hasParam("/task/engage_step_length"))
    ROS_WARN_STREAM("Parameter [/task/engage_step_length] not found");
  _ros_handle_p->param(std::string("/task/min_normal_force"), _kNormalForceMin, 4.0);
  if (!_ros_handle_p->hasParam("/task/min_normal_force"))
    ROS_WARN_STREAM("Parameter [/task/min_normal_force] not found");

  _F_W(0) = 0;
  _F_W(1) = 1;
  _F_W(2) = 0;

  return true;
}

bool StepCrawlingTaskServer::hostServices() {
  // --------------------------------------------------------
  // Establish Services
  // --------------------------------------------------------
  ros::ServiceServer reset_service            = _ros_handle_p->advertiseService("reset", &StepCrawlingTaskServer::SrvReset, (RobotBridge*)this);
  ros::ServiceServer move_tool_service        = _ros_handle_p->advertiseService("move_tool", &StepCrawlingTaskServer::SrvMoveTool, (RobotBridge*)this);
  ros::ServiceServer move_until_touch_service = _ros_handle_p->advertiseService("move_until_touch", &StepCrawlingTaskServer::SrvMoveUntilTouch, (RobotBridge*)this);
  ros::ServiceServer get_pose_service         = _ros_handle_p->advertiseService("get_pose", &StepCrawlingTaskServer::SrvGetPose, (RobotBridge*)this);
  ros::ServiceServer execute_task_service     = _ros_handle_p->advertiseService("execute_task", &StepCrawlingTaskServer::SrvExecuteTask, this);

  cout << endl << "[StepCrawlingTaskServer] Service servers are listening.." << endl;
  ros::spin();

  ROS_INFO_STREAM(endl << "[StepCrawlingTaskServer] Service servers stopped." << endl);
  return true;
}

bool StepCrawlingTaskServer::SrvExecuteTask(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res) {
  _controller.reset();

  if (_controller._f_queue.size() < 50) {
    cout << "Run update() for " << _main_loop_rate << " frames:" << endl;
    // first, run update for 1s to populate the data deques
    ros::Rate pub_rate(_main_loop_rate);
    for (int i = 0; i < _main_loop_rate; ++i) {
      bool b_is_safe = _controller.update();
      // if(!b_is_safe) break;
      pub_rate.sleep();
    }
    cout << "Done." << endl;
  }

  Timer timer;
  std::srand(std::time(0));
  MatrixXd f_data, v_data;
  for (int fr = 0; fr < _kNumOfTimeSteps; ++fr) {
    timer.tic();
    /**
     * Estimate Natural Constraints
     */
    // get the weighted data
    f_data = MatrixXd::Zero(3, _controller._f_queue.size());
    v_data = MatrixXd::Zero(3, _controller._v_queue.size());
    for (int i = 0; i < _controller._f_queue.size(); ++i)
      f_data.col(i) = _controller._f_queue[i].head(3) * _controller._f_weights[i];
    for (int i = 0; i < _controller._v_queue.size(); ++i)
      v_data.col(i) = _controller._v_queue[i].head(3) * _controller._v_weights[i];

    // SVD on velocity data
    Eigen::JacobiSVD<MatrixXd> svd_v(v_data.transpose(), Eigen::ComputeThinV);
    VectorXd sigma_v = svd_v.singularValues();
    int DimV = 0;
    for (int i = 0; i < 3; ++i)
      if (sigma_v(i) > _v_singular_value_threshold) DimV ++;

    // get a basis for row space of velocity data
    MatrixXd rowspace_v = svd_v.matrixV().leftCols(DimV);

    // filter out force data that:
    //    1. has a small weight
    std::vector<int> f_id;
    for (int i = 0; i < f_data.cols(); ++i) {
      double length = f_data.col(i).norm();
      if (length > 1.5) { // weighted length in newton
        f_id.push_back(i);
      }
    }

    double pose_fb[7], pose_set[7];
    _robot.getPose(pose_fb);
    Matrix3d R_WT = quat2SO3(pose_fb[3], pose_fb[4], pose_fb[5], pose_fb[6]);

    int f_data_length = f_id.size();
    if (f_data_length < 5) {
      ROS_WARN_STREAM("[StepCrawlingTaskServer] lacks significant force data");
      // stabilize onto the surface.
      Vector3d F_T = R_WT.transpose() * _F_W.normalized() * _kNormalForceMin;
      double force_temp[6] = {- F_T(0), - F_T(1), - F_T(2), 0, 0, 0};
      _controller.ExecuteHFVC(6, 0, Matrix6d::Identity(), pose_fb, force_temp,
          HS_CONTINUOUS, _main_loop_rate, 10*_kTimeStepSec);
      continue;
    }
    int DimF = 1;

    MatrixXd f_data_filtered = MatrixXd::Zero(3, f_data_length);
    for (int i = 0; i < f_data_length; ++i)
      f_data_filtered.col(i) = f_data.col(f_id[i]);

    MatrixXd f_data_selected;
    // SVD on filtered force data, get the prime direction
    Eigen::JacobiSVD<MatrixXd> svd_f(f_data_filtered.transpose(), Eigen::ComputeThinV);
    Vector3d F_T = svd_f.matrixV().col(0).normalized();
    // check sign
    MatrixXd check_sum = F_T.transpose() * f_data_filtered * VectorXd::Ones(f_data_length);
    if (check_sum(0,0) < 0) F_T = -F_T;
    MatrixXd Nf = F_T.transpose();

    /**
     * Infer Goal
     */
    // Transform Nf to world frame
    _F_W = R_WT * F_T;
    cout << "F_W: " << _F_W.transpose().format(MatlabFmt) << endl;
    Vector3d goal_direction_W = _F_W.cross(-Vector3d::UnitX());
    cout << "goal_direction_W: " << goal_direction_W.transpose().format(MatlabFmt) << endl;

    MatrixXd G = MatrixXd::Zero(1, 3);
    VectorXd b_G = VectorXd::Zero(1);
    G = (R_WT.transpose() * goal_direction_W).transpose();
    b_G(0) = _kGoalVelocityM;

    /**
     * Do Hybrid Servoing (3D)
     */
    HFVC action;
    int kDimActualized      = 3;
    int kDimUnActualized    = 0;
    int kDimSlidingFriction = 0;
    int kNumSeeds           = 3;
    int kDimLambda          = 1;
    int kPrintLevel         = 2;

    VectorXd F = VectorXd::Zero(3);
    MatrixXd Aeq(0, 1+3); // dummy
    VectorXd beq(0); // dummy
    MatrixXd A = MatrixXd::Zero(1, 1 + 3);
    VectorXd b_A = VectorXd::Zero(1);
    A(0, 0) = -1;
    b_A(0) = -_kNormalForceMin;

    cout << "Nf: " << Nf.format(MatlabFmt) << endl;
    cout << "G: " << G.format(MatlabFmt) << endl;
    cout << "b_G: " << b_G.format(MatlabFmt) << endl;
    cout << "F: " << F.format(MatlabFmt) << endl;
    cout << "Aeq: " << Aeq.format(MatlabFmt) << endl;
    cout << "beq: " << beq.format(MatlabFmt) << endl;
    cout << "A: " << A.format(MatlabFmt) << endl;
    cout << "b_A: " << b_A.format(MatlabFmt) << endl;

    solvehfvc(Nf, G, b_G, F, Aeq, beq, A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds, kPrintLevel,
      &action);

    double computation_time_ms = timer.toc();

    /**
     * Execute the hybrid action (6D)
     */
    Vector6d v_Tr = Vector6d::Zero();
    for (int i = 0; i < action.n_av; ++i)  v_Tr(i+action.n_af) = action.w_av(i);

    // Tool frame velocity
    Matrix6d R_a;
    R_a << action.R_a, Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();
    Matrix6d R_a_inv = R_a.inverse();
    Vector6d v_T = R_a_inv*v_Tr;

    // World frame velocity, set pose
    _robot.getPose(pose_fb);
    Matrix4d SE3_WT_fb = posemm2SE3(pose_fb);
    Matrix6d Adj_WT = SE32Adj(SE3_WT_fb);
    Vector6d v_W = Adj_WT*v_T;
    Matrix4d SE3_WT_command;
    SE3_WT_command = SE3_WT_fb + wedge6(v_W)*SE3_WT_fb*_kTimeStepSec;
    SE32Posemm(SE3_WT_command, pose_set);

    // Tool frame force
    Vector6d force_Tr_set = Vector6d::Zero();
    for (int i = 0; i < action.n_af; ++i)  force_Tr_set[i] = action.eta_af(i);
    Vector6d force_T = R_a_inv*force_Tr_set;
    Matrix6d Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));

    Vector6d force_W = Adj_TW.transpose() * force_T;
    cout << "force_Tr_set: " << force_Tr_set.transpose().format(MatlabFmt) << endl;
    cout << "force_T: " << force_T.transpose().format(MatlabFmt) << endl;
    printf("V in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v_W[0], v_W[1],
        v_W[2],v_W[3],v_W[4],v_W[5]);
    printf("F in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", force_W[0],
        force_W[1], force_W[2],force_W[3],force_W[4],force_W[5]);
    cout << "Computation Time: " << computation_time_ms << endl << endl;

    if (fr == _kNumOfTimeSteps-1) {
      // print to file
      std::ofstream fp;
      // f_queue
      fp.open(_task_data_file_path + "f_queue.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < _controller._f_queue.size(); ++i) {
        stream_array_in(fp, _controller._f_queue[i].data(), 3);
        fp << endl;
      }
      fp.close();
      // v_queue
      fp.open(_task_data_file_path + "v_queue.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < _controller._v_queue.size(); ++i) {
        stream_array_in(fp, _controller._v_queue[i].data(), 3);
        fp << endl;
      }
      fp.close();
      // f_weights
      fp.open(_task_data_file_path + "f_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < _controller._f_weights.size(); ++i)
        fp << _controller._f_weights[i] << endl;
      fp.close();

      // v_weights
      fp.open(_task_data_file_path + "v_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < _controller._v_weights.size(); ++i)
        fp << _controller._v_weights[i] << endl;
      fp.close();

      // f_data_filtered
      fp.open(_task_data_file_path + "f_data_filtered.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << f_data_filtered.transpose() << endl;
      fp.close();
      // f_data_selected
      fp.open(_task_data_file_path + "f_data_selected.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << f_data_selected.transpose() << endl;
      fp.close();
      // others
      fp.open(_task_data_file_path + "process.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << DimV << endl << DimF << endl;
      stream_array_in(fp, v_T.data(), 3);
      fp << endl;
      stream_array_in(fp, force_T.data(), 3);
      fp.close();
    }

    if (std::isnan(force_Tr_set[0])) {
      cout << "================== NaN =====================" << endl;
      cout << "Press Enter to continue.." << endl;
      getchar();
    }

    cout << "Current pose: " << pose_fb[0] << ", " << pose_fb[1] << ", " << pose_fb[2] << endl;
    cout << "Set pose: " << pose_set[0] << ", " << pose_set[1] << ", " << pose_set[2] << endl;
    cout << "Press Enter to begin motion: \n";
    // getchar();
    // cout << "motion begins:" << endl;
    _controller.ExecuteHFVC(action.n_af, action.n_av+3,
        R_a, pose_set, force_Tr_set.data(),
        HS_CONTINUOUS, _main_loop_rate, _kTimeStepSec);

  } // end for
  return true;
}