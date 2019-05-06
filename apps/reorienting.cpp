#include <apps/reorienting.h>


#include "ZZJHand.h"


ReOrienting::ReOrienting(ForceControlHardware *robot,
    ForceControlController *controller) {
  robot_        = robot;
  controller_   = controller;
  traj_is_read_ = false;
}

ReOrienting::~ReOrienting() {
  if (!rtype_) delete [] rtype_;
  if (!stuck_) delete [] stuck_;
}

bool ReOrienting::initialize(const std::string& file_name,
    const int main_loop_rate) {
  // --------------------------------------------------------
  // Read trajectory files
  // --------------------------------------------------------
  ROS_INFO("Reading Trajectory files..\n");

  ifstream f_para;
  f_para.open(file_name + "/para.txt");
  if (!f_para) {
    cerr << "Unable to open file for f_para'.";
    return false; // terminate with error
  }
  f_para >> N_TRJ_;
  cout << "N_TRJ: " << N_TRJ_ << endl;
  f_para >> grp_width_;
  f_para >> finger_radius_;
  f_para >> finger_thickness_;
  f_para >> open_finger_incre_for_pivot_mm_;
  f_para >> safe_Z_min_;
  f_para.close();

  // trajectories
  ifstream f_rtype, f_stuck, f_q_WG, f_p_WG;
  f_rtype.open(file_name + "/rtype.txt");
  f_stuck.open(file_name + "/stuck.txt");
  f_q_WG.open(file_name + "/q_WG.txt");
  f_p_WG.open(file_name + "/p_WG.txt");

  if (!f_rtype) {
    cerr << "Unable to open file for f_rtype'.";
    return false; // terminate with error
  }
  if (!f_stuck) {
    cerr << "Unable to open file for f_stuck'.";
    return false; // terminate with error
  }
  if (!f_q_WG) {
    cerr << "Unable to open file for f_q_WG'.";
    return false; // terminate with error
  }
  if (!f_p_WG) {
    cerr << "Unable to open file for p_WG'.";
    return false; // terminate with error
  }


  rtype_ = new bool[N_TRJ_];
  stuck_ = new bool[N_TRJ_];
  q_WG_.resize(4, N_TRJ_);
  p_WG_.resize(3, N_TRJ_);

  for (int i = 0; i < N_TRJ_; ++i) {
    f_rtype >> rtype_[i];
    f_stuck >> stuck_[i];
    f_q_WG >> q_WG_(0, i) >> q_WG_(1, i) >> q_WG_(2, i) >> q_WG_(3, i);
    f_p_WG >> p_WG_(0, i) >> p_WG_(1, i) >> p_WG_(2, i);

    // cout << i << " rtype: " << rtype[i] << " stuck: " << stuck[i];
    // cout << " qgrp: " << qgrp(0, i) << " " << qgrp(1, i) << " " << qgrp(2, i) << " " << qgrp(3, i);
    // cout << " grpz: " << grpz[i] << endl;
  }
  traj_is_read_ = true;
  main_loop_rate_ = main_loop_rate;

  f_rtype.close();
  f_stuck.close();
  f_q_WG.close();
  f_p_WG.close();
  return true;
}

bool ReOrienting::run() {
  if (!traj_is_read_) {
    ROS_ERROR("Error: trajectory is not read. Call initialize first.");
    return false;
  }
  traj_is_read_ = false;

  // grasp
  ZZJHand *hand = ZZJHand::Instance();
  if(hand->GraspFirmly() != MMC_SUCCESS) {
    hand->closeEpos();
    ROS_ERROR("Calling hand->GraspFirmly() failed.");
    return false;
  }

  // --------------------------------------------------------
  //  Main loop
  // --------------------------------------------------------
  ros::Duration period(EGM_PERIOD);
  ros::Rate pub_rate(main_loop_rate_);

  bool is_stuck = false;
  float pose_set[7], pose[7], force_set[6];
  robot_->getPose(pose);
  Vector3f xyz_delta;
  int mode = -1;
  controller_->reset();

  for (int fr = 1; fr < N_TRJ_; ++fr)  {
      /* ********************************************
       * There are three possible modes:
       *  1. rtype==0 (firm grasp) -- Hybrid servoing
       *  2. rtype==1, sticking == 0 -- velocity servoing
       *  3. rtype==1, sticking == 1 -- Hybrid servoing
      */
      int mode_new = 0;
      if (rtype_[fr] == false) mode_new = 1;
      else if(stuck_[fr] == false) mode_new = 2;
      else mode_new = 3;

      // Control the gripper
      if ( mode_new != mode ) {
        cout << "[New mode] Mode = " << mode_new << endl;
        // cout << "[New mode] Mode = " << mode_new << ", Press Enter to start..";
        // getchar();

        if (mode_new == 1) {
          // Switched to firm grasp
          if(hand->GraspFirmly() != MMC_SUCCESS) {
            ROS_ERROR("Failed to call hand->GraspFirmly().");
            hand->closeEpos();
            return -1;
          }
        }
        else {
          // Switched to Pivoting
          if(hand->Pivoting(open_finger_incre_for_pivot_mm_) != MMC_SUCCESS) {
            ROS_ERROR("Failed to call hand->Pivoting().");
            hand->closeEpos();
            return false;
          }
        }
      }

      // control the robot
      pose_set[0] = p_WG_(0, fr);
      pose_set[1] = p_WG_(1, fr);
      pose_set[2] = p_WG_(2, fr);
      pose_set[3] = q_WG_(0, fr);
      pose_set[4] = q_WG_(1, fr);
      pose_set[5] = q_WG_(2, fr);
      pose_set[6] = q_WG_(3, fr);

      // check collision with table
      Quaternionf qtemp;
      qtemp.w() = pose_set[3];
      qtemp.x() = pose_set[4];
      qtemp.y() = pose_set[5];
      qtemp.z() = pose_set[6];
      Vector3f grp_z  = qtemp._transformVector(Vector3f(0.0f, 0.0f, 1.0f));
      Vector3f grp_x  = qtemp._transformVector(Vector3f(1.0f, 0.0f, 0.0f));
      Vector3f grp_ax = Eigen::AngleAxis<float>(-22.5f*PI/180.0f, grp_z)._transformVector(grp_x);
      Vector3f grp(pose_set[0], pose_set[1], pose_set[2]);
      Vector3f gp1_bottom = grp + grp_ax*(grp_width_/2.0f + finger_thickness_) + grp_z*finger_radius_;
      Vector3f gp2_bottom = grp - grp_ax*(grp_width_/2.0f + finger_thickness_) + grp_z*finger_radius_;
      if ((gp1_bottom(2) < safe_Z_min_) || (gp2_bottom(2) < safe_Z_min_)) {
          cout << "The Z command is below safety limit. Gripper would hit table.";
          return false;
      }

      switch (mode_new) {
        case 1: {
          // Hybrid servoing
        }
        case 2: {
          // pure velocity control
          UT::setArray(force_set, 0, 6);
          controller_->updateAxis(Matrix3f::Identity(), 0);
          controller_->setForce(force_set);
          controller_->setPose(pose_set);
        }
        case 3: {
          // Hybrid servoing
        }
        default: {
          ROS_ERROR("Wrong mode_new: %d\n", mode_new);
          return false;
        }
      }

      mode = mode_new;

      ros::Time time_now = ros::Time::now();
      controller_->update(time_now, period);
      pub_rate.sleep();
  }

  // // write the current robot pose to file
  // robot.getPose(pose);
  // ofstream f_grp_pose;
  // f_grp_pose.open(GRP_POSE_FILE_PATH);
  // for (int i = 0; i < 7; ++i)
  //     f_grp_pose << pose[i] << " ";
  // f_grp_pose.close();


  ROS_INFO("[Regrasping] Run is done.");
  return true;
}