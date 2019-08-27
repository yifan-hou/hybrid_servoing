#include "plane_engaging.h"

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include <Eigen/SVD>

#include <solvehfvc.h>
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>

using namespace RUT;


PlaneEngaging::PlaneEngaging(ForceControlHardware *robot,
        ForceControlController *controller) {
  robot_ = robot;
  controller_ = controller;
}

PlaneEngaging::~PlaneEngaging() {
}

bool PlaneEngaging::initialize(const std::string& file_name,
    const int main_loop_rate, ros::NodeHandle& root_nh) {
  main_loop_rate_ = main_loop_rate;
  folder_path_ = file_name;
  // parameters in file
  //  G, b_G
  ifstream fp;
  fp.open(file_name + "plane_engaging/para.txt");
  if (!fp) {
    cerr << "Unable to open parameter file.";
    return false;
  }

  int nRowG, nColG;

  fp >> nRowG >> nColG;
  G_.resize(nRowG, nColG);
  b_G_.resize(nRowG);
  for (int i = 0; i < nRowG; ++i)
    for (int j = 0; j < nColG; ++j)
      fp >> G_(i,j);

  for (int i = 0; i < nRowG; ++i) fp >> b_G_(i);

  fp.close();

  // double check
  cout << "G_: " << G_.format(MatlabFmt) << endl;
  cout << "b_G_: " << b_G_.format(MatlabFmt) << endl;

  // parameters in ROS server
  std::vector<double> scale_force_vector;
  root_nh.getParam("/constraint_estimation/scale_force_vector", scale_force_vector);
  if (!root_nh.hasParam("/constraint_estimation/scale_force_vector"))
    ROS_WARN_STREAM("Parameter [/constraint_estimation/scale_force_vector] not found!");

  force_scale_matrix_inv_ = Matrix6d::Zero();
  for (int i = 0; i < 6; ++i) force_scale_matrix_inv_(i,i) = 1.0/scale_force_vector[i];

  root_nh.param(string("/constraint_estimation/v_singular_value_threshold"),
      v_singular_value_threshold_, 0.1);
  root_nh.param(string("/constraint_estimation/f_singular_value_threshold"),
      f_singular_value_threshold_, 0.1);
  if (!root_nh.hasParam("/constraint_estimation/v_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter "
      "[/constraint_estimation/v_singular_value_threshold] not found, "
      " using default: " << v_singular_value_threshold_);
  if (!root_nh.hasParam("/constraint_estimation/f_singular_value_threshold"))
      ROS_WARN_STREAM("Parameter "
      "[/constraint_estimation/f_singular_value_threshold] not found, "
      "using default: " << f_singular_value_threshold_);

  root_nh.param(string("/plane_engaging/Number_of_frames"),
      N_TRJ_, 1);
  if (!root_nh.hasParam("/plane_engaging/Number_of_frames"))
      ROS_WARN_STREAM("Parameter "
      "[/plane_engaging/Number_of_frames] not found, "
      " using default: " << N_TRJ_);

  return true;
}

bool PlaneEngaging::run() {
  MatrixXd f_data, v_data;
  controller_->reset();

  if (controller_->_f_queue.size() < 50) {
    cout << "Run update() for " << main_loop_rate_ << " frames:" << endl;
    // first, run update for 1s to populate the data deques
    ros::Rate pub_rate(main_loop_rate_);
    ros::Duration period(EGM_PERIOD);
    for (int i = 0; i < main_loop_rate_; ++i) {
      ros::Time time_now = ros::Time::now();
      bool b_is_safe = controller_->update(time_now, period);
      if(!b_is_safe) break;
      pub_rate.sleep();
    }
    cout << "Done." << endl;
  }

  Timer timer;
  std::srand(std::time(0));
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    timer.tic();
    /* Estimate Natural Constraints from pool of data */
    // get the weighted data
    // debug
    f_data = MatrixXd::Zero(6, controller_->_f_queue.size());
    v_data = MatrixXd::Zero(6, controller_->_v_queue.size());
    for (int i = 0; i < controller_->_f_queue.size(); ++i)
      f_data.col(i) = controller_->_f_queue[i] * controller_->_f_weights[i];
    for (int i = 0; i < controller_->_v_queue.size(); ++i)
      v_data.col(i) = controller_->_v_queue[i] * controller_->_v_weights[i];

    // SVD on velocity data
    Eigen::JacobiSVD<MatrixXd> svd_v(v_data.transpose(), Eigen::ComputeThinV);
    VectorXd sigma_v = svd_v.singularValues();
    int DimV = 0;
    for (int i = 0; i < 6; ++i)
      if (sigma_v(i) > v_singular_value_threshold_) DimV ++;

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
    int f_data_length = f_id.size();
    MatrixXd f_data_filtered = MatrixXd::Zero(6, f_data_length);
    for (int i = 0; i < f_data_length; ++i)
      f_data_filtered.col(i) = f_data.col(f_id[i]);

    int DimF = 0;
    MatrixXd Nf;
    MatrixXd f_data_selected;
    if (f_data_length > 5) {
      // SVD to filtered force data
      Eigen::JacobiSVD<MatrixXd> svd_f(f_data_filtered.transpose(), Eigen::ComputeThinV);
      VectorXd sigma_f = svd_f.singularValues();
      // check dimensions, estimate natural constraints
      double threshold = max(f_singular_value_threshold_, 0.1*sigma_f(0));
      for (int i = 0; i < 6; ++i)
        if (sigma_f(i) > threshold) DimF ++;

      if (DimF > 3) {
        cout << "DimF: " << DimF << endl;
        cout << "Press Enter to continue" << endl;
        getchar();
      }
      // MatrixXd N = SVD_V_f.block<6, DimF>(0, 0).transpose();
      // Sample DimF force directions
      MatrixXd f_data_normalized = f_data_filtered;
      for (int i = 0; i < f_data_length; ++i)
        f_data_normalized.col(i).normalize();

      int kNFSamples = (int)pow(double(f_data_length), 0.7);
      f_data_selected = MatrixXd(6, DimF);
      if (DimF == 1) {
        f_data_selected = f_data_filtered.rowwise().mean();
        f_data_selected.normalize();
      } else {
        MatrixXd f_data_selected_new(6, DimF);
        double f_distance = 0;
        assert(f_data_length < 32767);
        for (int i = 0; i < kNFSamples; ++i) {
          // 1. sample
          for (int s = 0; s < DimF; s++) {
            int r = (rand() % f_data_length);
            f_data_selected_new.col(s) = f_data_normalized.col(r);
          }
          // 2. compute distance
          double f_distance_new = 0;
          for (int ii = 0; ii < DimF-1; ++ii)
            for (int jj = ii+1; jj < DimF; ++jj)
              f_distance_new += (f_data_selected_new.col(ii) -
                  f_data_selected_new.col(jj)).norm();
          // 3. Update data
          if (f_distance_new > f_distance) {
            f_data_selected = f_data_selected_new;
            f_distance = f_distance_new;
          }
        } // end sampling
      }

      // unscale
      Nf = f_data_selected.transpose() * force_scale_matrix_inv_;
    } else {
      DimF = 0;
      Nf = MatrixXd(0, 6);
      f_data_selected = MatrixXd(6, 0);
    }

    /* Do Hybrid Servoing */
    HFVC action;
    int kDimActualized      = 6;
    int kDimUnActualized    = 0;
    int kDimSlidingFriction = 0;
    int kNumSeeds           = 3;
    int kDimLambda          = DimF;
    int kPrintLevel         = 1;

    VectorXd F = VectorXd::Zero(6);
    MatrixXd Aeq(0, DimF+6); // dummy
    VectorXd beq(0); // dummy
    MatrixXd A = MatrixXd::Zero(DimF, DimF + 6);
    VectorXd b_A = VectorXd::Zero(DimF);
    A.leftCols(DimF) = -MatrixXd::Identity(DimF,DimF);
    double kLambdaMin = 10.0;
    for (int i = 0; i < DimF; ++i) b_A(i) = -kLambdaMin;

    solvehfvc(Nf, G_, b_G_, F, Aeq, beq, A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds, kPrintLevel,
      &action);

    double computation_time_ms = timer.toc();
    /*  Execute the hybrid action */
    Vector6d v_Tr = Vector6d::Zero();
    for (int i = 0; i < action.n_av; ++i)  v_Tr(i+action.n_af) = action.w_av(i);

    double pose_fb[7];
    robot_->getPose(pose_fb);
    Matrix4d SE3_WT_fb = posemm2SE3(pose_fb);
    Matrix6d Adj_WT = SE32Adj(SE3_WT_fb);
    Matrix6d R_a = action.R_a;
    Matrix6d R_a_inv = R_a.inverse();
    Vector6d v_T = R_a_inv*v_Tr;

    const double dt = 0.1; // s
    const double kVMax = 0.002; // m/s,  maximum speed limit
    const double scale_rot_to_tran = 0.5; // 1m = 2rad
    Vector6d v_T_scaled = v_T;
    v_T_scaled.tail(3) *= scale_rot_to_tran;
    if (v_T_scaled.norm() > kVMax) {
      double scale_safe = kVMax/v_T_scaled.norm();
      v_T *= scale_safe;
    }

    Vector6d v_W = Adj_WT*v_T;
    Matrix4d SE3_WT_command;
    SE3_WT_command = SE3_WT_fb + wedge6(v_W)*SE3_WT_fb*dt;
    double pose_set[7];
    SE32Posemm(SE3_WT_command, pose_set);

    Vector6d force_Tr_set = Vector6d::Zero();
    for (int i = 0; i < action.n_af; ++i)  force_Tr_set[i] = action.eta_af(i);
    Vector6d force_T = R_a_inv*force_Tr_set;
    Matrix6d Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));
    Vector6d force_W = Adj_TW.transpose() * force_T;

    printf("V in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v_W[0], v_W[1],
        v_W[2],v_W[3],v_W[4],v_W[5]);
    printf("F in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", force_W[0],
        force_W[1], force_W[2],force_W[3],force_W[4],force_W[5]);
    cout << "Computation Time: " << computation_time_ms << endl << endl;

    if (fr == N_TRJ_-1) {
      // print to file
      ofstream fp;
      // f_queue
      fp.open(folder_path_ + "plane_engaging/f_queue.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_f_queue.size(); ++i) {
        stream_array_in(fp, controller_->_f_queue[i].data(), 6);
        fp << endl;
      }
      fp.close();
      // v_queue
      fp.open(folder_path_ + "plane_engaging/v_queue.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_v_queue.size(); ++i) {
        stream_array_in(fp, controller_->_v_queue[i].data(), 6);
        fp << endl;
      }
      fp.close();
      // f_weights
      fp.open(folder_path_ + "plane_engaging/f_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_f_weights.size(); ++i)
        fp << controller_->_f_weights[i] << endl;
      fp.close();

      // v_weights
      fp.open(folder_path_ + "plane_engaging/v_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_v_weights.size(); ++i)
        fp << controller_->_v_weights[i] << endl;
      fp.close();

      // f_data_filtered
      fp.open(folder_path_ + "plane_engaging/f_data_filtered.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << f_data_filtered.transpose() << endl;
      fp.close();
      // f_data_selected
      fp.open(folder_path_ + "plane_engaging/f_data_selected.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << f_data_selected.transpose() << endl;
      fp.close();
      // others
      fp.open(folder_path_ + "plane_engaging/process.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << DimV << endl << DimF << endl;
      stream_array_in(fp, v_T.data(), 6);
      fp << endl;
      stream_array_in(fp, force_T.data(), 6);
      fp.close();
    }

    if (std::isnan(force_Tr_set[0])) {
      cout << "================== NaN =====================" << endl;
      cout << "Press Enter to continue.." << endl;
      getchar();
    }

    cout << "motion begins:" << endl;
    controller_->ExecuteHFVC(action.n_af, action.n_av,
        action.R_a, pose_set, force_Tr_set.data(),
        HS_CONTINUOUS, main_loop_rate_);

  } // end for
}
