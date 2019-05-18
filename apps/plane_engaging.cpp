#include <apps/plane_engaging.h>

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include <Eigen/SVD>

#include "algorithms/solvehfvc.h"
#include "yifanlibrary/utilities.h"
#include "yifanlibrary/TimerLinux.h"

using namespace UT;

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");

PlaneEngaging::PlaneEngaging(ForceControlHardware *robot,
        ForceControlController *controller) {
  robot_ = robot;
  controller_ = controller;
}

PlaneEngaging::~PlaneEngaging() {
}

bool PlaneEngaging::initialize(const std::string& file_name,
    const int main_loop_rate) {
  main_loop_rate_ = main_loop_rate;
  folder_path_ = file_name;
  // parameters
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

  return true;
}

bool PlaneEngaging::run() {
  MatrixXd f_data, v_data;
  controller_->reset();
  Timer timer;
  int N_TRJ_ = 1; // 1000
  std::srand(std::time(0));
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    timer.tic();
    /* Estimate Natural Constraints from pool of data */
    // get the weighted data
    f_data = MatrixXd::Zero(6, controller_->_f_queue.size()); // debug: make sure the size does change
    v_data = MatrixXd::Zero(6, controller_->_v_queue.size());
    for (int i = 0; i < controller_->_f_queue.size(); ++i)
      f_data.col(i) = controller_->_f_queue[i] * controller_->_f_weights[i];
    for (int i = 0; i < controller_->_v_queue.size(); ++i)
      v_data.col(i) = controller_->_v_queue[i] * controller_->_v_weights[i];

    // SVD on velocity data
    Eigen::JacobiSVD<MatrixXd> svd_v(v_data, Eigen::ComputeThinV);
    VectorXd sigma_v = svd_v.singularValues();
    int DimV = 0;
    for (int i = 0; i < 6; ++i)
      if (sigma_v(i) > v_singular_value_threshold_) DimV ++;

    // get a basis for row space of velocity data
    MatrixXd rowspace_v = svd_v.matrixV().leftCols(DimV);

    // filter out force data that:
    //    1. aligns with row(v), or
    //    2. has a small weight
    int f_data_length = 0;
    MatrixXd f_data_filtered = MatrixXd::Zero(6, f_data.cols());
    for (int i = 0; i < f_data.cols(); ++i) {
      double length = f_data.col(i).norm();
      if (length > 1.5) { // weighted length in newton
        double length_in_v = (rowspace_v.transpose()*f_data.col(i)).norm();
        if (length_in_v/length < 0.2) {
          f_data_filtered.col(f_data_length) = f_data.col(i);
          f_data_length ++;
        }
      }
    }
    f_data_filtered.resize(6, f_data_length); // debug: make sure the crop works properly

    // SVD to filtered force data
    Eigen::JacobiSVD<MatrixXd> svd_f(f_data_filtered, Eigen::ComputeThinV);
    VectorXd sigma_f = svd_f.singularValues();
    // check dimensions, estimate natural constraints
    int DimF = 0;
    for (int i = 0; i < 6; ++i)
      if (sigma_f(i) > f_singular_value_threshold_) DimF ++;
    // MatrixXd N = SVD_V_f.block<6, DimF>(0, 0).transpose();
    // Sample DimF force directions
    MatrixXd f_data_normalized = f_data_filtered;
    for (int i = 0; i < f_data_length; ++i)
      f_data_normalized.col(i).normalize();

    int kNFSamples = 100;
    MatrixXd f_data_selected(6, DimF);
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
        for (int jj = 0; jj < DimF-1-ii; ++jj)
          f_distance_new += (f_data_selected_new.col(ii) -
              f_data_selected_new.col(jj)).norm();
      // 3. Update data
      if (f_distance_new > f_distance) {
        f_data_selected = f_data_selected_new;
        f_distance = f_distance_new;
      }
    } // end sampling

    MatrixXd Nf(DimF, 6);
    for (int i = 0; i < DimF; ++i)
      Nf.row(i) = f_data_selected.col(i).transpose();

    /* Do Hybrid Servoing */

    HFVC action;
    int kDimActualized      = 6;
    int kDimUnActualized    = 0;
    int kDimSlidingFriction = 0;
    int kNumSeeds           = 3;
    int kDimLambda          = DimF;

    VectorXd F = VectorXd::Zero(6);
    MatrixXd Aeq(1, 1); // dummy
    VectorXd beq(1); // dummy
    MatrixXd A = MatrixXd::Zero(DimF, DimF + 6);
    VectorXd b_A = VectorXd::Zero(DimF);
    A.leftCols(DimF) = -MatrixXd::Identity(DimF,DimF);
    double kLambdaMin = 5.0;
    for (int i = 0; i < DimF; ++i) b_A(i) = - kLambdaMin;

    solvehfvc(Nf, G_, b_G_, F, Aeq, beq, A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds,
      &action);

    double computation_time_ms = timer.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;
    /*  Execute the hybrid action */
    Vector6d v_Tr = Vector6d::Zero();
    for (int i = 0; i < action.n_av; ++i)  v_Tr(i+action.n_af) = action.w_av(i);

    const double kVMax = 0.1; // m/s,  maximum speed limit
    if (v_Tr.norm() > kVMax) {
      v_Tr.normalize();
      v_Tr *= kVMax;
    }

    const double dt = 0.2; // s
    double pose_fb[7];
    robot_->getPose(pose_fb);
    Matrix4d SE3_WT_fb = posemm2SE3(pose_fb);
    Matrix6d Adj_WT = SE32Adj(SE3_WT_fb);
    Matrix6d R_a = action.R_a;
    Matrix6d R_a_inv = R_a.inverse();
    Vector6d v_T = R_a_inv*v_Tr;
    Vector6d v_W = Adj_WT*v_T;
    Matrix4d SE3_WT_command;
    SE3_WT_command = SE3_WT_fb + wedge6(v_W)*SE3_WT_fb*dt;
    double pose_set[7];
    SE32Posemm(SE3_WT_command, pose_set);

    Vector6d force_Tr_set = Vector6d::Zero();
    for (int i = 0; i < action.n_af; ++i)  force_Tr_set[i] = action.eta_af(i);
    Vector6d force_T = R_a_inv*force_Tr_set;

    cout << "\nSolved. Solution Action:" << endl;
    cout << "n_af: " << action.n_af << endl;
    cout << "n_av: " << action.n_av << endl;
    cout << "force_Tr_set: " << force_Tr_set[0] << "|"
                          << force_Tr_set[1] << "|"
                          << force_Tr_set[2] << " || "
                          << force_Tr_set[3] << "|"
                          << force_Tr_set[4] << "|"
                          << force_Tr_set[5] << endl;
    cout << "Press Enter to begin motion!" << endl;
    // getchar();
    // cout << "motion begins:" << endl;
    controller_->ExecuteHFVC(action.n_af, action.n_av,
        action.R_a, pose_set, force_Tr_set,
        HS_CONTINUOUS, main_loop_rate_);

    if (N_TRJ_ == 1) {
      // print to file
      ofstream fp;
      // f_queue
      fp.open(folder_path_ + "plane_engaging/f_queue.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_f_queue.size(); ++i) {
        stream_array_in(fp, _f_queue[i].data(), 6);
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
        stream_array_in(fp, _v_queue[i].data(), 6);
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
        fp << _f_weights[i] << endl;
      fp.close();
      // v_weights
      fp.open(folder_path_ + "plane_engaging/v_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_v_weights.size(); ++i)
        fp << _v_weights[i] << endl;
      fp.close();

      // v_weights
      fp.open(folder_path_ + "plane_engaging/v_weights.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->_v_weights.size(); ++i)
        fp << _v_weights[i] << endl;
      fp.close();

      // f_data_filtered
      fp.open(folder_path_ + "plane_engaging/f_data_filtered.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->f_data_filtered.size(); ++i) {
        stream_array_in(fp, f_data_filtered[i].data(), 6);
        fp << endl;
      }
      fp.close();
      // f_data_selected
      fp.open(folder_path_ + "plane_engaging/f_data_selected.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      for (int i = 0; i < controller_->f_data_selected.size(); ++i) {
        stream_array_in(fp, f_data_selected[i].data(), 6);
        fp << endl;
      }
      fp.close();
      // others
      fp.open(folder_path_ + "plane_engaging/process.txt");
      if (!fp) {
        cerr << "Unable to open file to write.";
        return false;
      }
      fp << DimV << "\t" << DimF << endl;
      stream_array_in(fp, v_T.data(), 6);
      fp << endl;
      stream_array_in(fp, force_T.data(), 6);
      fp.close();
    }
  } // end for
}
