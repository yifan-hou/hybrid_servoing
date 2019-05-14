#include <apps/plane_engaging.h>

#include <Eigen/SVD>

#include "solvehfvc.h"
#include "utilities.h"
#include "yifanlibrary/TimerLinux.h"

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
  f_data = MatrixXd::Zero(6, controller_->_pool_size);
  v_data = MatrixXd::Zero(6, controller_->_pool_size);

  controller_->reset();
  Timer time;
  int N_TRJ_ = 1000;
  for (int fr = 0; fr < N_TRJ_; ++fr) {
    time.tic();
    /* Estimate Natural Constraints from pool of data */
    // get the weighted data
    f_data = MatrixXd::Zero(6, controller_->_f_queue.size()); // debug: make sure the size does change
    v_data = MatrixXd::Zero(6, controller_->_v_queue.size());
    for (int i = 0; i < controller_->_f_queue.size(); ++i)
      f_data.col(i) = controller_->_f_queue[i] * controller_->_f_weights[i];
    for (int i = 0; i < controller_->_v_queue.size(); ++i)
      v_data.col(i) = controller_->_v_queue[i] * controller_->_v_weights[i];
    JacobiSVD<MatrixXd> svd_v(v_data, ComputeThinV);
    MatrixXd SVD_V_v = svd_v.computeV();
    VectorXd sigma_v = svd_v.singularValues();

    // check dimension
    int DimV = 0;
    for (int i = 0; i < 6; ++i)
      if (sigma_v(i) > v_singular_value_threshold_) DimV ++;

    // get a basis for row space of velocity data
    MatrixXd rowspace_v = SVD_V_v.block<6, DimV>(0, 0);

    // filter out force data that aligns with row(v)
    int f_data_length = 0;
    MatrixXd f_data_filtered = MatrixXd::Zero(6, f_data.cols());
    for (int i = 0; i < f_data.cols(); ++i) {
      double length_in_v = (rowspace_v.transpose()*f_data.col(i)).norm();
      if (length_in_v/f_data.col(i).norm() < 0.2) {
        f_data_filtered.col(f_data_length) = f_data.col(i);
        f_data_length ++;
      }
    }
    f_data_filtered.resize(6, f_data_length); // debug: make sure the crop works properly

    // SVD to force data
    JacobiSVD<MatrixXd> svd_f(f_data_filtered, ComputeThinV);
    MatrixXd SVD_V_f = svd_f.computeV();
    VectorXd sigma_f = svd_f.singularValues();
    // check dimensions, estimate natural constraints
    int DimF = 0;
    for (int i = 0; i < 6; ++i)
      if (sigma_f(i) > f_singular_value_threshold_) DimF ++;
    MatrixXd N = SVD_V_f.block<6, DimF>(0, 0).transpose();
    // sample DimF force directions
    

    MatrixXd N;

    // get possitive direction from force measurement
    // get A b_A
    /* Do Hybrid Servoing */

    HFVC action;
    int kDimActualized      = 6;
    int kDimUnActualized    = 0;
    int kDimSlidingFriction = 0;
    int kNumSeeds           = 3;
    int kDimLambda = N.rows();

    VectorXf F(6, 1) = VectorXf::Zero(6);
    MatrixXd Aeq(1, 1); // dummy
    VectorXd beq(1); // dummy
    solvehfvc(N, G, b_G, F, Aeq, beq, A, b_A,
      kDimActualized, kDimUnActualized,
      kDimSlidingFriction, kDimLambda,
      kNumSeeds,
      &action);

    double computation_time_ms = time.toc();
    cout << "\n\nComputation Time: " << computation_time_ms << endl;
    /*  Execute the hybrid action */
    double pose_set[7];
    robot_->getPose(pose_set); // get quaternion
    for (int i = 0; i < 3; ++i) pose_set[i] = 1000.0 * p_WH(i); // mm

    double force_set[6] = {0};
    for (int i = 0; i < action.n_af; ++i)  force_set[i] = action.eta_af(i);

    cout << "\nSolved. Solution Action:" << endl;
    cout << "n_af: " << action.n_af << endl;
    cout << "n_av: " << action.n_av << endl;
    // cout << "R_a: " << action.R_a << endl;
    // cout << "pose_set: " << pose_set[0] << "|"
    //                      << pose_set[1] << "|"
    //                      << pose_set[2] << " || "
    //                      << pose_set[3] << "|"
    //                      << pose_set[4] << "|"
    //                      << pose_set[5] << "|"
    //                      << pose_set[6] << endl;
    cout << "force_set: " << force_set[0] << "|"
                          << force_set[1] << "|"
                          << force_set[2] << " || "
                          << force_set[3] << "|"
                          << force_set[4] << "|"
                          << force_set[5] << endl;
    cout << "running time: " << kTimeStepSec_ << endl;
    cout << "Press Enter to begin motion!" << endl;
    // getchar();
    // cout << "motion begins:" << endl;
    ExecuteHFVC_ABBEGM(action.n_af, action.n_av,
        action.R_a, pose_set, force_set,
        HS_CONTINUOUS, kTimeStepSec_, main_loop_rate_,
        robot_, controller_);
  } // end for

}
