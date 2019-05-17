///
/// Solve for hybrid force-velocity control actions
///
/// Input arguments
///   required:
///       Omega: matrix for mapping from generalized velocity to configuration
///               time derivatives: q_dot = Omega*v
///       Jac_phi_q_all: Jacobian of holonomic constraints w.r.t. configuration
///       G, b_G: Goal description, affine constraints on generalized velocity
///               G*v = b_G
///       F: External force vector. Same size as generalized force
///       Aeq, beq: Guard condition, Aeq*v = beq
///       A, b_A: Guard condition, A*v <= b_A
///       dims:
///           dims.Actualized: number of actualized dimensions
///           dims.UnActualized: number of unactualized dimensions
///           dims.SlidingFriction: number of sliding friction dimensions
///           dims.Lambda: number of reaction forces
///
///   optional:
///       num_seeds: number of random initializations to try when solving for
///               the velocity control
/// Outputs
///   n_av: number, dimensionality of velocity controlled actions
///   n_af: number, dimensionality of force controlled actions
///   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
///           direction of velocity&force control actions
///   w_av: n_av x 1 vector, magnitudes of velocity controls
///   eta_af: n_af x 1 vector,  magnitudes of force controls

#include <Eigen/Geometry>


typedef struct
{
    int n_av;
    int n_af;
    Eigen::MatrixXd R_a;
    Eigen::VectorXd w_av;
    Eigen::VectorXd eta_af;
} HFVC;

bool solvehfvc(const Eigen::MatrixXd &N_All,
  const Eigen::MatrixXd &G, const Eigen::VectorXd &b_G,
  const Eigen::VectorXd &F,
  const Eigen::MatrixXd &Aeq, const Eigen::VectorXd &beq,
  const Eigen::MatrixXd &A, const Eigen::VectorXd &b_A,
  const int kDimActualized, const int kDimUnActualized,
  const int kDimSlidingFriction, const int kDimLambda,
  const int kNumSeeds,
  HFVC *action);
