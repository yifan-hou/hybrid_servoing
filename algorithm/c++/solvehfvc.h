/**
 * Solver for hybrid servoing.
 *
 * Author:
 *    Yifan Hou
 *    <yifanh@cmu.edu>
 */
#include <Eigen/Geometry>

/**
 * Struct for a hybrid force-velocity control action.
 *
 * n_av: dimension of velocity controls.
 * n_af: dimension of force controls.
 * R_a: n x n transformation matrix, n = n_av + n_af. Transfer from original
 *      action space to the transformed action space.
 * w_av: velocity control component, described in the transformed action space.
 * eta_af: force control component, described in the transformed action space.
 */
typedef struct
{
    int n_av;
    int n_af;
    Eigen::MatrixXd R_a;
    Eigen::VectorXd w_av;
    Eigen::VectorXd eta_af;
} HFVC;

/**
 * Solve the hybrid servoing problem. Given a system of rigid bodies with known
 * contacts, find the best hybrid force-velocity control to satisfy the given
 * velocity goal while avoid crashing & maintaining contact modes.
 *
 * Consider a kDOF dimensional system, where kDOF = @p kDimActualized + @p
 * kDimUnActualized dimensions. The generalized force f and generalized velocity
 * v are kDOF dimensional.
 *
 * The contact force f_c is kConF = @p kDimLambda + @p kDimSlidingFriction
 * dimensional. The @p kDimLambda dimensional force includes contact normal
 * forces and friction for sticking contacts. The @p kDimSlidingFriction
 * dimensional force includes sliding friction force.
 *
 * There are kForce = kConF + kDOF dimensional forces (contact forces and
 * generalized force) in the system.
 *
 * @param[in]  N_All                kConF x kDOF Constraint Jacobian Matrix. The
 *                                  first @p kDimLambda rows of @p N_All,
 *                                  denoted as N, describes holonomic
 *                                  constraints: N*v=0
 * @param[in]  G                    Matrix. Goal description. G*v=b_G. Its
 *                                  number of rows determines the number of goal
 *                                  constraints.
 * @param[in]  b_G                  Column vector with the same number of rows
 *                                  as G.
 * @param[in]  F                    kDOF dimensional external force vector. Used
 *                                  in Newton's Second Law.
 * @param[in]  Aeq                  Matrix with kForce columns. Aeq*[f_c; f] =
 *                                  beq is the equality constraint on the force
 *                                  vector.
 * @param[in]  beq                  Column vector with the same number of rows
 *                                  as Aeq.
 * @param[in]  A                    Matrix with kForce columns. A*[f_c; f] <=
 *                                  b_A is the inequality constraint on the
 *                                  force vector.
 * @param[in]  b_A                  Column vector with the same number of rows
 *                                  as A.
 * @param[in]  kDimActualized       Dimensionality of actions.
 * @param[in]  kDimUnActualized     Dimensionality of unactualized DOFs.
 * @param[in]  kDimSlidingFriction  Dimensionality of sliding frictions.
 * @param[in]  kDimLambda           Dimensionality of holonomic constraints.
 * @param[in]  kNumSeeds            Number of random seeds to use in
 *                                  projected-gradient-descent algorithm.
 * @param[in]  kPrintLevel          0: no print. 1: print brief info about final
 *                                  results. 2: print info about each stage of
 *                                  the algorithm.
 * @param      action               Solution to the hybrid servoing problem. The
 *                                  best action to take.
 *
 * @return     True if successfully find a solution.
 */
bool solvehfvc(const Eigen::MatrixXd &N_All,
  const Eigen::MatrixXd &G, const Eigen::VectorXd &b_G,
  const Eigen::VectorXd &F,
  const Eigen::MatrixXd &Aeq, const Eigen::VectorXd &beq,
  const Eigen::MatrixXd &A, const Eigen::VectorXd &b_A,
  const int kDimActualized, const int kDimUnActualized,
  const int kDimSlidingFriction, const int kDimLambda,
  const int kNumSeeds, const int kPrintLevel,
  HFVC *action);
