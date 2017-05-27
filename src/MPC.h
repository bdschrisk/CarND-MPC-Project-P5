#ifndef MPC_H
#define MPC_H

#include <vector>
#include <iostream>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:

  size_t N_;
  double dt_;

  double ref_cte_;
  double ref_epsi_;
  double ref_v_;

  MPC();

  virtual ~MPC();

  void Init(double cte_ref, double epsi_ref, double v_ref);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  Eigen::VectorXd Predict(Eigen::VectorXd state, Eigen::VectorXd actuators, double dt);
};

#endif /* MPC_H */
