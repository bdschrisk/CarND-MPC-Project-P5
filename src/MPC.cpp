#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// Using timeseries rule of: 2N+1
// subtracting the first state due to the initial forward prediction
const size_t N = 11;
const double dt = 0.1;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double ref_cte;
  double ref_epsi;
  double ref_v;

  FG_eval(Eigen::VectorXd coeffs, double cte_ref, double epsi_ref, double v_ref) { 
    this->coeffs = coeffs;
    this->ref_cte = cte_ref;
    this->ref_epsi = epsi_ref;
    this->ref_v = v_ref;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += 16 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 12 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 8 * CppAD::pow(vars[delta_start + i], 2); // 4
      fg[0] += 6 * CppAD::pow(vars[a_start + i], 2); // 3
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 400 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    
    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x = vars[x_start + i];
      AD<double> y = vars[y_start + i];
      AD<double> psi = vars[psi_start + i];
      AD<double> v = vars[v_start + i];
      AD<double> cte = vars[cte_start + i];
      AD<double> epsi = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta = vars[delta_start + i];
      AD<double> alpha = vars[a_start + i];

      AD<double> f_x = coeffs[0] + coeffs[1] * x + coeffs[2] * pow(x, 2) + coeffs[3] * pow(x, 3);
      AD<double> psi_des = CppAD::atan(coeffs[1] + (2 * coeffs[3] * x) + (3 * coeffs[3] * pow(x, 2)));

      // kinematic constraints
      fg[2 + x_start + i] = x1 - (x + v * CppAD::cos(psi) * dt);
      fg[2 + y_start + i] = y1 - (y + v * CppAD::sin(psi) * dt);
      fg[2 + psi_start + i] = psi1 - (psi + v * delta / Lf * dt);
      fg[2 + v_start + i] = v1 - (v + alpha * dt);
      fg[2 + cte_start + i] = cte1 - ((f_x - y) + (v * CppAD::sin(epsi) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi - psi_des) + v * delta / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::Init(double cte_ref, double epsi_ref, double v_ref) {
  this->ref_cte_ = cte_ref;
  this->ref_epsi_ = epsi_ref;
  this->ref_v_ = v_ref;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.  
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, ref_cte_, ref_epsi_, ref_v_);
  
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the actuator values.
  return { solution.x[delta_start],   solution.x[a_start] };
}

Eigen::VectorXd MPC::Predict(Eigen::VectorXd state, Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());
  
  // extract state
  auto x = state(0);
  auto y = state(1);
  auto psi = state(2);
  auto v = state(3);

  auto delta = actuators(0);
  auto a = actuators(1);
  // apply kinematics
  next_state(0) = x + (v * cos(psi) * dt);
  next_state(1) = y + (v * sin(psi) * dt);
  next_state(2) = psi + (v / Lf * delta * dt);
  next_state(3) = v + (a * dt);

  return next_state;
}
