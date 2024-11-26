#include "synthesis/controller_synthesis.h"
#include "grs/grs.h"
#include <chrono>
#include <iostream>
#include <filesystem>
#include "gnuplot-iostream.h"
#include <iterator>
#include <random>
#include <functional>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "solve_ode/solve_ode.h"
#include "uav/uav_dynamics.h"

ControllerSynthesis::ControllerSynthesis(std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> proxyDynamics, std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> trueDynamics, Eigen::VectorXd initialState, Eigen::VectorXd y, Eigen::MatrixXd inputDynamics, int stateDimension, int inputDimension, double t_final, double epsilon, double k, double delta_t) : proxyDynamics(proxyDynamics), trueDynamics(trueDynamics), initialState(initialState), y(y), inputDynamics(inputDynamics), stateDimension(stateDimension), inputDimension(inputDimension), t_final(t_final), epsilon(epsilon), k(k), delta_t(delta_t){
  // Constructor -- setting initial conditions
  // Group of variables needed to evaluate integral solutions
  t_span = {0.0, t_final}; // Total span of time we synthesize control action for
  setIntegralTimeDiscretization(integralTimeDiscretization);
  // Theta \in [0,1]
  theta = {0.0}; //Initialize theta at its starting point 0
  
  tau = delta_t * (inputDimension + 1); // tau needs to be defined before r
  r = radius(k); // Generate circle of convergence
  // Create containers to hold the controlled trajectory, control signals, and theta
  // Controlled Trajectory
  initialState = columnVectorOrientation(initialState);
  states.conservativeResize(initialState.size(), 1);
  states.col(0) = initialState;
  std::cout << "Initial State: " << initialState << std::endl << "states: " << states << std::endl;
  // Controlled Input
  Eigen::VectorXd u0 = initialInput();
  u0 = columnVectorOrientation(u0);
  inputs.conservativeResize(u0.size(), 1);
  inputs.col(0) = u0;
  std::cout << "Initial Input: " << inputs << std::endl;

  // Update states and inputs with the initial reached state and corresponding control u_{0,0}
  states.conservativeResize(initialState.size(), 1);
  initialTrajectory(u0);
  //std::cout << "Updated states: " << states.transpose() << std::endl;
  //std::cout << "Updated Inputs: " << inputs.transpose() << std::endl;
}

void ControllerSynthesis::initializeController() {
  theta = {0.0}; //Initialize theta at its starting point 0
  r = radius(k); // Generate circle of convergence
  // Create containers to hold the controlled trajectory, control signals, and theta
  // Controlled Trajectory
  initialState = columnVectorOrientation(initialState);
  std::cout << "Initial State: " << initialState << std::endl;
  
  // Controlled Input
  Eigen::VectorXd u0 = initialInput();
  u0 = columnVectorOrientation(u0);
  // Append u0 to inputs
  inputs.conservativeResize(Eigen::NoChange, inputs.cols() + 1);
  inputs.col(inputs.cols() - 1) = u0;
  std::cout << "Initial Input: " << u0 << std::endl;

  // Update states and inputs with the initial reached state and corresponding control u_{0,0}
  initialTrajectory(u0);
}

// Getter and setter functions
void ControllerSynthesis::setY(const Eigen::VectorXd& new_y){
  y = new_y;
}
Eigen::VectorXd ControllerSynthesis::getY() const {
  return y;
}

void ControllerSynthesis::setEpsilon(double new_epsilon) {
  epsilon = new_epsilon;
}
double ControllerSynthesis::getEpsilon() const {
  return epsilon;
}

void ControllerSynthesis::setK(int new_k) {
  k = new_k;
}
double ControllerSynthesis::getK() const {
  return k;
}

void ControllerSynthesis::setDelta_t(double new_delta_t) {
  delta_t = new_delta_t;
}
double ControllerSynthesis::getDelta_t() const {
  return delta_t;
}

void ControllerSynthesis::setTheta(const std::vector<double>& new_theta) {
  theta = new_theta;
}
std::vector<double> ControllerSynthesis::getTheta() const {
  return theta;
}

void ControllerSynthesis::setIntegralTimeDiscretization(int new_integralTimeDiscretication) {
  integralTimeDiscretization = new_integralTimeDiscretication;

  // Generate t_eval: 500 points from 0 to T
  t_eval.resize(integralTimeDiscretization);
  double step_eval = t_final / (integralTimeDiscretization - 1);
  for (int i = 0; i < integralTimeDiscretization; ++i) {
      t_eval[i] = i * step_eval;
  }

  // Generate t_eval_intvl: 500 points from 0 to delta_t
  t_eval_intvl.resize(integralTimeDiscretization);
  double step_intvl = delta_t / (integralTimeDiscretization - 1);
  for (int i = 0; i < integralTimeDiscretization; ++i) {
      t_eval_intvl[i] = i * step_intvl;
  }
}

double ControllerSynthesis::getIntegralTimeDiscretization() const {
  return integralTimeDiscretization;
}

double ControllerSynthesis::getRadius() const {
  return r;
}

void ControllerSynthesis::setInitialState(Eigen::VectorXd newInitialState) {
  initialState = newInitialState;
}
Eigen::VectorXd ControllerSynthesis::getInitialState() const {
  return initialState;
}

Eigen::MatrixXd ControllerSynthesis::getStates() const {
  return states;
}

Eigen::MatrixXd ControllerSynthesis::getInputs() const {
  return inputs;
}

// Function definition: generate a vector of random signs
std::vector<int> ControllerSynthesis::generateRandomSigns(int size) {
    std::vector<int> signs(size);  // Create a vector of the given size
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 1); // Randomly returns 0 or 1

    // Fill the vector with random signs (-1 or 1)
    for (int i = 0; i < size; ++i) {
        // Convert 0 to -1, so you only get -1 or 1
        signs[i] = dist(gen) == 0 ? 1 : -1;
    }

    return signs;
}

// Calculate pseudoinverse of some matrix
Eigen::MatrixXd ControllerSynthesis::pseudoInverse(const Eigen::MatrixXd &A) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute the reciprocal of all singular values
  Eigen::VectorXd singularValuesInv = svd.singularValues();
  for (int i = 0; i < singularValuesInv.size(); ++i) {
    singularValuesInv(i) = 1.0 / singularValuesInv(i);
  }

  // Return V*singularValuesInv*U^T
  return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}

// Calculate initial control input u_{0,0}
Eigen::VectorXd ControllerSynthesis::initialInput() {
  // Compute the vector difference
  Eigen::VectorXd vect = y - initialState;

  // Normalize vect to get u_hat
  Eigen::VectorXd u_hat = vect / vect.norm();

  // Compute the pseudoinverse of G0
  Eigen::MatrixXd G0_inv = pseudoInverse(inputDynamics);

  // Compute the 2-norm of G0_inv
  double G0_inv_norm = G0_inv.jacobiSvd().singularValues()(0);

  // Calculate the initial control
  Eigen::VectorXd initialControl = (G0_inv * u_hat) / G0_inv_norm * (1 - epsilon);

  return initialControl;
}

// Generate linear target trajectory given x0 and y
std::vector<std::pair<double, double>> ControllerSynthesis::generateLinearTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, int steps) {
    // Ensure the number of steps is valid
    if (steps < 2) {
        throw std::invalid_argument("Number of steps must be at least 2.");
    }
    
    // Ensure the initial and final states have the same dimension
    if (x0.size() != xf.size()) {
        throw std::invalid_argument("Initial and final states must have the same dimension.");
    }

    // Create a vector to store the trajectory
    std::vector<std::pair<double, double>> trajectory;

    // Compute the step size (delta) for each dimension
    Eigen::VectorXd delta = (xf - x0) / (steps - 1);

    // Generate the trajectory points
    for (int i = 0; i < steps; ++i) {
        Eigen::VectorXd point = x0 + i * delta;
        // Convert Eigen::VectorXd to std::pair<double, double>
        trajectory.emplace_back(point[0], point[1]);
    }

    return trajectory;
}

// Function that returns radius using k and delta_t
double ControllerSynthesis::radius(int K) {
  if (K == -1) {
    K = k; // Use the member variable k if no argument was passed
  }

  double t_start = 0.0;
  double t_end = tau * K;
  double dt = (t_end - t_start) / (integralTimeDiscretization * K * (inputDimension + 1));

  // Generate random input vector `u` from the identity matrix
  Eigen::MatrixXd e = Eigen::MatrixXd::Identity(inputDimension, inputDimension);
  int randomIndex = 1 + (std::rand() % (inputDimension - 1));
  Eigen::VectorXd u = e.col(randomIndex);

  // Define the lambda function for the system dynamics
  auto func = [&](const std::vector<double>& x_std, std::vector<double>& dxdt_std, double t) {
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(x_std.data(), x_std.size());
    Eigen::VectorXd dxdt = proxyDynamics(t, x, u);  // Using the proxy dynamics function
    Eigen::Map<Eigen::VectorXd>(dxdt_std.data(), dxdt_std.size()) = dxdt;
  };

  // Convert Eigen::VectorXd x0 to std::vector<double> for Boost.ODEInt
  std::vector<double> x_std(initialState.data(), initialState.data() + initialState.size());

  // Integrate using Boost.ODEInt’s runge_kutta4 solver
  //boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
  boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double>> stepper;
  boost::numeric::odeint::integrate_const(stepper, func, x_std, t_start, t_end, dt);
  
  // Convert the last state back to Eigen::VectorXd and calculate its norm
  Eigen::VectorXd soln_last = Eigen::Map<Eigen::VectorXd>(x_std.data(), x_std.size()); 

  Eigen::VectorXd maxTrajectory = soln_last - initialState;

  std::cout << "radius: " << maxTrajectory.norm() << std::endl;

  return maxTrajectory.norm();
} 

// Generate a circle for figures
std::vector<std::pair<double, double>> ControllerSynthesis::generateCircle(
    double radius, double x_center, double y_center, int numPoints) {
  
  // Check for valid radius and number of points
  if (radius <= 0) {
    throw std::invalid_argument("Radius must be positive.");
  }
  if (numPoints < 3) {
    throw std::invalid_argument("Number of points must be at least 3.");
  }

  std::vector<std::pair<double, double>> circlePoints;
  const double pi = 3.141592653589793;

  // Generate points around the circle
  for (int i = 0; i < numPoints; ++i) {
    double theta = 2.0 * pi * i / numPoints;  // Angle in radians
    double x = x_center + radius * std::cos(theta);
    double y = y_center + radius * std::sin(theta);
    circlePoints.emplace_back(x, y);
  }

  return circlePoints;
}

// Generate a matrix of inputs u_{n, j} for j = {0, 1, ..., m}
Eigen::MatrixXd ControllerSynthesis::learnCycleInputs(Eigen::VectorXd u) {
  int inputSize = static_cast<int>(u.size());
  Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(inputSize, inputSize);
  std::vector<int> randomSigns = generateRandomSigns(identityMatrix.rows()); // array of randomly assigned positive/negative canonical vectors
  for (int i = 0; i < identityMatrix.rows(); ++i) {
    identityMatrix(i, i) = randomSigns[i];
  }
      
  // Stacks inputs to together (e.g. if u = [1;0] then u_enlarge = ([1, 0], [1, 0]))
  Eigen::MatrixXd u_enlarge = u.replicate(1, identityMatrix.rows()).transpose();
  Eigen::MatrixXd u_new = u_enlarge + epsilon * identityMatrix;
      
  return u_new;
}

// Initial state update given control u_{0,0}
void ControllerSynthesis::initialTrajectory(Eigen::VectorXd u0) {
  u0 = columnVectorOrientation(u0);
  Eigen::MatrixXd u_int = learnCycleInputs(u0);
  //std::cout << "u_int: " << u_int << std::endl;
  //std::cout << "inputs: " << inputs << std::endl;
  
  // Solve ODE with initial state x0 and control input u0
  Eigen::VectorXd x0 = initialState;
  //std::cout << "initial state: " << x0 << std::endl;
  SolveODE solver(x0);
  auto soln_dt = solver.solveOdeControl(trueDynamics, x0, u0, t_eval_intvl);
  
  bool is_first_iteration = true;
    // Append the solution to `states` without intermediate resizing
    for (const auto& [time, state] : soln_dt) {      // Resize state and append the last column
       if (is_first_iteration) {
          is_first_iteration = false; // Skip the first iteration and move to the next one
          continue;
        }
      states.conservativeResize(Eigen::NoChange, states.cols() + 1);
      states.col(states.cols() - 1) = state;
      // std::cout << "Current State: " << state.transpose() << std::endl;
    }

    // Solve ODE within each interval [tau_n + i*dt, tau_n+(i+1)*dt]
    for (int i = 0; i < inputDimension; ++i) {
        // Get the last column of `states` for the new initial condition
        x0 = states.col(states.cols() - 1);
        solver.setState(x0); //reinitialize the initial state

        //std::cout << "u_int.row(i).transpose(): " << u_int.row(i).transpose() << std::endl;

        // Solve ODE with each intermediate control input u_int.row(i)
        soln_dt = solver.solveOdeControl(trueDynamics, x0, u_int.row(i).transpose(), t_eval_intvl);
        
        // Append the control input to `inputs`
        inputs.conservativeResize(inputs.rows(), inputs.cols() + 1);
        inputs.col(inputs.cols() - 1) = u_int.row(i).transpose();
        //std::cout << "New Inputs: " << inputs << std::endl;

      // Reset the boolean to avoid recording repeated states
      is_first_iteration = true;
      // Append the solution to `states` without intermediate resizing
      for (const auto& [time, state] : soln_dt) {
        if (is_first_iteration) {
          is_first_iteration = false; // Skip the first iteration and move to the next one
          continue;
        }
        // Resize state and append the last column
        states.conservativeResize(Eigen::NoChange, states.cols() + 1);
        states.col(states.cols() - 1) = state;
        // std::cout << "After 500 Current State: " << state.transpose() << ",\t" << "State Length: " << states.cols() << std::endl;
      }
    }
}

// Function to compute the intersection between a line and a circle
std::pair<double, Eigen::VectorXd> ControllerSynthesis::zSequence(const Eigen::VectorXd &center) {
  // Line equation: y_line = theta * (y - x0) + x0
  // Circle equation: (y_line - center)^2 = r^2
  /*double a = y.squaredNorm();*/
  /*double b = -2 * center.dot(y);*/
  /*double c = center.squaredNorm() - r * r;*/

  // Define the difference vector (y - x0)
  Eigen::VectorXd direction = y - initialState;

  // Coefficients of the quadratic equation
  double a = direction.squaredNorm();  // Norm of (y - x0)
  double b = 2 * direction.dot(initialState - center);  // Interaction term
  double c = (initialState - center).squaredNorm() - r * r;  // Circle offset

  // Sovle the quadratic equation for a circle in 2D: a * theta^2 + b * theta + c = 0
  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    std::cerr << "No real solution found for the intersection." << std::endl;
    return std::make_pair(-1, Eigen::VectorXd::Zero(y.size()));  // Return invalid result
  }
  
  // Compute both solutions
  double theta1 = (-b + std::sqrt(discriminant)) / (2 * a);
  double theta2 = (-b - std::sqrt(discriminant)) / (2 * a);

  // Choose the appropriate solution baed on the condition t > theta[-1]
  double theta_new;
  if (theta1 > theta.back() - 0.00000001) {
    theta_new = theta1;
  } else if (theta2 > theta.back() - 0.00000001) {
    theta_new = theta2;
  } else {
    std::cerr << "No valid theta found based on the condition." << std::endl;
    return std::make_pair(-1, Eigen::VectorXd::Zero(y.size()));  // Return invalid result
  }

  // Append the new theta value
  theta.push_back(theta_new);
  std::cout << "theta = " << theta_new << std::endl;

  // Calculate the corresponding point on the line: y_line = theta_new * y
  Eigen::VectorXd y_line = theta_new * (y - initialState) + initialState;

  // Return the new theta and y_line
  return std::make_pair(theta_new, y_line);
}

// Approximate the minimum of the inner product of the gradient of d_z and f + gu by solving a linear programming problem to determine lambda
std::pair<Eigen::VectorXd, double> ControllerSynthesis::dist_true(const Eigen::VectorXd& z) {
  // Gradient and difference computation
  Eigen::VectorXd grad = 2 * (states.col(states.cols() - 1) - z);

  std::vector<Eigen::VectorXd> x_vec_reverse;
  for (int m = 0; m <= this->inputDimension; ++m) {
    int col_index = states.cols() - 1 - m * (integralTimeDiscretization - 1);
    if (col_index >= 0 && col_index < states.cols()) {
      x_vec_reverse.push_back(states.col(col_index));
    }
  }

  int additional_col_index = states.cols() - inputDimension * (integralTimeDiscretization - 1) - integralTimeDiscretization;
  if (additional_col_index >= 0 && additional_col_index < states.cols()) {
    x_vec_reverse.push_back(states.col(additional_col_index));
  }

  // Reverse columns to get x_vec with the correct order
  Eigen::MatrixXd x_vec(x_vec_reverse[0].size(), x_vec_reverse.size());
  for (size_t i = 0; i < x_vec_reverse.size(); ++i) {
    x_vec.col(x_vec_reverse.size() - i - 1) = x_vec_reverse[i];
  }

  // Skip initial column for difference calculation
  Eigen::MatrixXd x_vec_diff = (x_vec.rightCols(x_vec.cols() - 1) - x_vec.leftCols(x_vec.cols() - 1)) / delta_t;

  // Coefficients for the objective function
  Eigen::VectorXd c = x_vec_diff.transpose() * grad;

  int num_vars = c.size();

  // Set up GLPK problem
  glp_prob *lp;
  lp = glp_create_prob();            // Create a new problem
  glp_set_prob_name(lp, "dist_true");  // Set problem name
  glp_set_obj_dir(lp, GLP_MIN);      // Set minimization

  // Add variables (columns)
  glp_add_cols(lp, num_vars);

  for (int i = 1; i <= num_vars; ++i) {
      glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);  // Bounds 0 <= lambda_i <= 1
  }

  // Set objective function coefficients
  for (int i = 1; i <= num_vars; ++i) {
      glp_set_obj_coef(lp, i, c[i-1]);  // Objective: minimize c * lambda
  }

  // Add constraint: sum(lambda) = 1
  glp_add_rows(lp, 1);  // Add one constraint
  glp_set_row_bnds(lp, 1, GLP_FX, 1.0, 1.0);  // Equality constraint: sum(lambda_i) = 1

  // Populate matrix A (constraint coefficients)
  std::vector<int> ia(1 + num_vars);  // 1-based index
  std::vector<int> ja(1 + num_vars);
  std::vector<double> ar(1 + num_vars);
  for (int i = 1; i <= num_vars; ++i) {
      ia[i] = 1;       // Constraint row
      ja[i] = i;       // Column index
      ar[i] = 1.0;     // Coefficient: A * lambda = 1
  }

  // Load the constraint matrix
  glp_load_matrix(lp, num_vars, ia.data(), ja.data(), ar.data());

  // Solve the linear programming problem
  glp_simplex(lp, NULL);

  // Get the solution (lambda_optimal)
  Eigen::VectorXd lambda_optimal(num_vars);
  for (int i = 1; i <= num_vars; ++i) {
      lambda_optimal[i-1] = glp_get_col_prim(lp, i);
  }

  // Calculate the objective value
  double objective_value = 0;
  for (int i = 1; i <= num_vars; ++i) {
      objective_value += glp_get_obj_coef(lp, i) * lambda_optimal[i-1];
  }

  // Clean up
  glp_delete_prob(lp);

  // Return lambda_optimal and the objective value
  return std::make_pair(lambda_optimal, objective_value);
}

void ControllerSynthesis::savePlotToFigures(
    const std::vector<std::pair<double, double>> &synthesizedTrajectory,
    const std::vector<std::pair<double, double>> &desiredTrajectory,
    int iteration,
    const std::vector<std::pair<double, double>> &convergentRadius,
    const std::vector<std::pair<double, double>> &reachableSet) {
      // Ensure the `figures` directory exists
    std::filesystem::path figuresDir = std::filesystem::current_path().parent_path() / "figures/grs_waypoints";
    std::filesystem::create_directory(figuresDir);

    // Generate a unique filename for each iteration
    std::ostringstream filename;
    filename << "trajectory_plot_grs_" << iteration << ".png";

    // Define the output file path
    std::filesystem::path outputFile = figuresDir / filename.str();

    // Plot using Gnuplot
    Gnuplot gp;
    gp << "set terminal pngcairo enhanced size 800,600\n"; // Use PNG format
    gp << "set output '" << outputFile.string() << "'\n";  // Set output file
    gp << "set title 'Real-Time Trajectory Following Using Reachable Predictive Control Within 0.2 Seconds'\n";
    gp << "set xlabel 'x_{1}' font ',14' enhanced\n";
    gp << "set ylabel 'x_{2}' font ',14' enhanced\n";

    /*// Set axis ranges*/
    /*gp << "set xrange [" << -21.0 << ":" << 0.0 << "]\n";*/
    /*gp << "set yrange [" << 0.0 << ":" << 16.0 << "]\n";*/

    // Set axis ranges
    gp << "set xrange [" << -25.0 << ":" << 25.0 << "]\n";
    gp << "set yrange [" << -20.0 << ":" << 25.0 << "]\n";

    // Specify which trajectories to plot
    gp << "plot ";
    if (!reachableSet.empty()) {
      gp << "'-' with lines title 'Guaranteed Reachable Set', ";
    }
    gp << "'-' with lines lw 2.5 lc rgb 'black' title 'Desired Trajectory', "
       << "'-' with lines lw 2.5 title 'Learned Trajectory'";
    if (!convergentRadius.empty()){
       gp << ", '-' with lines title 'Convergence Circle'";
    }
    gp << "\n";
    if (!reachableSet.empty()) {
      std::cout << "reachable set is not empty" << std::endl;
      gp.send1d(reachableSet);
    }
    gp.send1d(desiredTrajectory);   // Send desired trajectory
    gp.send1d(synthesizedTrajectory);         // Send learned trajectory

    if (!convergentRadius.empty()) {
      gp.send1d(convergentRadius);
    }

    std::cout << "Plot saved to: " << outputFile.string() << std::endl;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> ControllerSynthesis::synthesizeControl(bool view_plot, bool save_plot) {
  std::cout << "Number of States: " << states.cols() << std::endl;

  std::vector<std::pair<double, double>> proxyTrajectory;

  // Start measuring time
  auto start = std::chrono::high_resolution_clock::now();

  SolveODE solver(states.col(states.cols() - 1));

  double t;
  Eigen::VectorXd z;
  std::tie(t, z) = zSequence(states.col(states.cols() - 1));

  Eigen::VectorXd lambda_optimal;
  double q;
  std::tie(lambda_optimal, q) = dist_true(z);
    
  for (int i = 0; i < iteration; ++i) {
    std::cout << "Iteration: " << i << std::endl;
    Eigen::VectorXd u = (1 - epsilon) * (lambda_optimal.transpose() * getInputs().rightCols(inputDimension + 1).transpose());

    // Intermediate control inputs
    Eigen::MatrixXd u_int = learnCycleInputs(u);

    // Solve ODE over [t_n, t_n+dt] with most recent state as initial condition
    auto soln_dt = solver.solveOdeControl(trueDynamics, states.col(states.cols() - 1), u, t_eval_intvl);

    for (const auto& [time, state] : soln_dt) {
        states.conservativeResize(Eigen::NoChange, states.cols() + 1);
        states.col(states.cols() - 1) = state;
    }

    // Solve ODE within [tau_n + (i * dt), tau_n + (i + 1) * dt] intervals
    for (int j = 0; j < u_int.rows(); ++j) {
      Eigen::VectorXd x0 = states.col(states.cols() - 1);
      Eigen::VectorXd u_segment = u_int.row(j).transpose();
      auto soln_dt_interval = solver.solveOdeControl(trueDynamics, x0, u_segment, t_eval_intvl);

      for (const auto& [time, state] : soln_dt_interval) {
        states.conservativeResize(Eigen::NoChange, states.cols() + 1);
        states.col(states.cols() - 1) = state;
      }

      inputs.conservativeResize(inputs.rows(), inputs.cols() + 1);
      inputs.col(inputs.cols() - 1) = u_segment;
    }

    // Update z and lambda for the next iteration
    std::tie(t, z) = zSequence(states.col(states.cols() - 1));
    std::tie(lambda_optimal, q) = dist_true(z);

    std::cout << "theta(t): " << t << std::endl;

    if (t > 0.8) {
      std::cout << "Number of States: " << states.cols() << std::endl;
      // Prepare state data for Gnuplot
      std::vector<std::pair<double, double>> state_data;
      for (int i = 0; i < states.cols(); ++i) {
        state_data.emplace_back(states(0, i), states(1, i));
      }
  std::cout << "Number of State Data Columns: " << state_data.size() << std::endl;

      std::vector<std::pair<double, double>> desiredTrajectory = generateLinearTrajectory(initialState, y);

      if (view_plot){
        // Plot using Gnuplot
        Gnuplot gp;
        gp << "set title 'Trajectory Plot of States'\n";
        //gp << "set xrange [" << -50.0 << ":" << 50.0 << "]\n";
        //gp << "set yrange [" << -50.0 << ":" << 50.0 << "]\n";
        gp << "plot '-' with lines title 'Learned Trajectory', '-' with lines title 'Desired Trajectory'\n";
        gp.send1d(state_data);
        gp.send1d(desiredTrajectory);
      }
      std::cout << "Max theta reached" << std::endl;
      Eigen::MatrixXd lastColumn = states.col(states.cols() - 1);
      std::cout << "End State: " << lastColumn << std::endl;
      return std::make_pair(lastColumn, inputs);
      break;
    }

    if (save_plot){
      if ((i+1) % 1 == 0){
        if ((i + 1) == 1) {
          // Print the guaranteed reachable set of the UAV system
          double Lf = 1.0;
          double Lg = 1.0;
          UavDynamics uav(initialState, Lf, Lg);

          Eigen::VectorXd y_initial(2);
          y_initial << 5.0, 5.0;
          GRS grs(y_initial);

          // Define parameters for plotting the GRS
          double t_start = 0.0;
          double t_end = 0.2;
          double dt = 0.1;

          // Define the type for the RK45 solver
          boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;

          // Call `proxyReachableSet` function with the RK45 solver
          proxyTrajectory = grs.proxyReachableSet(stepper, y_initial, uav, t_start, t_end, dt);
        }

        // Prepare state data for Gnuplot
        std::vector<std::pair<double, double>> state_data;
        for (int i = 0; i < states.cols(); ++i) {
          state_data.emplace_back(states(0, i), states(1, i));
        }
        
        // Create the desired trajectory
        std::vector<std::pair<double, double>> desiredTrajectory = generateLinearTrajectory(initialState, y);
        std::cout << "getRadius: " << getRadius() << std::endl;

        // Produce the radius of convergence for the plot
        Eigen::VectorXd lastColumn = states.col(states.cols() - 1);
        double x_center = lastColumn(0); double y_center = lastColumn(1);
        double radius = getRadius();
        std::vector<std::pair<double, double>> convergentCircle = generateCircle(radius, x_center, y_center);
        /*std::vector<std::pair<double, double>> convergentCircle = {};*/

        // Save plots to the figures folder
        savePlotToFigures(state_data, desiredTrajectory, i, convergentCircle, proxyTrajectory);
      }
    }
  }
  std::cout << "Number of States: " << states.cols() << std::endl;

  // Stop measuring time
  auto stop = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = stop - start;
  std::cout << "Total Elapsed Time: " << elapsed.count() << std::endl;

  // Plotting results
  std::vector<std::pair<double, double>> y_data = {{y(0), y(1)}};

  // Prepare state data for Gnuplot
  std::vector<std::pair<double, double>> state_data;
  for (int i = 0; i < states.cols(); ++i) {
    state_data.emplace_back(states(0, i), states(1, i));
  }

  std::cout << "Number of State Data Columns: " << state_data.size() << std::endl;

  std::vector<std::pair<double, double>> desiredTrajectory = generateLinearTrajectory(initialState, y);

  if (view_plot){
    // Plot using Gnuplot
    Gnuplot gp;
    gp << "set title 'Trajectory Plot of States'\n";
    gp << "plot '-' with lines title 'Learned Trajectory', '-' with lines title 'Desired Trajectory'\n";
    gp.send1d(state_data);
    gp.send1d(desiredTrajectory);
  }

  // Extract the last column
  Eigen::VectorXd lastColumn = states.col(states.cols() - 1);
  std::cout << "End State: " << lastColumn << std::endl;
  return std::make_pair(lastColumn, inputs);

    /*if (save_plot) {*/
    /*  // Save plots to the figures folder*/
    /*  savePlotToFigures(state_data, desiredTrajectory);*/
    /*}*/
}
