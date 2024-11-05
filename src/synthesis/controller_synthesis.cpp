#include "synthesis/controller_synthesis.h"
#include <chrono>
#include <iostream>
#include "gnuplot-iostream.h"
#include <iterator>
#include <random>
#include <functional>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "solve_ode/solve_ode.h"
#include "uav/uav_dynamics.h"

ControllerSynthesis::ControllerSynthesis(std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> proxyDynamics, std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> trueDynamics, Eigen::VectorXd initialState, Eigen::VectorXd y, Eigen::MatrixXd inputDynamics, int stateDimension, int inputDimension, double t_final, double epsilon, double k, double delta_t) : proxyDynamics(proxyDynamics), trueDynamics(trueDynamics), initialState(initialState), y(y), inputDynamics(inputDynamics), stateDimension(stateDimension), inputDimension(inputDimension), t_final(t_final), epsilon(epsilon), k(k), delta_t(delta_t) {
  // Constructor -- setting initial conditions
  // Group of variables needed to evaluate integral solutions
  t_span = {0.0, t_final}; // Total span of time we synthesize control action for
  // Generate t_eval: 500 points from 0 to T
  Eigen::VectorXd t_eval = Eigen::VectorXd::LinSpaced(defaultIntegralTimeDiscretization, 0.0, t_final);
  // Generate t_eval_intvl: 500 points from 0 to dt
  Eigen::VectorXd t_eval_intvl = Eigen::VectorXd::LinSpaced(defaultIntegralTimeDiscretization, 0.0, delta_t);

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
  // Theta \in [0,1]
  theta = {0.0}; //Initialize theta at its starting point 0

  // Update states and inputs with the initial reached state and corresponding control u_{0,0}
  states.conservativeResize(initialState.size(), 1);
  initialTrajectory(u0);
  std::cout << "Updated states: " << states.transpose() << std::endl;
  std::cout << "Updated Inputs: " << inputs.transpose() << std::endl;
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

void ControllerSynthesis::setIntegralTimeDiscretization(double new_integralTimeDiscretication) {
  integralTimeDiscretization = new_integralTimeDiscretication;
}
double ControllerSynthesis::getIntegralTimeDiscretization() const {
  return integralTimeDiscretization;
}

double ControllerSynthesis::getRadius() const {
  return r;
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
  double G0_inv_norm = G0_inv.norm();

  // Calculate the initial control
  Eigen::VectorXd initialControl = (G0_inv * u_hat) / G0_inv_norm * (1 - epsilon);

  return initialControl;
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

  return soln_last.norm();
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
  
  // Solve ODE with initial state x0 and control input u0
  Eigen::VectorXd x0 = initialState;
  SolveODE solver(x0);
  auto soln_dt = solver.integrate(trueDynamics, u0, 0.0, delta_t, delta_t);
  
  bool is_first_iteration = true;
    // Append the solution to `states` without intermediate resizing
    for (const auto& [time, state] : soln_dt) {      // Resize state and append the last column
       if (is_first_iteration) {
          is_first_iteration = false; // Skip the first iteration and move to the next one
          continue;
        }
      states.conservativeResize(Eigen::NoChange, states.cols() + 1);
      states.col(states.cols() - 1) = state;
      std::cout << "Current State: " << state.transpose() << std::endl;
    }

    // Solve ODE within each interval [tau_n + i*dt, tau_n+(i+1)*dt]
    for (int i = 0; i < inputDimension; ++i) {
        // Get the last column of `states` for the new initial condition
        x0 = states.col(states.cols() - 1);
        solver.setState(x0); //reinitialize the initial state

        // Solve ODE with each intermediate control input u_int.row(i)
        soln_dt = solver.integrate(trueDynamics, u_int.row(i).transpose(), 0.0, delta_t, delta_t);
        
        // Append the control input to `inputs`
        inputs.conservativeResize(inputs.rows(), inputs.cols() + 1);
        inputs.col(inputs.cols() - 1) = u_int.row(i).transpose();

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
        std::cout << "Current State: " << state.transpose() << std::endl;
      }
    }
}

// Function to compute the intersection between a line and a circle
std::pair<double, Eigen::VectorXd> ControllerSynthesis::zSequence(const Eigen::VectorXd &center) {
  // Line equation: y_line = theta * y
  // Circle equation: (y_line - center)^2 = r^2
  double a = y.squaredNorm();
  std::cout << "Error A " << center.size() << "/t" << y.size() << std::endl;
  double b = -2 * center.dot(y);
  std::cout << "Error B " << std::endl;
  double c = center.squaredNorm() - r * r;

  std::cout << "Sequence error test 1 " << std::endl;

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
  Eigen::VectorXd y_line = theta_new * y;

  // Return the new theta and y_line
  return std::make_pair(theta_new, y_line);
}

// Approximate the minimum of the inner product of the gradient of d_z and f + gu by solving a linear programming problem to determine lambda
std::pair<Eigen::VectorXd, double> ControllerSynthesis::dist_true(Eigen::MatrixXd& x_vec, const Eigen::VectorXd& z, double integralTimeDiscretization) {
  // Gradient and difference computation
  std::cout << "x_vec: " << x_vec.transpose() << std::endl;
  Eigen::VectorXd grad = 2 * (x_vec.col(x_vec.cols() - 1) - z);

  // Define the number of columns to print, which is m + 1
  int columns_to_save = inputDimension + 1;

  // Ensure we are not accessing more columns than available
  int start_col = (x_vec.cols() > columns_to_save) ? x_vec.cols() - columns_to_save : 0;

  Eigen::MatrixXd x_vec_diff = x_vec.block(0, start_col, x_vec.rows(), columns_to_save);

    for (int i = 0; i < columns_to_save; ++i) {
    std::cout << "Last Column " << i << ": " << x_vec_diff.col(i).transpose() << std::endl;
  }

  /*// Generate x_vec_reverse of size m+1*/
  /*std::vector<Eigen::VectorXd> x_vec_reverse;*/
  /*//int time_discretization = static_cast<int>(integralTimeDiscretization);*/
  /**/
  /*for (int m = 0; m <= inputDimension; ++m) {*/
  /*      int col_index = x_vec.cols() - 1 - m;*/
  /*      std::cout << "column index: " << col_index << std::endl;*/
  /*      x_vec_reverse.push_back(x_vec.col(col_index));*/
  /*      std::cout << "x_vec_reverse: " << x_vec.col(col_index) << std::endl;*/
  /*  }*/
  /**/
  /*  int additional_col_index = x_vec.cols() - inputDimension - 1;*/
  /*  x_vec_reverse.push_back(x_vec.col(additional_col_index));*/
  /**/
  /*  // Reverse the vector into a matrix*/
  /*  Eigen::MatrixXd x_vec_mat(x_vec_reverse[0].size(), x_vec_reverse.size());*/
  /*  for (size_t i = 0; i < x_vec_reverse.size(); ++i) {*/
  /*      x_vec_mat.col(x_vec_reverse.size() - i - 1) = x_vec_reverse[i];*/
  /*  }*/
  /**/
  /*  // Step 4: Calculate `x_vec_difference`*/
  /*  Eigen::MatrixXd x_vec_diff = (x_vec_mat.rightCols(x_vec_mat.cols() - 1) - x_vec_mat.leftCols(x_vec_mat.cols() - 1));*/
  /**/
  /*  std::cout << "x_vec: " << x_vec << std::endl;*/
  /*  std::cout << "x_vec_difference: " << x_vec_diff << std::endl;*/

  /*Eigen::MatrixXd x_vec_diff = (x_vec.rightCols(x_vec.cols() - 1) - x_vec.leftCols(x_vec.cols() - 1));*/

  std::cout << "x_vec_difference: " << x_vec_diff << std::endl;

  // Coefficients for the objective function
  Eigen::VectorXd c = x_vec_diff.transpose() * grad;

  int num_vars = c.size();

  std::cout << "num_vars: " << num_vars << std::endl;

  // Set up GLPK problem
  glp_prob *lp;
  lp = glp_create_prob();            // Create a new problem
  glp_set_prob_name(lp, "dist_true");  // Set problem name
  glp_set_obj_dir(lp, GLP_MIN);      // Set minimization

  std::cout << "dist_true 2" << std::endl;

  // Add variables (columns)
  glp_add_cols(lp, num_vars);

  std::cout << "dist_true 3" << std::endl;
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

  // Start measuring time
  auto start = std::chrono::high_resolution_clock::now();

  // Solve the linear programming problem
  glp_simplex(lp, NULL);

  // Stop measuring time
  auto stop = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = stop - start;

  // Output the runtime
  std::cout << "Time taken by GLPK simplex optimizer: " << elapsed.count() << " seconds" << std::endl;

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

void ControllerSynthesis::synthesizeControl() {
    // Instantiate SolveODE with the initial last state of states
    SolveODE solver(states.col(states.cols() - 1));
    // Convert Eigen::MatrixXd x_vec to std::vector for gnuplot-iostream compatibility
    std::vector<std::pair<double, double>> x_vec_data;
    
    // Print the last state in states
    std::cout << "Last state:\n" << states.col(states.cols() - 1).transpose() << std::endl;

    // Initialize z and lambda based on the last state
    double t;
    Eigen::VectorXd z;

    std::cout << "zSequence error " << std::endl;

    std::tie(t, z) = zSequence(states.col(states.cols() - 1));
    
    std::cout << "Test print 0.0 " << std::endl;

    Eigen::VectorXd lambda_optimal;
    double q;
    
    std::cout << "Test print 0 " << std::endl;

    std::tie(lambda_optimal, q) = dist_true(states, z, integralTimeDiscretization);

    std::cout << "Test print 1 " << std::endl;

    for (int i = 0; i < iteration; ++i) {
        std::cout << "Iteration: " << i << std::endl;
        std::cout << "Lambda optimal: " << lambda_optimal.transpose() << std::endl;
        std::cout << "cwise Product: " << getInputs().rightCols(inputDimension + 1).colwise().reverse() << std::endl; 
        // Reverse the last `inputDimension + 1` columns of `getInputs()`
        Eigen::MatrixXd reversedInputs = getInputs().rightCols(inputDimension + 1).colwise().reverse();
        // Control synthesis using the optimal lambda
        Eigen::VectorXd u = (1 - epsilon) * (lambda_optimal.transpose() * reversedInputs.transpose());

        // Determine intermediate control inputs
        Eigen::MatrixXd u_int = learnCycleInputs(u);

        // Update solver's initial state to the last state in states
        solver.setState(states.col(states.cols() - 1));

        // Define tau_n as a function of tau and the number of iterations to reach y
        double tau_n = iteration * tau;

        // Solve ODE within [tau_n, tau_n + dt] and update states
        auto soln_dt = solver.integrate(trueDynamics, u, tau_n, tau_n + delta_t, delta_t);
        
        bool is_first_iteration = true;
        // Append the solution to `states` without intermediate resizing
        for (const auto& [time, state] : soln_dt) {      // Resize state and append the last column
          if (is_first_iteration) {
            is_first_iteration = false; // Skip the first iteration and move to the next one
            continue;
          }
          states.conservativeResize(Eigen::NoChange, states.cols() + 1);
          states.col(states.cols() - 1) = state;
          std::cout << "Current State: " << state.transpose() << std::endl;
        }

        // Solve ODE within each interval [tau_n + i*dt, tau_n + (i+1)*dt]
        for (int j = 0; j < stateDimension; ++j) {
          // Extract the j-th row of u_int as the control input for this interval
          Eigen::VectorXd initial_state = u_int.row(j).transpose(); // Convert row to column vector

          // Update solver's initial state for each interval
          solver.setState(states.col(states.cols() - 1));
          auto soln_dt = solver.integrate(trueDynamics, initial_state, tau_n + j * delta_t, tau_n + (j + 1) * delta_t, delta_t);

          // Resize `inputs` to add the j-th row as the last column
          inputs.conservativeResize(inputs.rows(), inputs.cols() + 1);
          inputs.col(inputs.cols() - 1) = u_int.row(j).transpose();

          // Reset the boolean to avoid recording repeated states
          is_first_iteration = true;
          // Append the solution to `states` without intermediate resizing
          for (const auto& [time, state] : soln_dt) {      // Resize state and append the last column
            if (is_first_iteration) {
              is_first_iteration = false; // Skip the first iteration and move to the next one
              continue;
            }
            states.conservativeResize(Eigen::NoChange, states.cols() + 1);
            states.col(states.cols() - 1) = state;
            std::cout << "Current State: " << state.transpose() << std::endl;
          }
        }

        // Update z_n and lambda for the next iteration
        std::tie(t, z) = zSequence(states.col(states.cols() - 1));
        std::tie(lambda_optimal, q) = dist_true(states, z, integralTimeDiscretization);

    }

    /*Gnuplot gp;*/
    Eigen::VectorXd y(2);
    y << -21.59829362, -6.38579617;
    // Convert y to a std::vector with a single point
    std::vector<std::pair<double, double>> y_data = {{y(0), y(1)}};

  std::vector<std::pair<double, double>> state_data;
  for (int i = 0; i < states.cols(); ++i) {
      state_data.emplace_back(states(0, i), states(1, i));
  }

  // Initialize Gnuplot
Gnuplot gp;
gp << "set title 'Trajectory Plot of States'\n";
gp << "plot '-' with lines title 'States Trajectory', '-' with points title 'Y Point'\n";
gp.send1d(state_data);
gp.send1d(y_data);      // Send y data as a point plot
}
