#include "gnuplot-iostream.h"
#include "uav/uav_dynamics.h"
#include "grs/grs.h"
#include "synthesis/controller_synthesis.h"
#include "solve_ode/solve_ode.h"
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <functional>

void run_synthesize_control_test(ControllerSynthesis& controller) {
    controller.synthesizeControl();
    // Assume print statements in synthesizeControl produce output values to check
}

int main() {
  /*// Initial state and inputs for the GRS*/
  /*Eigen::VectorXd initialStates(2);*/
  /*initialStates << 0.0, 0.0;*/
  /**/
  /*// Initialize the UAV dynamics */
  /*UavDynamics uav(0.1, 0.5, 0.01, 1, initialStates, 1, 1); */
  /**/
  /*Eigen::VectorXd inputs(2);*/
  /*inputs << 1.0, 0.0;*/
  /**/
  /*// Create a GRS object with initial states */
  /*GRS grs(initialStates);*/
  /**/
  /*// Time range and step size*/
  /*double t_start = 0.0;*/
  /*double t_end = 1.0;*/
  /*double dt = 0.1;*/
  /**/
  /*// Use Runge-Kutta 4th order solver*/
  /*boost::numeric::odeint::runge_kutta4<std::vector<double>> rk4;*/
  /*// Create a vector to store the trajectory as (y1, y2) pairs*/
  /*std::vector<std::pair<double, double>> trajectory;*/
  /**/
  /*  // Integrate the system using the true dynamics and print the trajectory*/
  /*std::cout << "Trajectory of the actual system: " << std::endl;*/
  /*grs.integrateTrueSystem(rk4, initialStates, uav, inputs, t_start, t_end, dt, trajectory);*/
  /**/
  /**/
  /*  for (const auto &point : trajectory) {*/
  /*      std::cout << "(y1, y2) = (" << point.first << ", " << point.second << ")" << std::endl;*/
  /*  }*/
  /**/
  /*  // Clear the trajectory vector before using it for the proxy system*/
  /*  trajectory.clear();*/
  /**/
  /*  // Reset the initial states for the proxy system*/
  /*  Eigen::VectorXd proxyInitialStates(2);*/
  /*  proxyInitialStates << 0.0, 0.0;*/
  /**/
  /*  // Integrate the proxy system and print its trajectory*/
  /*  std::cout << "\nTrajectory of the proxy system:" << std::endl;*/
  /*  grs.integrateProxySystem(rk4, proxyInitialStates, uav, inputs, t_start, t_end, dt, trajectory);*/
  /**/
  /*  for (const auto &point : trajectory) {*/
  /*      std::cout << "(y1, y2) = (" << point.first << ", " << point.second << ")" << std::endl;*/
  /*  }*/
  /**/
  /*int resolution = 1000;*/
  /*initialStates << 0.0, 0.0;*/
  /*//std::cout << "Reachable set of the actual system:" << std::endl;*/
  /*std::vector<std::pair<double, double>> proxyTrajectory = grs.proxyReachableSet(rk4, initialStates, uav, resolution, t_start, t_end, dt);*/
  /*std::vector<std::pair<double, double>> trueTrajectory = grs.trueReachableSet(rk4, initialStates, uav, resolution, t_start, t_end, dt);*/
  /**/
  /*Gnuplot gp;*/
  /*gp << "set title 'Guaranteed Reachable Sets'\n";*/
  /*gp << "plot '-' with lines title 'True Trajectory', '-' with lines title 'Proxy Trajectory'\n";*/
  /*gp.send1d(trueTrajectory);  // Send the first dataset*/
  /*gp.send1d(proxyTrajectory); // Send the second dataset*/
  
  
  //------------------------------------------------------//


  /*// Assuming n = 3 and time_steps = 5 for x_vec, same as Python code*/
  /*int stateDimension = 2;*/
  /*int inputDimension = 2;*/
  /*double t_final = 0.1; //seconds*/
  /*double epsilon = 0.01;*/
  /*double k = 5.0;*/
  /*double delta_t = 0.01;*/
  /*// Bind the member functions to the uav instance*/
  /*auto proxyDynamics = std::bind(&UavDynamics::proxyDynamics, &uav, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);*/
  /*auto trueDynamics = std::bind(&UavDynamics::trueDynamics, &uav, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);*/
  /*// Create an instance for controller synthesis to occur*/
  /*ControllerSynthesis controller(proxyDynamics, trueDynamics, initialStates, stateDimension, inputDimension, t_final, epsilon, k, delta_t);*/
  /*  double time_discretization = 100;*/
  /**/
  /*  // Initialize x_vec with the same values as in your Python code*/
  /*  Eigen::MatrixXd x_vec(2, 4);*/
  /*  x_vec << -0.82938118, -0.57324241, 1.02063516, 0.70680584,*/
  /*            0.27330638, -0.68569364, -0.35485204, -0.43491367;*/
  /**/
  /*  // Initialize z with the same values as in your Python code*/
  /*  Eigen::VectorXd z(2);*/
  /*  z << 0.61863985, 2.48353929;*/
  /**/
  /**/
  /*  // Call dist_true function*/
  /*  auto result = controller.dist_true(x_vec, z);*/
  /**/
  /*  // Print the results*/
  /*  std::cout << "Lambda Optimal: " << result.first.transpose() << std::endl;*/
  /*  std::cout << "Objective Value: " << result.second << std::endl;*/
  /**/
  /*  std::cout << "radius: " << controller.radius(k) << std::endl;*/

// ----------------------------------------------------------- //

    // Create an instance of your dynamics class (replace with actual initialization)
    // Define initial state and inputs
    Eigen::VectorXd initialState(2); // Example size
    initialState << 0.0, 0.0;
    Eigen::VectorXd inputs(2);
    inputs << 0.5, 0.5;
    inputs = inputs.transpose();

    Eigen::VectorXd container;
    container.conservativeResize(inputs.size());

    if (inputs.cols() == 1 && inputs.rows() > 1) {
      container.col(0) = inputs.transpose();
    } else {
      container.col(0) = inputs;
    }

    std::cout << "Container: " << container << std::endl;

    UavDynamics dynamics(0.1, 0.5, 0.01, 1, initialState, 1, 1);

    auto proxyDynamics = dynamics.getProxyDynamics();
    auto trueDynamics = dynamics.getTrueDynamics();
    int stateDimension = 2;
    int inputDimension = 2;
    double k = 5.0;
    double epsilon = 0.1;
    double t_final = 1.0;
    double delta_t = 0.0015;
    Eigen::MatrixXd G0 = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd y(2);
    y << -21.59829362, -6.38579617;

    // Instantiate ControllerSynthesis with the parameters
    ControllerSynthesis controller(proxyDynamics, trueDynamics, initialState, y, G0, stateDimension, inputDimension, t_final, epsilon, k, delta_t);

    std::cout << "Test print " << std::endl;

    // Run the test
    run_synthesize_control_test(controller);
    
    /*// Instantiate SystemIntegrate*/
    /*SolveODE system(initialState);*/
    /**/
    /*// Integrate and obtain trajectory*/
    /*double t_start = 0.0;*/
    /*double t_end = 5.0;*/
    /*double dt = 0.01;*/
    /*auto trajectory = system.integrate(proxyDynamics, inputs, t_start, t_end, dt);*/
    /**/
    /*// Print trajectory*/
    /*for (const auto &[t, y] : trajectory) {*/
    /*    std::cout << "t: " << t << ", y: " << y.transpose() << std::endl;*/
    /*}*/
  
  return 0;
}
