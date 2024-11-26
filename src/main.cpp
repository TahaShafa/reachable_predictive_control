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

// Define the type for the RK45 solver
boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;

/*void createTrajectory(int iteration) {*/
/*    y_initial = controller.synthesizeControl(view_plot); //Follow piecewise linear trajectory*/
/*    std::cout << "y_initial: " << y_initial << std::endl;*/
/*    grs.setInitialState(y_initial); //reinitialize the initial condition to final state reached using the controller*/
/*    controller.setInitialState(y_initial);*/
/*    y_final = grs.randomReachableState(uav, t_start, t_end, dt); //Randomly generate new goal state*/
/*    controller.setY(y_final); //Set new goal state*/
/*    controller.initializeController();*/
/*}*/

int main() {

// ----------------------------------------------------------- //
  // Print the guaranteed reachable set of the UAV system
  Eigen::VectorXd initialState(2); // Example size
  initialState << 0.0, 0.0;
  double Lf = 1.0;
  double Lg = 1.0;
  UavDynamics uav(initialState, Lf, Lg);

  Eigen::VectorXd y_initial(2);
  //y_initial << 0.0, 0.0;
  y_initial << 0.0, 0.0;
  GRS grs(y_initial);

  // Define parameters for plotting the GRS
  int resolution = 2000;  // Number of resolution points on the circle
  double t_start = 0.0;
  double t_end = 0.2;
  double dt = 0.1;

  Eigen::VectorXd y = grs.randomReachableState(uav, t_start, t_end, dt); 

  std::cout << "Boundary State: " << y << std::endl;

  // Call `proxyReachableSet` function with the RK45 solver
  std::vector<std::pair<double, double>> proxyTrajectory = grs.proxyReachableSet(stepper, y_initial, uav, t_start, t_end, dt);

  std::vector<std::pair<double, double>> trueTrajectory = grs.trueReachableSet(stepper, y_initial, uav, t_start, t_end, dt);

  // Initialize Gnuplot
  Gnuplot gp;
  gp << "set title 'Reachable Set Trajectories'\n";
  gp << "set xlabel 'X'\n";
  gp << "set ylabel 'Y'\n";
  gp << "plot '-' with lines title 'True Reachable Set', '-' with lines title 'Proxy Reachable Set'\n";
  gp.send1d(trueTrajectory);
  gp.send1d(proxyTrajectory);

    // Create an instance of your dynamics class (replace with actual initialization)
    UavDynamics dynamics(initialState, Lf, Lg);

    auto proxyDynamics = dynamics.getProxyDynamics();
    auto trueDynamics = dynamics.getTrueDynamics();
    int stateDimension = 2;
    int inputDimension = 2;
    double k = 5.0;                 
    double epsilon = 0.1;
    double t_final = 0.2;
    double delta_t = 0.0015;
    Eigen::MatrixXd G0 = dynamics.getG0();
    /*Eigen::VectorXd y_final(2);*/
    /*y_final << -16.6228, 12.3907;*/
    Eigen::VectorXd y_final = y;

    // Instantiate ControllerSynthesis with the parameters
    ControllerSynthesis controller(proxyDynamics, trueDynamics, initialState, y_final, G0, stateDimension, inputDimension, t_final, epsilon, k, delta_t);

    // Run the test
    bool view_plot = true; // Generate the plot
    bool save_plot = false; // Save the plot

    int number_trajectories = 3;
    for (int i = 0; i < number_trajectories; i++) {
      auto [y_initial, inputs] = controller.synthesizeControl(view_plot); //Follow piecewise linear trajectory
      std::cout << "y_initial: " << y_initial << std::endl;
      grs.setInitialState(y_initial); //reinitialize the initial condition to final state reached using the controller
      controller.setInitialState(y_initial);
      y_final = grs.randomReachableState(uav, t_start, t_end, dt); //Randomly generate new goal state
      controller.setY(y_final); //Set new goal state
      controller.initializeController();
      std::cout << "Input Size: " << inputs.cols() << std::endl;
      std::cout << "Last Input: " << inputs.col(inputs.cols() - 1).transpose() << std::endl;
    }
    /*controller.synthesizeControl(view_plot, save_plot);*/

  return 0;
}
