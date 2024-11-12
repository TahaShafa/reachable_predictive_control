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

int main() {

// ----------------------------------------------------------- //
  // Print the guaranteed reachable set of the UAV system
  Eigen::VectorXd initialState(2); // Example size
  initialState << 0.0, 0.0;
  double Lf = 1.0;
  double Lg = 1.0;
  UavDynamics uav(initialState, Lf, Lg);

  Eigen::VectorXd y_initial(2);
  y_initial << 0.0, 0.0;
  GRS grs(y_initial);

  // Define parameters for plotting the GRS
  int resolution = 2000;  // Number of resolution points on the circle
  double t_start = 0.0;
  double t_end = 1.0;
  double dt = 0.1;

  // Call `proxyReachableSet` function with the RK45 solver
  std::vector<std::pair<double, double>> proxyTrajectory = grs.proxyReachableSet(stepper, y_initial, uav, resolution, t_start, t_end, dt);

  std::vector<std::pair<double, double>> trueTrajectory = grs.trueReachableSet(stepper, y_initial, uav, resolution, t_start, t_end, dt);

  // Initialize Gnuplot
  Gnuplot gp;
  gp << "set title 'Reachable Set Trajectories'\n";
  gp << "set xlabel 'X'\n";
  gp << "set ylabel 'Y'\n";
  gp << "plot '-' with lines title 'Proxy Trajectory', '-' with lines title 'True Trajectory'\n";
  gp.send1d(trueTrajectory);
  gp.send1d(proxyTrajectory);

    // Create an instance of your dynamics class (replace with actual initialization)
    // Define initial state and inputs
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

    UavDynamics dynamics(initialState, Lf, Lg);

    auto proxyDynamics = dynamics.getProxyDynamics();
    auto trueDynamics = dynamics.getTrueDynamics();
    int stateDimension = 2;
    int inputDimension = 2;
    double k = 5.0;
    double epsilon = 0.1;
    double t_final = 1.0;
    double delta_t = 0.0015;
    Eigen::MatrixXd G0 = dynamics.getG0();
    Eigen::VectorXd y(2);
    y << -21.59829362, -6.38579617;

    // Instantiate ControllerSynthesis with the parameters
    ControllerSynthesis controller(proxyDynamics, trueDynamics, initialState, y, G0, stateDimension, inputDimension, t_final, epsilon, k, delta_t);

    // Run the test
    controller.synthesizeControl();

  return 0;
}
