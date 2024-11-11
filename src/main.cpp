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


int main() {
<<<<<<< HEAD
=======

// ----------------------------------------------------------- //
>>>>>>> 38c6ee4 (Trajectory Following Beginning to Work)

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

    double Lf = 1.0;
    double Lg = 1.0;

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
<<<<<<< HEAD
    run_synthesize_control_test(controller);
  
=======
    controller.synthesizeControl();

>>>>>>> 38c6ee4 (Trajectory Following Beginning to Work)
  return 0;
}

/*// Test program to verify solveOdeControl*/
/*int main() {*/
/*    // Define initial state, control input, and time intervals for testing*/
/*    Eigen::VectorXd x0(2);*/
/*    x0 << 0.0, 0.0;*/
/*    Eigen::VectorXd u0(2);*/
/*    u0 << 1.0, 1.0;*/
/**/
/*    UavDynamics dynamics(0.1, 0.5, 0.01, 1, x0, 1, 1);*/
/**/
/*    auto proxyDynamics = dynamics.getProxyDynamics();*/
/*    auto trueDynamics = dynamics.getTrueDynamics();*/
/**/
/**/
/*    // Generate t_eval_intvl: 500 points from 0 to 0.1*/
/*    int integralTimeDiscretization = 500;*/
/*    double delta_t = 0.0015;*/
/*    std::vector<double> t_eval_intvl(integralTimeDiscretization);*/
/*    double step_intvl = delta_t / (integralTimeDiscretization - 1);*/
/*    for (int i = 0; i < integralTimeDiscretization; ++i) {*/
/*        t_eval_intvl[i] = i * step_intvl;*/
/*    }*/
/**/
/*    // Create SolveODE instance with initial state*/
/*    SolveODE solver(x0);*/
/**/
/*    // Run solveOdeControl with the test parameters*/
/*    auto soln_dt = solver.solveOdeControl(trueDynamics, x0, u0, t_eval_intvl);*/
/**/
/*    // Print out time and state results to verify output*/
/*    std::cout << "Time and State Output from solveOdeControl:\n";*/
/*    for (const auto &[time, state] : soln_dt) {*/
/*        std::cout << "Time: " << time << " State: " << state.transpose() << std::endl;*/
/*    }*/
/**/
/*    return 0;*/
/*}*/
