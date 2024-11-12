#ifndef GRS_H 
#define GRS_H

#include "uav/uav_dynamics.h"
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

const double PI = 3.141592653589793;

/*class UavDynamics; // Forward declaration of UavDynamics class*/

class GRS {
  private:

    Eigen::VectorXd initialStates;

    // Generate a random d-dimensional unit vector
    Eigen::VectorXd randomNormEqualOne(int d) {
      Eigen::VectorXd randomVector = Eigen::VectorXd::Random(d);
      double norm = randomVector.norm();
      return randomVector/norm;
    }
    
    // Generate a ball of radius one with user-defined resolution
    Eigen::MatrixXd createBall(int resolution) {
      Eigen::MatrixXd ball(2, resolution + 1);  // Create a 2x(resolution+1) matrix


      for (int i = 0; i <= resolution; ++i) {
        double angle = 2.0 * PI * static_cast<double>(i) / resolution;
        ball(0, i) = std::cos(angle);  // x-coordinate
        ball(1, i) = std::sin(angle);  // y-coordinate
   }

      return ball;
    }

  public:
    GRS(Eigen::VectorXd initialStates);

    // Define a setter for the initial states 
    void setInitialState(Eigen::VectorXd initStates);
    Eigen::VectorXd getInitialStates() const;

    // Generate the proxy reachable set data
    Eigen::MatrixXd proxyReahcableSet(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, UavDynamics &uav, const Eigen::VectorXd &initialStates, const Eigen::VectorXd &inputs);

/*-----------------------------------------------------------------*/
/* Generate trajectories for the proxy system dynamics */

/*    // Define the system of ODEs using the proxy dynamics*/
/*    template <typename DynamicsClass>*/
/*    void odeSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, DynamicsClass &dynamics, const Eigen::VectorXd &inputs) {*/
/**/
/*    // Convert std::vector<double> to Eigen::VectorXd*/
/*    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());*/
/*    dynamics.setInitialStates(initialStates);*/
/**/
/*    // Compute the derivative using Eigen::VectorXd in your proxyDynamics function*/
/*    Eigen::VectorXd dydt = dynamics.proxyDynamics(t, y, inputs);*/
/**/
/*    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt*/
/*    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;*/
/*}*/
/**/
/*    // Generic integration function*/
/*    template <typename Solver, typename DynamicsClass>*/
/*    void integrateSystem(*/
/*      Solver solver,*/
/*      DynamicsClass &dynamics,*/
/*      Eigen::VectorXd &y, */
/*      Eigen::VectorXd &inputs, */
/*      double t_start, */
/*      double t_end, */
/*      double dt, */
/*      std::vector<std::pair<double, double>> &trajectory) {*/
/**/
/*    // Store the initial conditions to reset y_std after each time this is called*/
/*    Eigen::VectorXd y_initial = y;*/
/**/
/*    // Call the already defined odeProxySystem function*/
/*    auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {*/
/*        // Call the external odeProxySystem function directly*/
/*        odeSystem(y_std, dydt_std, t, dynamics, inputs);*/
/*    };*/
/**/
/*    // Convert Eigen::VectorXd to std::vector<double>*/
/*    std::vector<double> y_std(y.data(), y.data() + y.size());*/
/**/
/*    // Define an observer function to save the trajectory at each time step*/
/*    auto observer = [&](const std::vector<double> &y_std, double t) {*/
/*        trajectory.emplace_back(y_std[0], y_std[1]); // Store (y1, y2)*/
/*    };*/
/**/
/*    // Perform the integration using Boost.ODEInt's integrate_const*/
/*    boost::numeric::odeint::integrate_const(solver, ode_system, y_std, t_start, t_end, dt, observer);*/
/**/
/*    // Convert std::vector<double> back to Eigen::VectorXd (final state)*/
/*    y = Eigen::Map<Eigen::VectorXd>(y_std.data(), y_std.size());*/
/**/
/*    // Reset y back to initial conditions*/
/*    y = y_initial;*/
/*}*/

    // Define the system of ODEs using the proxy dynamics
    inline void odeProxySystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, UavDynamics &uav, const Eigen::VectorXd &inputs) {

    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());
    uav.setInitialStates(initialStates);

    // Compute the derivative using Eigen::VectorXd in your proxyDynamics function
    Eigen::VectorXd dydt = uav.proxyDynamics(t, y, inputs);

    // Ensure dydt_std has the correct size
    dydt_std.resize(dydt.size());

    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt
    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt.size()) = dydt;
}

// Generic integration function
    template <typename Solver>
void integrateProxySystem(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::pair<double, double>> &trajectory) {

    // Store the initial conditions to reset y_std after each time this is called
    Eigen::VectorXd y_initial = y;

    // Call the already defined odeProxySystem function
    auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {
        // Call the external odeProxySystem function directly
        odeProxySystem(y_std, dydt_std, t, uav, inputs);
    };

    // Convert Eigen::VectorXd to std::vector<double>
    std::vector<double> y_std(y.data(), y.data() + y.size());

    // Define an observer function to save the trajectory at each time step
    auto observer = [&](const std::vector<double> &y_std, double t) {
        trajectory.emplace_back(y_std[0], y_std[1]); // Store (y1, y2)
    };

    // Perform the integration using Boost.ODEInt's integrate_const
    boost::numeric::odeint::integrate_const(solver, ode_system, y_std, t_start, t_end, dt, observer);

    // Convert std::vector<double> back to Eigen::VectorXd (final state)
    y = Eigen::Map<Eigen::VectorXd>(y_std.data(), y_std.size());

    // Reset y back to initial conditions
    y = y_initial;
}



    // Generate the true reachable set data
    template <typename Solver>
    std::vector<std::pair<double, double>> proxyReachableSet(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, int resolution, double t_start, double t_end, double dt) {
      
      // Declare system inputs
      Eigen::VectorXd inputs(2);

      // Generate ball (unit circle) data
      Eigen::MatrixXd ball = createBall(resolution);

      // Vector to store the entire trajectory
      std::vector<std::pair<double, double>> trajectory;

      // Solve for the reachable set using (resolution) number of constant inputs of norm 1
      for (int i = 0; i <= resolution; ++i) {
        inputs << ball(0, i), ball(1, i);
        integrateProxySystem(solver, y, uav, inputs, t_start, t_end, dt, trajectory);
      }

      return trajectory;

    }



/*--------------------------------------------------------------------------*/
/* Generate trajectories for the true system dynamics */

  // Define the system of ODEs using the true dynamics
  inline void odeTrueSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, UavDynamics &uav, const Eigen::VectorXd &inputs) {

    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());

    // Compute the derivative using Eigen::VectorXd in your trueDynamics function
    Eigen::VectorXd dydt = uav.trueDynamics(t, y, inputs);

    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt
    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;
  }



  // Generic integration function for odeTrueSystem
  template <typename Solver>
  void integrateTrueSystem(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, const Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::pair<double, double>> &trajectory) {
    
    // Store the initial conditions to reset y_std after each time this is called
    Eigen::VectorXd y_initial = y;

    // Define the ODE system for true dynamics
    auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {
        odeTrueSystem(y_std, dydt_std, t, uav, inputs);
    };

    // Convert Eigen::VectorXd to std::vector<double>
    std::vector<double> y_std(y.data(), y.data() + y.size());

    // Define an observer function to save the trajectory at each time step
    auto observer = [&](const std::vector<double> &y_std, double t) {
        trajectory.emplace_back(y_std[0], y_std[1]); // Store (y1, y2)
    };

    /*// Define an observer function to print the trajectory at each time step*/
    /*auto observer = [](const std::vector<double> &y_std, double t) {*/
    /*    std::cout << "t: " << t << ", y1: " << y_std[0] << ", y2: " << y_std[1] << std::endl;*/
    /*};*/

    // Perform the integration using Boost.ODEInt's integrate_const
    boost::numeric::odeint::integrate_const(solver, ode_system, y_std, t_start, t_end, dt, observer);

    // Convert std::vector<double> back to Eigen::VectorXd (final state)
    y = Eigen::Map<Eigen::VectorXd>(y_std.data(), y_std.size());

    // Reset y back to the initial conditions
    y = y_initial;
  }



    // Generate the true reachable set data
    template <typename Solver>
    std::vector<std::pair<double, double>> trueReachableSet(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, int resolution, double t_start, double t_end, double dt) {
      
      // Declare system inputs
      Eigen::VectorXd inputs(2);

      // Generate ball (unit circle) data
      Eigen::MatrixXd ball = createBall(resolution);

      // Vector to store the entire trajectory
      std::vector<std::pair<double, double>> trajectory;

      // Solve for the reachable set using (resolution) number of constant inputs of norm 1
      for (int i = 0; i <= resolution; ++i) {
        inputs << ball(0, i), ball(1, i);
        integrateTrueSystem(solver, y, uav, inputs, t_start, t_end, dt, trajectory);
      }

      return trajectory;

    }
};

#endif
