#ifndef GRS_H 
#define GRS_H

#include "uav/uav_dynamics.h"
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <random>

const double PI = 3.141592653589793;

/*class UavDynamics; // Forward declaration of UavDynamics class*/

class GRS {
  private:

    Eigen::VectorXd initialStates;
    static constexpr int defaultResolution = 2000;
    int resolution = defaultResolution;

    // Generate a random d-dimensional unit vector
    Eigen::VectorXd randomNormEqualOne(int d) {
      Eigen::VectorXd randomVector = Eigen::VectorXd::Random(d);
      double norm = randomVector.norm();
      return randomVector/norm;
    }
    
    // Generate a ball of radius one with user-defined resolution
    Eigen::MatrixXd createBall(int resolution = defaultResolution) {
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
    void setResolution(int newResolution);
    int getResolution() const;

    /*// Assign random point at the boundary of the proxy reachable set*/
    /*Eigen::VectorXd randomReachableState(UavDynamics &uav, double t_start, double t_end, double dt);*/

  // Returns a random reachable state at the edge of the GRS
template <typename Dynamics>
Eigen::VectorXd randomReachableState(Dynamics &dynamics, double t_start, double t_end, double dt) {
  // Define an upper limit
  const int upperLimit = resolution;

  // Create a random device and seed the random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the distribution range
  std::uniform_int_distribution<> dist(0 , upperLimit);

  // Generate a random number integer in the specified range
  int randomNumber = dist(gen);
  // Choose an input u of maximal norm on a random point on the edge of a 2D circle
  double angle = 2.0 * PI * static_cast<double>(randomNumber) / upperLimit;
  // We'll want to be smarter about how we initialize this as a function of the number of states and not simply a hard-coded 2
  Eigen::VectorXd u(2);
  u << std::cos(angle), std::sin(angle);

  std::cout << "Checkpoint 0" << std::endl;

  // Define the type for the RK45 solver
  boost::numeric::odeint::runge_kutta4<std::vector<double>> solver;
  // Vector to store the entire trajectory
  //std::vector<std::pair<double, double>> trajectory;
  std::vector<std::vector<double>> trajectory;
  integrateProxySystem(solver, initialStates, dynamics, u, t_start, t_end, dt, trajectory);

  std::cout << "Checkpoint 1" << std::endl;
  
  /*  auto last_element = trajectory.back();*/
  /*Eigen::VectorXd lastState(2);*/
  /*lastState << last_element.first, last_element.second;*/

  // Get the last state from the trajectory
  auto last_element = trajectory.back();
  Eigen::VectorXd lastState(last_element.size()); // Dynamically resize to the size of `last_element`

  // Copy the values from the last element of the trajectory to `lastState`
  for (size_t i = 0; i < last_element.size(); ++i) {
    lastState[i] = last_element[i];
  }

  return lastState;
}


/*-----------------------------------------------------------------*/
/* Generate trajectories for the proxy system dynamics */

    // Define the system of ODEs using the proxy dynamics
    template <typename Dynamics>
    inline void odeProxySystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, Dynamics &dynamics, const Eigen::VectorXd &inputs) {

    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());
    dynamics.setInitialStates(initialStates);

    // Compute the derivative using Eigen::VectorXd in your proxyDynamics function
    Eigen::VectorXd dydt = dynamics.proxyDynamics(t, y, inputs);

    // Ensure dydt_std has the correct size
    dydt_std.resize(dydt.size());

    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt
    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt.size()) = dydt;
}

  // Generic integration function
  template <typename Solver, typename Dynamics>
  void integrateProxySystem(Solver solver, Eigen::VectorXd &y, Dynamics &dynamics, Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::vector<double>> &trajectory) {

    // Store the initial conditions to reset y_std after each time this is called
    Eigen::VectorXd y_initial = y;
    
    std::cout << "Checkpoint 2" << std::endl;

    // Call the already defined odeProxySystem function
auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {
    std::cout << "ODE System: t = " << t << ", y_std = ";
    for (const auto& y : y_std) std::cout << y << " ";
    std::cout << std::endl;

    odeProxySystem(y_std, dydt_std, t, dynamics, inputs);
};

    std::cout << "Checkpoint 3" << std::endl;

    // Convert Eigen::VectorXd to std::vector<double>
    std::vector<double> y_std(y.data(), y.data() + y.size());

    std::cout << "Checkpoint 4" << std::endl;

    /*// Define an observer function to save the trajectory at each time step*/
    /*auto observer = [&](const std::vector<double> &y_std, double t) {*/
    /*    trajectory.emplace_back(y_std[0], y_std[1]); // Store (y1, y2)*/
    /*};*/


auto observer = [&](const std::vector<double>& y_std, double t) {
    std::cout << "Observer: t = " << t << ", y_std = ";
    for (const auto& y : y_std) std::cout << y << " ";
    std::cout << std::endl;

    trajectory.emplace_back(std::vector<double>(y_std.begin(), y_std.end()));
};


    std::cout << "Checkpoint 5" << std::endl;

    // Perform the integration using Boost.ODEInt's integrate_const
    boost::numeric::odeint::integrate_const(solver, ode_system, y_std, t_start, t_end, dt, observer);

    std::cout << "Checkpoint 6" << std::endl;

    // Convert std::vector<double> back to Eigen::VectorXd (final state)
    y = Eigen::Map<Eigen::VectorXd>(y_std.data(), y_std.size());

    // Reset y back to initial conditions
    y = y_initial;
}

    // Generate the true reachable set data
    template <typename Solver, typename Dynamics>
    std::vector<std::vector<double>> proxyReachableSet(Solver solver, Eigen::VectorXd &y, Dynamics &dynamics, double t_start, double t_end, double dt, int resolution = defaultResolution) {
      
      // Declare system inputs
      Eigen::VectorXd inputs(2);

      // Generate ball (unit circle) data
      Eigen::MatrixXd ball = createBall(resolution);

      // Vector to store the entire trajectory
      //std::vector<std::pair<double, double>> trajectory;
      std::vector<std::vector<double>> trajectory;


      // Solve for the reachable set using (resolution) number of constant inputs of norm 1
      for (int i = 0; i <= resolution; ++i) {
        inputs << ball(0, i), ball(1, i);
        integrateProxySystem(solver, y, dynamics, inputs, t_start, t_end, dt, trajectory);
      }

      return trajectory;
    }


/*    // Define the system of ODEs using the proxy dynamics*/
/*    inline void odeProxySystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, UavDynamics &uav, const Eigen::VectorXd &inputs) {*/
/**/
/*    // Convert std::vector<double> to Eigen::VectorXd*/
/*    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());*/
/*    uav.setInitialStates(initialStates);*/
/**/
/*    // Compute the derivative using Eigen::VectorXd in your proxyDynamics function*/
/*    Eigen::VectorXd dydt = uav.proxyDynamics(t, y, inputs);*/
/**/
/*    // Ensure dydt_std has the correct size*/
/*    dydt_std.resize(dydt.size());*/
/**/
/*    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt*/
/*    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt.size()) = dydt;*/
/*}*/
/**/
/*  // Generic integration function*/
/*  template <typename Solver>*/
/*  void integrateProxySystem(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::pair<double, double>> &trajectory) {*/
/**/
/*    // Store the initial conditions to reset y_std after each time this is called*/
/*    Eigen::VectorXd y_initial = y;*/
/**/
/*    // Call the already defined odeProxySystem function*/
/*    auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {*/
/*        // Call the external odeProxySystem function directly*/
/*        odeProxySystem(y_std, dydt_std, t, uav, inputs);*/
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

    /*// Generate the true reachable set data*/
    /*template <typename Solver>*/
    /*std::vector<std::pair<double, double>> proxyReachableSet(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, double t_start, double t_end, double dt, int resolution = defaultResolution) {*/
    /**/
    /*  // Declare system inputs*/
    /*  Eigen::VectorXd inputs(2);*/
    /**/
    /*  // Generate ball (unit circle) data*/
    /*  Eigen::MatrixXd ball = createBall(resolution);*/
    /**/
    /*  // Vector to store the entire trajectory*/
    /*  std::vector<std::pair<double, double>> trajectory;*/
    /**/
    /*  // Solve for the reachable set using (resolution) number of constant inputs of norm 1*/
    /*  for (int i = 0; i <= resolution; ++i) {*/
    /*    inputs << ball(0, i), ball(1, i);*/
    /*    integrateProxySystem(solver, y, uav, inputs, t_start, t_end, dt, trajectory);*/
    /*  }*/
    /**/
    /*  return trajectory;*/
    /*}*/


/*--------------------------------------------------------------------------*/
/* Generate trajectories for the true system dynamics */

  // Define the system of ODEs using the true dynamics
  template <typename Dynamics>
  inline void odeTrueSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, Dynamics &dynamics, const Eigen::VectorXd &inputs) {

    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());

    // Compute the derivative using Eigen::VectorXd in your trueDynamics function
    Eigen::VectorXd dydt = dynamics.trueDynamics(t, y, inputs);

    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt
    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;
  }


  // Generic integration function for odeTrueSystem
  template <typename Solver, typename Dynamics>
  void integrateTrueSystem(Solver solver, Eigen::VectorXd &y, Dynamics &dynamics, const Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::vector<double>> &trajectory) {
    
    // Store the initial conditions to reset y_std after each time this is called
    Eigen::VectorXd y_initial = y;

    // Define the ODE system for true dynamics
    auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {
        odeTrueSystem(y_std, dydt_std, t, dynamics, inputs);
    };

    // Convert Eigen::VectorXd to std::vector<double>
    std::vector<double> y_std(y.data(), y.data() + y.size());

    // Define an observer function to save the trajectory at each time step
auto observer = [&](const std::vector<double>& y_std, double t) {
    std::cout << "Observer: t = " << t << ", y_std = ";
    for (const auto& y : y_std) std::cout << y << " ";
    std::cout << std::endl;

    trajectory.emplace_back(std::vector<double>(y_std.begin(), y_std.end()));
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
    template <typename Solver, typename Dynamics>
    std::vector<std::vector<double>> trueReachableSet(Solver solver, Eigen::VectorXd &y, Dynamics &dynamics, double t_start, double t_end, double dt, int resolution = defaultResolution) {
      
      // Declare system inputs
      Eigen::VectorXd inputs(2);

      // Generate ball (unit circle) data
      Eigen::MatrixXd ball = createBall(resolution);

      // Vector to store the entire trajectory
      std::vector<std::vector<double>> trajectory;

      std::cout << "True Checkpoint 0" << std::endl;

      // Solve for the reachable set using (resolution) number of constant inputs of norm 1
      for (int i = 0; i <= resolution; ++i) {
        inputs << ball(0, i), ball(1, i);
        integrateTrueSystem(solver, y, dynamics, inputs, t_start, t_end, dt, trajectory);
      }

      return trajectory;

    }

/*--------------------------------------------------------------------------*/
/* Generate trajectories for the true system dynamics */

  /*// Define the system of ODEs using the true dynamics*/
  /*inline void odeTrueSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, UavDynamics &uav, const Eigen::VectorXd &inputs) {*/
  /**/
  /*  // Convert std::vector<double> to Eigen::VectorXd*/
  /*  Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());*/
  /**/
  /*  // Compute the derivative using Eigen::VectorXd in your trueDynamics function*/
  /*  Eigen::VectorXd dydt = uav.trueDynamics(t, y, inputs);*/
  /**/
  /*  // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt*/
  /*  Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;*/
  /*}*/
  /**/
  /**/
  /**/
  /*// Generic integration function for odeTrueSystem*/
  /*template <typename Solver>*/
  /*void integrateTrueSystem(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, const Eigen::VectorXd &inputs, double t_start, double t_end, double dt, std::vector<std::pair<double, double>> &trajectory) {*/
  /**/
  /*  // Store the initial conditions to reset y_std after each time this is called*/
  /*  Eigen::VectorXd y_initial = y;*/
  /**/
  /*  // Define the ODE system for true dynamics*/
  /*  auto ode_system = [&](const std::vector<double>& y_std, std::vector<double>& dydt_std, double t) {*/
  /*      odeTrueSystem(y_std, dydt_std, t, uav, inputs);*/
  /*  };*/
  /**/
  /*  // Convert Eigen::VectorXd to std::vector<double>*/
  /*  std::vector<double> y_std(y.data(), y.data() + y.size());*/
  /**/
  /*  // Define an observer function to save the trajectory at each time step*/
  /*  auto observer = [&](const std::vector<double> &y_std, double t) {*/
  /*      trajectory.emplace_back(y_std[0], y_std[1]); // Store (y1, y2)*/
  /*  };*/
  /**/
  /*  // Perform the integration using Boost.ODEInt's integrate_const*/
  /*  boost::numeric::odeint::integrate_const(solver, ode_system, y_std, t_start, t_end, dt, observer);*/
  /**/
  /*  // Convert std::vector<double> back to Eigen::VectorXd (final state)*/
  /*  y = Eigen::Map<Eigen::VectorXd>(y_std.data(), y_std.size());*/
  /**/
  /*  // Reset y back to the initial conditions*/
  /*  y = y_initial;*/
  /*}*/
  /**/
  /**/
  /**/
  /*  // Generate the true reachable set data*/
  /*  template <typename Solver>*/
  /*  std::vector<std::pair<double, double>> trueReachableSet(Solver solver, Eigen::VectorXd &y, UavDynamics &uav, double t_start, double t_end, double dt, int resolution = defaultResolution) {*/
  /**/
  /*    // Declare system inputs*/
  /*    Eigen::VectorXd inputs(2);*/
  /**/
  /*    // Generate ball (unit circle) data*/
  /*    Eigen::MatrixXd ball = createBall(resolution);*/
  /**/
  /*    // Vector to store the entire trajectory*/
  /*    std::vector<std::pair<double, double>> trajectory;*/
  /**/
  /*    // Solve for the reachable set using (resolution) number of constant inputs of norm 1*/
  /*    for (int i = 0; i <= resolution; ++i) {*/
  /*      inputs << ball(0, i), ball(1, i);*/
  /*      integrateTrueSystem(solver, y, uav, inputs, t_start, t_end, dt, trajectory);*/
  /*    }*/
  /**/
  /*    return trajectory;*/
  /**/
  /*  }*/
};

#endif
