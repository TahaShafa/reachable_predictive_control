#include "car/car_dynamics.h"
#include "gnuplot-iostream.h"
#include "uav/uav_dynamics.h"
#include "grs/grs.h"
#include "synthesis/controller_synthesis.h"
#include "solve_ode/solve_ode.h"
#include <boost/math/policies/policy.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <functional>
#include <fstream>

// Define the type for the RK45 solver
boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;

void printLastTwoRows(const std::vector<std::vector<double>>& trajectory, const std::string& name) {
    if (trajectory.size() < 2) {
        std::cerr << "Error: Not enough rows in " << name << " to print the last two rows." << std::endl;
        return;
    }
    
    std::cout << "Last two rows of " << name << ":" << std::endl;

    // Print the second last row
    std::cout << "Row " << trajectory.size() - 2 << ": ";
    for (const auto& val : trajectory[trajectory.size() - 2]) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    // Print the last row
    std::cout << "Row " << trajectory.size() - 1 << ": ";
    for (const auto& val : trajectory[trajectory.size() - 1]) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

void printLastTwoColumns(const std::vector<std::vector<double>>& trajectory) {
    for (const auto& row : trajectory) {
        if (row.size() >= 2) { // Ensure the row has at least two columns
            std::cout << row[row.size() - 2] << " " << row[row.size() - 1] << std::endl;
        } else {
            std::cerr << "Row has fewer than 2 columns!" << std::endl;
        }
    }
}

// Function to extract the last two columns from a trajectory
std::vector<std::pair<double, double>> extractLastTwoColumns(const std::vector<std::vector<double>>& trajectory) {
    std::vector<std::pair<double, double>> lastTwoColumns;

    for (const auto& row : trajectory) {
        if (row.size() >= 2) {
            lastTwoColumns.emplace_back(row[row.size() - 2], row[row.size() - 1]);
        } else {
            std::cerr << "Row has fewer than 2 columns!" << std::endl;
        }
    }
    return lastTwoColumns;
}

void printTrajectorySize(const std::vector<std::vector<double>>& trajectory) {
    // Number of rows
    size_t numRows = trajectory.size();

    // Number of columns (assuming all rows have the same size)
    size_t numCols = numRows > 0 ? trajectory[0].size() : 0;

    std::cout << "ProxyCarTrajectory size: " << numRows << " rows by " << numCols << " columns" << std::endl;
}

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
  y_initial << 0.0, 1.0;
  GRS grs_uav(y_initial);

  // Define parameters for plotting the GRS
  int resolution = 2000;  // Number of resolution points on the circle
  double t_start = 0.0;
  double t_end = 0.2;
  double dt = 0.1;

  Eigen::VectorXd y = grs_uav.randomReachableState(uav, t_start, t_end, dt); 

  std::cout << "Boundary State: " << y << std::endl;

  // Call `proxyReachableSet` function with the RK45 solver
  std::vector<std::vector<double>> proxyTrajectory = grs_uav.proxyReachableSet(stepper, y_initial, uav, t_start, t_end, dt);

  std::vector<std::vector<double>> trueTrajectory = grs_uav.trueReachableSet(stepper, y_initial, uav, t_start, t_end, dt);

  // Initialize Gnuplot
  /*Gnuplot gp;*/
  /*gp << "set title 'Reachable Set Trajectories'\n";*/
  /*gp << "set xlabel 'X'\n";*/
  /*gp << "set ylabel 'Y'\n";*/
  /*//gp << "plot '-' with lines title 'Proxy Reachable Set'\n";*/
  /*gp << "plot '-' with lines title 'True Reachable Set', '-' with lines title 'Proxy Reachable Set'\n";*/
  /*gp.send1d(trueTrajectory);*/
  /*gp.send1d(proxyTrajectory);*/
  
  Eigen::VectorXd carInitialState(2);
  carInitialState << 0.0, 0.0;
  double length = 1.0;
  CarDynamics car(carInitialState, Lf, Lg, length);

  Eigen::VectorXd y_initial_car(2);
  y_initial_car << 0.0, 0.0;
  GRS grs_car(y_initial_car);

  std::cout << "Passed this point" << std::endl;

  //Eigen::VectorXd y_car = grs_car.randomReachableState(car, t_start, t_end, dt);

  //std::cout << "Boundary State for Car: " << y_car << std::endl;

  // Call `proxyReachableSet` function with the RK45 solver
  std::vector<std::vector<double>> proxyCarTrajectory = grs_car.proxyReachableSet(stepper, carInitialState, car, t_start, t_end, dt);

  std::vector<std::vector<double>> trueCarTrajectory = grs_car.trueReachableSet(stepper, carInitialState, car, t_start, t_end, dt);

  // Extract last two columns
    std::vector<std::pair<double, double>> proxySubCarTrajectory = extractLastTwoColumns(proxyCarTrajectory);
    std::vector<std::pair<double, double>> trueSubCarTrajectory = extractLastTwoColumns(trueCarTrajectory);

    /*  // Initialize Gnuplot*/
    /*Gnuplot gp2;*/
    /*gp2 << "set title 'Reachable Set Trajectories'\n";*/
    /*gp2 << "set xlabel 'velocity'\n";*/
    /*gp2 << "set ylabel 'theta'\n";*/
    /*gp2 << "plot '-' with lines title 'True Reachable Set'\n";*/
    /*//gp << "plot '-' with lines title 'True Reachable Set', '-' with lines title 'Proxy Reachable Set'\n";*/
    /**/
    /*// Send the data to Gnuplot*/
    /*gp2.send1d(trueSubCarTrajectory);*/
    /*//gp.send1d(proxySubCarTrajectory);*/
    /**/
    /*Gnuplot gp3;*/
    /*gp3 << "set title 'Reachable Set Trajectories'\n";*/
    /*gp3 << "set xlabel 'theta'\n";*/
    /*gp3 << "set ylabel 'velocity'\n";*/
    /*gp3 << "plot '-' with lines title 'Proxy Reachable Set'\n";*/
    /*//gp << "plot '-' with lines title 'True Reachable Set', '-' with lines title 'Proxy Reachable Set'\n";*/
    /**/
    /*// Send the data to Gnuplot*/
    /*gp3.send1d(proxySubCarTrajectory);*/
  // Print the last two rows
  //printLastTwoRows(proxyCarTrajectory, "proxyCarTrajectory");
  //printTrajectorySize(proxyCarTrajectory);


    /*// Create an instance of your dynamics class (replace with actual initialization)*/
    /*UavDynamics uav_dynamics(initialState, Lf, Lg);*/
    /**/
    /*auto uav_proxyDynamics = uav_dynamics.getProxyDynamics();*/
    /*auto uav_trueDynamics = uav_dynamics.getTrueDynamics();*/
    int stateDimension = 2;
    int inputDimension = 2;
    double k = 5.0;                 
    double epsilon = 0.1;
    double t_final = 0.2;
    double delta_t = 0.0015;
    /*Eigen::MatrixXd G0 = uav_dynamics.getG0();*/
    /*//Eigen::VectorXd y_final(2);*/
    /*//y_final << -16.6228, 12.3907;*/
    /*Eigen::VectorXd y_final = y;*/
    /**/
    /*// Instantiate ControllerSynthesis with the parameters*/
    /*ControllerSynthesis uav_controller(uav_proxyDynamics, uav_trueDynamics, initialState, y_final, G0, stateDimension, inputDimension, t_final, epsilon, k, delta_t);*/
    /**/
    // Run the test
    bool view_plot = false; // Generate the plot
    bool save_plot = true; // Save the plot
                       // 

    /*int number_trajectories = 3;*/
    /*for (int i = 0; i < number_trajectories; i++) {*/
    /*  auto [y_initial, inputs] = uav_controller.synthesizeControl(view_plot); //Follow piecewise linear trajectory*/
    /*  std::cout << "y_initial: " << y_initial << std::endl;*/
    /*  grs_uav.setInitialState(y_initial); //reinitialize the initial condition to final state reached using the controller*/
    /*  uav_controller.seInitialState(y_initial);*/
    /*  y_final = grs_uav.randomReachableState(uav, t_start, t_end, dt); //Randomly generate new goal state*/
    /*  uav_controller.setY(y_final); //Set new goal state*/
    /*  uav_controller.initializeController();*/
    /*  std::cout << "Input Size: " << inputs.cols() << std::endl;*/
    /*  std::cout << "Last Input:  << inputs.col(inputs.cols() - 1).transpose() << std::endl;*/
   /*}*/

    // Create an instance of the car dynamics class
    length = 1.0;
    int carStateDimension = 2;
    CarDynamics car_dynamics(carInitialState, Lf, Lg, length);

    Eigen::VectorXd y_car(2);
    y_car << 0.0, 0.2;
    Eigen::VectorXd y_adjusted(2);
    //y_car = y_car / y_car.norm();
    //y_car = y_car * t_final; // t_final is the radius of the ball underapproximation of the GRS as the dynamics are modeled currently

    // Initialize controller for car
    auto car_proxyDynamics = car_dynamics.getProxyDynamics();
    auto car_trueDyanmics = car_dynamics.getTrueDynamics();
    Eigen::MatrixXd G0_car = car_dynamics.getG0();
    ControllerSynthesis car_controller(car_proxyDynamics, car_trueDyanmics, carInitialState, y_car, G0_car, carStateDimension, inputDimension, t_final, epsilon, k, delta_t);

    int number_trajectories = 20;
    for (int i = 0; i < number_trajectories; i++) {
      auto [y_initial, inputs, states] = car_controller.synthesizeControl(view_plot, save_plot, i);
      std::cout << "Number of inputs: " << inputs.cols() << std::endl;
      //Follow piecewise linear trajectory
      /*if (i >= 2) {*/
      /*  if (i >= 3){*/
      /*    y_car << 1.0, 0.0;*/
      /*    y_car = y_car / y_car.norm();*/
      /*    y_car = y_car * t_final; */
      /*  }*/
      /*  y_car << -1.0, 0.0;*/
      /*  y_car = y_car / y_car.norm();*/
      /*  y_car = y_car * t_final;*/
      /*}*/
      if (y_initial[1] > 1.0) {
        // Open a file to write
        std::ofstream outputFile("inputs_velocity_recorded_circle.txt");

        if (!outputFile) {
          std::cerr << "Error opening file for writing!" << std::endl;
          return 1;
        }

        std::cout << "Flag on" << std::endl;
        std::cout << "Number of inputs: " << inputs.cols() << std::endl;
            // Write every two entries per line
        for (size_t i = 0; i < inputs.size(); i += 2) {
          outputFile << inputs(i); // Write the first entry
          if (i + 1 < inputs.size()) { // Check if there is a second entry
            outputFile << " " << inputs(i + 1); // Write the second entry
          }
        outputFile << "\n"; // Move to the next line
        }

        std::cout << "y_initial: " << y_initial << std::endl;
        break;
      }
      //reinitialize the initial condition to final state reached using the controller
      grs_car.setInitialState(y_initial); 
      car_controller.setInitialState(y_initial);
      car_dynamics.setInitialStates(y_initial);
      car_dynamics.setG0(y_initial);
      // Get the new dynamics and enter the corresponding initial conditions
      Eigen::MatrixXd inputDynamics = car_dynamics.getG0();
      std::cout << "Input Dynamics: " << inputDynamics << std::endl << "Smallest Nonzero Singular Value: " << car_dynamics.getSmallestNonzeroSingularValue() << std::endl;
      y_initial << 0.0, y_initial[1];
      // Set new final desired position for your controller to navigate to
      double car_smallestSingularValue = car_dynamics.getSmallestNonzeroSingularValue();
      y_adjusted << 0.0, y_car[1] * car_smallestSingularValue;
      Eigen::VectorXd y_final = y_initial + y_adjusted; // On assumption that v = 1
      car_controller.setY(y_final); //Set new goal state
      car_controller.setG0(inputDynamics);
      car_controller.initializeController();
    }

    //car_controller.synthesizeControl(view_plot, save_plot);

  return 0;
}
