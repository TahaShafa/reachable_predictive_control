#include "car/car_dynamics.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <functional>

// Constructor implementation
CarDynamics::CarDynamics(Eigen::VectorXd initialStates, double Lf, double Lg, double length) : initialStates(initialStates), Lf(Lf), Lg(Lg), length(length) {
  // Constructor implementation
  initialStates = columnVectorOrientation(initialStates);
  /*if (initialStates.rows() != 4) {*/
  /*  std::cerr << "Error: There need to be four initial states [x, y, theta, v] for the car dynamics" << std::endl;*/
  /*  std::exit(EXIT_FAILURE); //Exit with failure status*/
  /*}*/

  // We have f(x0) is uniformly 0 for theta dot and acceleration
  f0 = Eigen::VectorXd(2);
  f0[0] = 0; f0[1] = 0;

  // Compute G0
  G0 = Eigen::MatrixXd(2, 2);
  //G0(0,0) = initialStates[3] / length; G0(0,1) = 0; G0(1,0) = 0; G0(1,1) = 1;
  G0(0,0) = 0; G0(0,1) = initialStates[1] / length; G0(1,0) = 1; G0(1,1) = 0;

  // Determine the smallest nonzero singular value of G0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(G0);
  Eigen::VectorXd singularValues = svd.singularValues();
  // Define a small threshold for near-zero values
  double epsilon = 1e-8;
  // Find the smallest nonzero singular value
  smallestNonzeroSingularValue = std::numeric_limits<double>::infinity();
  for (int i = 0; i < singularValues.size(); ++i) {
    if (singularValues[i] > epsilon && singularValues[i] < smallestNonzeroSingularValue) {
        smallestNonzeroSingularValue = singularValues[i];
    }
  }
  // Handle the case where no nonzero singular values are found
  if (smallestNonzeroSingularValue == std::numeric_limits<double>::infinity()) {
    std::cerr << "All singular values are zero or near-zero!" << std::endl;
    smallestNonzeroSingularValue = 0.0; // Assign a default value or handle as needed
    std::cerr << "Error: Reachable Predictive Control will trivially produce no result" << std::endl;
    std::exit(EXIT_FAILURE); //Exit with failure status
  }
}

// Getter and Setter for initial states
Eigen::VectorXd CarDynamics::getInitialStates() const {
  return initialStates;
}
void CarDynamics::setInitialStates(Eigen::VectorXd new_initialStates) {
  initialStates = new_initialStates;
}


Eigen::MatrixXd CarDynamics::getG0() const {
  return G0;
}
double CarDynamics::getSmallestNonzeroSingularValue() const {
  return smallestNonzeroSingularValue;
}
// We want to both update the G0 based on the current states and recalculate the new smallest nonzero singular value
void CarDynamics::setG0(Eigen::VectorXd states) {
  states = columnVectorOrientation(states);
  /*if (states.cols() != 4) {*/
  /*  std::cerr << "Error: There need to be four initial states [x, y, theta, v] for the car dynamics" << std::endl;*/
  /*  std::exit(EXIT_FAILURE); //Exit with failure status*/
  /*}*/

  //G0(0,0) = states[3] / length; G0(0,1) = 0; G0(1,0) = 0; G0(1,1) = 1;
  G0(0,0) = 0; G0(0,1) = states[1] / length; G0(1,0) = 1; G0(1,1) = 0;


  // Determine the smallest nonzero singular value of G0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(G0);
  Eigen::VectorXd singularValues = svd.singularValues();
  // Define a small threshold for near-zero values
  double epsilon = 1e-8;
  // Find the smallest nonzero singular value
  smallestNonzeroSingularValue = std::numeric_limits<double>::infinity();
  for (int i = 0; i < singularValues.size(); ++i) {
    if (singularValues[i] > epsilon && singularValues[i] < smallestNonzeroSingularValue) {
        smallestNonzeroSingularValue = singularValues[i];
    }
  }
  // Handle the case where no nonzero singular values are found
  if (smallestNonzeroSingularValue == std::numeric_limits<double>::infinity()) {
    std::cerr << "All singular values are zero or near-zero!" << std::endl;
    smallestNonzeroSingularValue = 0.0; // Assign a default value or handle as needed
    std::cerr << "Error: Reachable Predictive Control will trivially produce no result" << std::endl;
    std::exit(EXIT_FAILURE); //Exit with failure status
  }
}

/*Eigen::VectorXd CarDynamics::trueDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {*/
/*  // Unpack the state variables*/
/*  double x1 = states[0];*/
/*  double x2 = states[1];*/
/*  double theta =states[2];*/
/*  double v = states[3];*/
/*  double u1 = inputs[0]; // turning acceleration*/
/*  double u2 = inputs[1]; // acceleration*/
/**/
/*  Eigen::VectorXd dynamics(4);*/
/**/
/*  dynamics << v * std::cos(theta), v * std::sin(theta), v/length * u1, u2;*/
/**/
/*  return dynamics;*/
/*}*/

Eigen::VectorXd CarDynamics::trueDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {
  // Unpack the state variables
  double v = states[1];
  double u1 = inputs[0]; // turning acceleration
  double u2 = inputs[1]; // acceleration

  Eigen::VectorXd dynamics(2);

  dynamics << v/length * u2, u1;

  return dynamics;
}

// Function to return proxy dynamics as a std::function
std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> CarDynamics::getTrueDynamics() {
    return [this](double t, const Eigen::VectorXd &y, const Eigen::VectorXd &inputs) {
        return this->trueDynamics(t, y, inputs);
    };
}

/*Eigen::VectorXd CarDynamics::proxyDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {*/
/*  // Extract the last two rows as a subvector*/
/*  Eigen::VectorXd subvector_states = states.head(2);*/
/*  Eigen::VectorXd subvector_initialStates = initialStates.tail(2);*/
/*  Eigen::VectorXd distance(subvector_initialStates - subvector_states);*/
/*  double distanceNorm = distance.norm();*/
/**/
/*  // Unpack the state variables*/
/*  double x1 = states[0];*/
/*  double x2 = states[1];*/
/*  double theta = states[2];*/
/*  double v = states[3];*/
/*  double u1 = inputs[0]; // turning acceleration */
/*  double u2 = inputs[1]; // acceleration*/
/**/
/*  Eigen::VectorXd dynamics(4);*/
/*  dynamics << v * std::cos(theta), v * std::sin(theta), (smallestNonzeroSingularValue - (Lf + Lg) * distanceNorm) * u1, (smallestNonzeroSingularValue - (Lf + Lg) * distanceNorm) * u2;*/
/**/
/*  return dynamics;*/
/*}*/

Eigen::VectorXd CarDynamics::proxyDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {
  // Extract the last two rows as a subvector
  Eigen::VectorXd distance(initialStates - states);
  double distanceNorm = distance.norm();

  // Unpack the state variables
  double u1 = inputs[0]; // turning acceleration 
  double u2 = inputs[1]; // acceleration
  
  Eigen::VectorXd dynamics(2);
  dynamics << (smallestNonzeroSingularValue - (Lf + Lg) * distanceNorm) * u2, (smallestNonzeroSingularValue - (Lf + Lg) * distanceNorm) * u1;

  return dynamics;
}

// Function to return proxy dynamics as a std::function
std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> CarDynamics::getProxyDynamics() {
    return [this](double t, const Eigen::VectorXd &y, const Eigen::VectorXd &inputs) {
        return this->proxyDynamics(t, y, inputs);
    };
}
