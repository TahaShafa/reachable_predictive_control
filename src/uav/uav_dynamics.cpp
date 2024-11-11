#include "uav/uav_dynamics.h"
#include <cmath>
#include <algorithm>

const double PI = 3.141592653589793;

// Constructor implementation
UavDynamics::UavDynamics(Eigen::VectorXd initStates, double lf, double lg, double radius, double length, double rotorMass, double mass)
  : R(radius), l(length), M_rotor(rotorMass), M(mass), initialStates(initStates), Lf(lf), Lg(lg),
    Jx(2 * M * std::pow(R, 2) / 5 + 2 * std::pow(l,2) * M_rotor),
    Jy(2 * M * std::pow(R, 2) / 5 + 2 * std::pow(l,2) * M_rotor),
    Jz(2 * M * std::pow(R, 2) / 5 + 4 * std::pow(l,2) * M_rotor) {

      // Compute f(x0) as a function of Jx, Jy, Jz
      f0 = Eigen::VectorXd(2);
      f0[0] = (Jy - Jz) / Jx * (initStates[1] + 10) * PI/2;
      f0[1] = (Jz - Jx) / Jy * (initStates[0] + 15) * PI/2;

      // Compute G(x0) as a function of Jx, Jy, Jz
      G0 = Eigen::MatrixXd(2, 2); 
      G0(0,0) = 1/Jx; G0(0,1) = 0; G0(1,0) = 0; G0(1,1) = 1/Jy;

      // Determine the smallest singular value of G0
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(G0);
      Eigen::VectorXd singularValues = svd.singularValues();
      auto minElement = std::min_element(singularValues.data(), singularValues.data() + singularValues.size());
      smallestSingularValue = *minElement; 
}

// Getter for drift dynamics f at x0
Eigen::VectorXd UavDynamics::getf0() const {
  return f0;
}

// Getter for input vector field dynamics G at x0
Eigen::MatrixXd UavDynamics::getG0() const {
  return G0;
}

// Getter for smallest singular value of G at x0 
double UavDynamics::getSmallestSingularValue() const {
  return smallestSingularValue;
}

// Setter for drift uav_dynamics
void UavDynamics::setf0(Eigen::VectorXd driftDynamics){
  f0 = driftDynamics;
}

// Getter and Setter for initial states
Eigen::VectorXd UavDynamics::getInitialStates() const {
  return initialStates;
}
void UavDynamics::setInitialStates(Eigen::VectorXd new_initialStates) {
  initialStates = new_initialStates;
}

// Setter for input vector field dynamics
// Automatically calculate and update the smallest nonzero singular value once set
void UavDynamics::setG0(Eigen::MatrixXd inputDynamics){
  G0 = inputDynamics;
  // Recalculate the new smallest singular value based on new G0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(G0);
  Eigen::VectorXd singularValues = svd.singularValues();     
  auto minElement = std::min_element(singularValues.data(), singularValues.data() + singularValues.size());
      smallestSingularValue = *minElement;
}

// Calculate proxy system dynamics whose reachable set is contained in the reachable set of the uav dynamics
Eigen::VectorXd UavDynamics::proxyDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {
  Eigen::VectorXd v(initialStates - states);
  double normValue = v.norm();
  double x1= states[0]; double x2 = states[1];
  double u1 = inputs[0]; double u2 = inputs[1];
  Eigen::VectorXd dynamics(2);
  dynamics << f0[0] + (smallestSingularValue - (Lf + Lg) * normValue) * u1, f0[1] + (smallestSingularValue - (Lf + Lg) * normValue) * u2;
  return dynamics;
}

// Function to return proxy dynamics as a std::function
std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> UavDynamics::getProxyDynamics() {
    return [this](double t, const Eigen::VectorXd &y, const Eigen::VectorXd &inputs) {
        return this->proxyDynamics(t, y, inputs);
    };
}

// Calculate the true dynamics using the UAV model
Eigen::VectorXd UavDynamics::trueDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs) {
  double x1= states[0]; double x2 = states[1];
  double u1 = inputs[0]; double u2 = inputs[1];
  Eigen::VectorXd dynamics(2); 
  dynamics << (Jy - Jz) / Jx * (x2 + 10) * PI/2 + 1/Jx * u1, (Jz - Jx) / Jy * (x1 + 15) * PI/2 + 1/Jy * u2;
  return dynamics;
}

// Function to return proxy dynamics as a std::function
std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> UavDynamics::getTrueDynamics() {
    return [this](double t, const Eigen::VectorXd &y, const Eigen::VectorXd &inputs) {
        return this->trueDynamics(t, y, inputs);
    };
}
