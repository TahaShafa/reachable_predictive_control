#include "grs/grs.h"
#include "uav/uav_dynamics.h"
#include <cmath>
#include <random>

// Constructor
GRS::GRS(Eigen::VectorXd initStates) : initialStates(initStates) { }

// Define a setter for the initial states 
void GRS::setInitialState(Eigen::VectorXd initStates) {
  this->initialStates = initStates;
}
// Define a getter for the intitial states 
Eigen::VectorXd GRS::getInitialStates() const {
  return initialStates;
}

void GRS::setResolution(int newResolution){ resolution = newResolution; }
int GRS::getResolution() const { return resolution; }

// Returns a random reachable state at the edge of the GRS
Eigen::VectorXd GRS::randomReachableState(UavDynamics &uav, double t_start, double t_end, double dt) {
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

  // Define the type for the RK45 solver
  boost::numeric::odeint::runge_kutta4<std::vector<double>> solver;
  // Vector to store the entire trajectory
  std::vector<std::pair<double, double>> trajectory;
  integrateProxySystem(solver, initialStates, uav, u, t_start, t_end, dt, trajectory);

  auto last_element = trajectory.back();
  Eigen::VectorXd lastState(2);
  lastState << last_element.first, last_element.second;

  return lastState;
}

/*// Generate a random d-dimensional unit vector*/
/*Eigen::VectorXd GRS::randomNormEqualOne(int d) {*/
/*  Eigen::VectorXd randomVector = Eigen::VectorXd::Random(d);*/
/*  double norm = randomVector.norm();*/
/*  return randomVector/norm;*/
/*}*/

/*// Generate a ball of radius one with user-defined resolution*/
/*Eigen::MatrixXd GRS::createBall(int resolution) {*/
/*  Eigen::MatrixXd ball(2, resolution + 1);  // Create a 2x(resolution+1) matrix*/
/**/
/**/
/*  for (int i = 0; i <= resolution; ++i) {*/
/*     double angle = 2.0 * PI * static_cast<double>(i) / resolution;*/
/*     ball(0, i) = std::cos(angle);  // x-coordinate*/
/*     ball(1, i) = std::sin(angle);  // y-coordinate*/
/*  }*/
/**/
/*  return ball;*/
/*}*/
