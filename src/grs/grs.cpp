#include "grs/grs.h"
#include "Eigen/src/Core/Matrix.h"
#include <cmath>
// #include "uav/uav_dynamics.h"

/*const double PI = 3.141592653589793;*/

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
