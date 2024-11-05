#ifndef UAVDYNAMICS_H
#define UAVDYNAMICS_H

#include <Eigen/Dense> //Convenient library for performing operations on tensors
#include <functional>

class UavDynamics {
  private:
    const double R;
    const double l;
    const double M_rotor;
    const double M;
    const double Jx;
    const double Jy;
    const double Jz;
    const double Lf;
    const double Lg;
    Eigen::VectorXd f0; //drift dynamics around x0
    Eigen::MatrixXd G0; //input vector vield dynamics around x0
    Eigen::VectorXd initialStates;
    double smallestSingularValue;
  public:
    // Constructor
    UavDynamics(double radius, double length, double rotorMass, double mass, Eigen::VectorXd initialStates, double Lf, double Lg);
                                                    //
    // Inline getter and setter for drift dynamcs f at x0
    Eigen::VectorXd getf0() const; // Getter for f0
    void setf0(Eigen::VectorXd driftDynamics);

    // Inline getter and setter for input vector field dynamics G at x0
    Eigen::MatrixXd getG0() const; // Getter for G0
    void setG0(Eigen::MatrixXd inputDynamics);

    // Inline getter and setter for initial states (i.e., initial conditions)
    Eigen::VectorXd getInitialStates() const;
    void setInitialStates(Eigen::VectorXd new_initialStates); 

    // Inline getter for smallest singular value of G0 
    double getSmallestSingularValue() const;

    // Public member function to calculate the dynamics
    Eigen::VectorXd trueDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs); // Calculate the dynamics using a system model
    Eigen::VectorXd proxyDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs); // Calculate proxy system dynamics

    // Functions to return proxy dynamics and true dynamics as a std::function
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> getProxyDynamics();
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> getTrueDynamics();

    // Destructor
   // ~UavDynamics();
};

#endif
