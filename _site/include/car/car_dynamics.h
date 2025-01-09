#ifndef CARDYNAMICS_H
#define CARDYNAMICS_H

#include <Eigen/Dense>
#include <functional>

class CarDynamics {
  private:
    const double length;
    const double Lf;
    const double Lg;
    Eigen::VectorXd f0; //drift dynamics around x0
    Eigen::MatrixXd G0; //input vector vield dynamics around x0
    Eigen::VectorXd initialStates; //[x, y, v, theta]
    double smallestNonzeroSingularValue;

    // Orient to column vector for consistency in calculations
    Eigen::VectorXd columnVectorOrientation(const Eigen::VectorXd &vector) {
      Eigen::VectorXd container;
      container.conservativeResize(vector.size());
      if (vector.cols() == 1 && vector.rows() > 1) {
        container.col(0) = vector.transpose();
      } else {
        container.col(0) = vector;
      }
      return container;
    }

  public:
    // Constructor
    CarDynamics(Eigen::VectorXd initialStates, double Lf, double Lg, double length);

    void setG0(Eigen::VectorXd states);
    Eigen::MatrixXd getG0() const;
    double getSmallestNonzeroSingularValue() const;
    // Inline getter and setter for initial states (i.e., initial conditions)
    Eigen::VectorXd getInitialStates() const;
    void setInitialStates(Eigen::VectorXd new_initialStates); 


    Eigen::VectorXd trueDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs); // Calculate the dynaimmcs using a system model
    Eigen::VectorXd proxyDynamics(double t, Eigen::VectorXd states, Eigen::VectorXd inputs); // Calculate proxy system dynamics

  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> getTrueDynamics();
  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> getProxyDynamics();
};

#endif // CARDYNAMICS_HPP
