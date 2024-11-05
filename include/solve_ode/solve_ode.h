#ifndef SOLVEODE_H
#define SOLVEODE_H

#include <Eigen/Dense>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <utility> // for std::pair

class SolveODE {
public:

  void odeSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, const Eigen::VectorXd &inputs, std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamicsFunc);

    SolveODE(const Eigen::VectorXd &initialState);

    void setState(Eigen::VectorXd newState);
    Eigen::VectorXd getState() const ;

    // Integrate function to compute trajectory
    std::vector<std::pair<double, Eigen::VectorXd>> integrate(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamicsFunc, const Eigen::VectorXd &inputs, double t_start, double t_end, double dt);

private:
    Eigen::VectorXd state;             // Initial state
};

#endif // SOLVEODE_H
