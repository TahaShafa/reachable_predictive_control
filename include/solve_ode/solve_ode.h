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

    // Integrate function to support custom time evaluation points
    std::vector<std::pair<double, Eigen::VectorXd>> integrateTimeVec(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamicsFunc, const Eigen::VectorXd &inputs, double t_start, double t_end, const std::vector<double> &t_eval);

    // Integrate over a small interval [0, dt] with a constant input u
    std::vector<std::pair<double, Eigen::VectorXd>> solveOdeControl(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics, const Eigen::VectorXd& x0, const Eigen::VectorXd& u, const std::vector<double>& t_eval_intvl);

    // Integrate over a larger interval [0, T] with different random control inputs as per the index k
    std::vector<std::pair<double, Eigen::VectorXd>> solveOde(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics, const Eigen::VectorXd& x0, double T, const std::vector<double>& t_eval, int k, int m);

private:
    Eigen::VectorXd state;             // Initial state
};

#endif // SOLVEODE_H
