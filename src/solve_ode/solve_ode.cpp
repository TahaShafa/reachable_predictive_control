#include "solve_ode/solve_ode.h"

// Constructor requires the user to set the initial state
SolveODE::SolveODE(const Eigen::VectorXd &initialState) : state(initialState) {}


// Setter and getter for current state
void SolveODE::setState(Eigen::VectorXd newState) { state = newState; }
Eigen::VectorXd SolveODE::getState() const { return state; }


// Wrap the ODE defined elsewhere to work with Boost.ODEInt 
void SolveODE::odeSystem(const std::vector<double> &y_std, std::vector<double> &dydt_std, double t, const Eigen::VectorXd &inputs, std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamicsFunc) {
    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());

    // Compute the derivative using the provided dynamics function
    Eigen::VectorXd dydt = dynamicsFunc(t, y, inputs);

    // Convert Eigen::VectorXd back to std::vector<double> for Boost.ODEInt
    Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;
}


// Integrate function to compute trajectory
std::vector<std::pair<double, Eigen::VectorXd>> SolveODE::integrate(std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamicsFunc, const Eigen::VectorXd &inputs, double t_start, double t_end, double dt) {
    std::vector<std::pair<double, Eigen::VectorXd>> trajectory;

    // Convert Eigen::VectorXd to std::vector<double> for Boost
    std::vector<double> y_std(state.data(), state.data() + state.size());

    // Define an observer function to capture trajectory at each step
    auto observer = [&](const std::vector<double> &y_std, double t) {
        Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());
        trajectory.emplace_back(t, y);
    };

    // Integrate using Boost.ODEInt's runge_kutta4 solver
    boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
    auto ode_system = [&](const std::vector<double> &y_std, std::vector<double> &dydt_std, double t) {
        odeSystem(y_std, dydt_std, t, inputs, dynamicsFunc);
    };

    boost::numeric::odeint::integrate_const(stepper, ode_system, y_std, t_start, t_end, dt, observer);

    return trajectory;
}

