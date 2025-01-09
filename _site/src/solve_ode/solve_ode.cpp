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

// Integrate function that takes a customized time vector as an input for sampling at non-uniform time intervals
std::vector<std::pair<double, Eigen::VectorXd>> SolveODE::integrateTimeVec(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics,
    const Eigen::VectorXd &inputs, double t_start, double t_end, const std::vector<double>& t_eval) {
    std::vector<std::pair<double, Eigen::VectorXd>> trajectory;

    // Convert Eigen::VectorXd to std::vector<double> for Boost
    std::vector<double> y_std(state.data(), state.data() + state.size());

    // Define an observer to capture the trajectory
    auto observer = [&](const std::vector<double> &y_std, double t) {
        Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());
        trajectory.emplace_back(t, y);
    };

    // ODE system definition
    auto ode_system = [&](const std::vector<double> &y_std, std::vector<double> &dydt_std, double t) {
        Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(y_std.data(), y_std.size());
        Eigen::VectorXd dydt = dynamics(t, y, inputs);
        Eigen::Map<Eigen::VectorXd>(dydt_std.data(), dydt_std.size()) = dydt;
    };

    // Use integrate_times with t_eval directly
    boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
    boost::numeric::odeint::integrate_times(stepper, ode_system, y_std, t_eval, t_end, observer);

    return trajectory;
}


std::vector<std::pair<double, Eigen::VectorXd>> SolveODE::solveOdeControl(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u,
    const std::vector<double>& t_eval_intvl) {

    // Initialize solver state to x0
    setState(x0);

    // Integrate over [0, dt] with the specified evaluation points `t_eval_intvl`
    return integrateTimeVec(dynamics, u, 0.0, t_eval_intvl.back(), t_eval_intvl);
}

std::vector<std::pair<double, Eigen::VectorXd>> SolveODE::solveOde(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics,
    const Eigen::VectorXd& x0,
    double T,
    const std::vector<double>& t_eval,
    int k,
    int m) {

    // Initialize solver state to x0
    setState(x0);

    // Generate random control vector `u` as in the Python function
    Eigen::VectorXd v = Eigen::VectorXd::Random(m).normalized();
    std::vector<Eigen::VectorXd> u_arr;
    for (int i = 0; i < (1 << m); ++i) {
        Eigen::VectorXd u = v;
        for (int j = 0; j < m; ++j) {
            u[j] *= (i & (1 << j)) ? -1 : 1;
        }
        u_arr.push_back(u);
    }
    Eigen::VectorXd u = u_arr[k % u_arr.size()];

    // Integrate over [0, T] with the specified evaluation points `t_eval`
    return integrateTimeVec(dynamics, u, 0.0, T, t_eval);
}
