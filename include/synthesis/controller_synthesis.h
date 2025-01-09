#ifndef CONTROLLER_SYNTHESIS_H
#define CONTROLLER_SYNTHESIS_H
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <glpk.h>
#include <tuple>
#include <iostream>


class ControllerSynthesis {
private:
    // Private member variables and methods
    double delta_t;
    double epsilon;
    int k;
    static constexpr int defaultIntegralTimeDiscretization = 500; // default of 500 steps for evaluating an integral
    int integralTimeDiscretization = defaultIntegralTimeDiscretization;
    double t_final; // Duration of time in seconds we want to synthesize controller
    std::vector <double> t_span; // Overall time range
    std::vector<double> t_eval;
    std::vector<double> t_eval_intvl;
    int stateDimension; // Number of states
    int inputDimension; // Number of inputs
    double tau;
    double r;  // Radius of the circle dependent on k and delta_t
    Eigen::VectorXd initialState;
    Eigen::VectorXd y;  // The direction vector (target state within GRS)
    Eigen::MatrixXd inputDynamics; // G(x0)
    // Containers
    std::vector<double> theta;  // Store the theta values
    Eigen::MatrixXd states; // Container for all reached states
    Eigen::MatrixXd inputs; // Container for all synthesized controls
    // Function declaration: generates a vector of random signs
    std::vector<int> generateRandomSigns(int size);    
    std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> proxyDynamics;
    std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> trueDynamics;
    // Temporary variable determining number of iterations during control synthesis
    int iteration = 80;
    
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

    // Calculate pseduoinverse of some matrix*/ 
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &A);

    // Generate a circle of variable radius for plots
    std::vector<std::pair<double, double>> generateCircle(double radius, double x_center = 1.0, double y_center = 1.0, int numPoints = 500);

  public:
    // Constructor
    ControllerSynthesis(std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> proxyDynamics, std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> trueDynamics, Eigen::VectorXd initialState, Eigen::VectorXd y, Eigen::MatrixXd inputDynamics, int stateDimension, int inputDimension, double t_final, double epsilon, double k, double delta_t);

    // Run function to determine the initial controls and corresponding trajectory
    void initializeController();

    // Getter and setter functions
    void setY(const Eigen::VectorXd& new_y);
    Eigen::VectorXd getY() const;
    void setTheta(const std::vector<double>& new_theta);
    std::vector<double> getTheta() const;
    void setIntegralTimeDiscretization(int new_integralTimeDiscretization); //Lower number = faster runtime, less accuracy
                                                                         //
    double getIntegralTimeDiscretization() const;
    // void setRadius(double new_r); // Can only set radius using k and delta_t
    double getRadius() const;
    void setEpsilon(double epsilon);
    double getEpsilon() const;
    void setK(int k);
    double getK() const;
    void setDelta_t(double delta_t);
    double getDelta_t() const;
    void setInitialState(Eigen::VectorXd newInitialState);
    Eigen::VectorXd getInitialState() const;
    Eigen::MatrixXd getG0() const;
    void setG0(Eigen::MatrixXd newG0);
    Eigen::MatrixXd getStates() const;
    Eigen::MatrixXd getInputs() const;

    // Calculate initial control input u_{0,0}
    Eigen::VectorXd initialInput();

    // Provide an initial update to the states from their initial position using u_{0,0}
    void initialTrajectory(Eigen::VectorXd u0);

    // Generate linear target trajectory given x0 and y
    std::vector<std::pair<double, double>> generateLinearTrajectory(
      const Eigen::VectorXd& x0,
      const Eigen::VectorXd& xf,
      int steps = 500);

    // Function that returns radius using k and delta_t
    double radius(int K = -1); // Default argument as placeholder (-1)

    // Generate learn cycle inputs (delta_u) where each inputs is \pm e_i * epsilon added with the input at uLLL_{n,0}, i.e., generate u_{n,j}
    Eigen::MatrixXd learnCycleInputs(Eigen::VectorXd u); 

    // Function to compute the intersection between a line and a circle
    std::pair<double, Eigen::VectorXd> zSequence(const Eigen::VectorXd &center);

    // Approximate the minimum of the inner product of the gradient of d_z and f + gu by solving a linear programming problem to determine lambda
    std::pair<Eigen::VectorXd, double> dist_true(const Eigen::VectorXd& z);

    // Synthesize control and generate control trajectory
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> synthesizeControl(bool view_plot = false, bool save_plot = false, int controlIteration = 0);

    // Save plots to figures
    void savePlotToFigures(
      const std::vector<std::pair<double, double>> &synthesizedTrajectory,
      const std::vector<std::pair<double, double>> &desiredTrajectory,
      int iteration = 0,
      int controlIteration = 0,
      const std::vector<std::pair<double, double>> &convergentRadius = {},
      const std::vector<std::vector<double>> &reachableSet = {});

        // Save plots to figures
    void savePlotToFigures_temporary(
      const std::vector<std::pair<double, double>> &synthesizedTrajectory,
      const std::vector<std::pair<double, double>> &desiredTrajectory,
      int iteration = 0,
      int controlIteration = 0,
      const std::vector<std::pair<double, double>> &convergentRadius = {},
      const std::vector<std::vector<double>> &reachableSet = {});
};



#endif
