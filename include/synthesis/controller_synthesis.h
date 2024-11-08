#ifndef CONTROLLER_SYNTHESIS_H
#define CONTROLLER_SYNTHESIS_H
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <glpk.h>
#include <iostream>


class ControllerSynthesis {
private:
    // Private member variables and methods
    double delta_t;
    double epsilon;
    int k;
    double t_final; // Duration of time in seconds we want to synthesize controller
    std::vector <double> t_span; // Overall time range
    static constexpr double defaultIntegralTimeDiscretization = 500.0; // default of 500 steps for evaluating an integral
    double integralTimeDiscretization = defaultIntegralTimeDiscretization;
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
    int iteration = 50;
    
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

    // Calculate pseduoinverse of some matrix
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &A);

  public:
    // Constructor
    ControllerSynthesis(std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> proxyDynamics, std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> trueDynamics, Eigen::VectorXd initialState, Eigen::VectorXd y, Eigen::MatrixXd inputDynamics, int stateDimension, int inputDimension, double t_final, double epsilon, double k, double delta_t);

    // Getter and setter functions
    void setY(const Eigen::VectorXd& new_y);
    Eigen::VectorXd getY() const;
    void setTheta(const std::vector<double>& new_theta);
    std::vector<double> getTheta() const;
    void setIntegralTimeDiscretization(double new_integralTimeDiscretization); //Lower number = faster runtime, less accuracy
    double getIntegralTimeDiscretization() const;
    // void setRadius(double new_r); // Can only set radius using k and delta_t
    double getRadius() const;
    void setEpsilon(double epsilon);
    double getEpsilon() const;
    void setK(int k);
    double getK() const;
    void setDelta_t(double delta_t);
    double getDelta_t() const;
    Eigen::MatrixXd getStates() const;
    Eigen::MatrixXd getInputs() const;

    // Calculate initial control input u_{0,0}
    Eigen::VectorXd initialInput();

    // Provide an initial update to the states from their initial position using u_{0,0}
    void initialTrajectory(Eigen::VectorXd u0);

    // Function that returns radius using k and delta_t
    double radius(int K = -1); // Default argument as placeholder (-1)

    // Generate learn cycle inputs (delta_u) where each inputs is \pm e_i * epsilon added with the input at u_{n,0}, i.e., generate u_{n,j}
    Eigen::MatrixXd learnCycleInputs(Eigen::VectorXd u); 

    // Function to compute the intersection between a line and a circle
    std::pair<double, Eigen::VectorXd> zSequence(const Eigen::VectorXd &center);

    // Approximate the minimum of the inner product of the gradient of d_z and f + gu by solving a linear programming problem to determine lambda
    std::pair<Eigen::VectorXd, double> dist_true(Eigen::MatrixXd& x_vec, const Eigen::VectorXd& z, double integralTimeDiscretization = defaultIntegralTimeDiscretization);

    // Synthesize control and generate control trajectory
    void synthesizeControl();

};


#endif
