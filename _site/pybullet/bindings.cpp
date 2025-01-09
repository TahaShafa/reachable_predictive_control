#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>

#include "synthesis/controller_synthesis.h"
#include "uav/uav_dynamics.h"
#include "car/car_dynamics.h"
#include "grs/grs.h"
#include "solve_ode/solve_ode.h"

namespace py = pybind11;

PYBIND11_MODULE(pybullet_module, m) {
    m.doc() = "Python bindings for RPC project";  // Optional module docstring

    // Wrap UAV Dynamics class
    py::class_<CarDynamics>(m, "CarDynamics")
        .def(py::init<Eigen::VectorXd, double, double, double>(),  
         py::arg("initialStates"), py::arg("Lf"), py::arg("Lg"), py::arg("length"))
        .def("getProxyDynamics", &CarDynamics::getProxyDynamics)  // Expose simulate() method
        .def("getTrueDynamics", &CarDynamics::getTrueDynamics)  // Expose getState() method
        .def("getG0", &CarDynamics::getG0)
        .def("setG0", &CarDynamics::setG0)
        .def("setInitialStates", &CarDynamics::setInitialStates);


    // Wrap Controller Synthesis class
    py::class_<ControllerSynthesis>(m, "ControllerSynthesis")
        .def(py::init<
            std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>,
            std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>,
            Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd,
            int, int, double, double, double, double>(),
         py::arg("proxyDynamics"), py::arg("trueDynamics"),
         py::arg("initialState"), py::arg("y"), py::arg("inputDynamics"),
         py::arg("stateDimension"), py::arg("inputDimension"),
         py::arg("t_final"), py::arg("epsilon"), py::arg("k"), py::arg("delta_t"))
        .def("setY", &ControllerSynthesis::setY)
        .def("initializeController", &ControllerSynthesis::initializeController)
        .def("setInitialState", &ControllerSynthesis::setInitialState)
        .def("synthesizeControl", &ControllerSynthesis::synthesizeControl)
        .def("setG0", &ControllerSynthesis::setG0);
}
