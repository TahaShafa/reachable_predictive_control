import sys
sys.path.append("/home/taha-shafa/Projects/RPC/build/pybullet")
import pybullet_module
import numpy as np

# Example for CarDynamics
initial_states = np.array([0.0, 1.0, 2.0])
car = pybullet_module.CarDynamics(initial_states, 1.0, 2.0, 3.0)
#print(car.getPosition())

# Example for ControllerSynthesis
def proxy_dynamics(t, x, u):
    return x + u

def true_dynamics(t, x, u):
    return x * u

initial_state = np.array([0.0, 1.0])
y = np.array([2.0, 3.0])
input_dynamics = np.eye(2)
controller = pybullet_module.ControllerSynthesis(
    proxy_dynamics, true_dynamics,
    initial_state, y, input_dynamics,
    2, 2, 10.0, 0.01, 1.0, 0.1
)
print(controller)
