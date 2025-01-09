import sys
sys.path.append("/home/taha-shafa/Projects/RPC/build/pybullet")
import pybullet_module
import numpy as np
import time

stateDimension = 2
inputDimension = 2
k = 5.0 
epsilon = 0.1
t_final = 0.2
delta_t = 0.0015

view_plot = False # Generate the plot
save_plot = False # Save the plot

# Create an instance of the car dynamics class
length = 1.0
Lf = 1.0
Lg = 1.0
carInitialState = np.array([0.0, 0.0])
car_dynamics = pybullet_module.CarDynamics(carInitialState, Lf, Lg, length)

t_final = 0.2
y_car = np.array([0.0, 1.0])

# Normalize the vector
y_car = y_car / np.linalg.norm(y_car)

# Scale by t_final
y_car = y_car * t_final

# Initialize controller for car
car_proxyDynamics = car_dynamics.getProxyDynamics()
car_trueDyanmics = car_dynamics.getTrueDynamics()
G0_car = car_dynamics.getG0()

car_controller = pybullet_module.ControllerSynthesis(car_proxyDynamics, car_trueDyanmics, carInitialState, y_car, G0_car, stateDimension, inputDimension, t_final, epsilon, k, delta_t)

#[y_initial, inputs] = car_controller.synthesizeControl(view_plot, save_plot)

#print(inputs)

number_trajectories = 8;
for i in range(number_trajectories):
    start_time = time.time()
    [y_initial, inputs] = car_controller.synthesizeControl(view_plot, save_plot)
    elapsed_time = time.time() - start_time
    print(f"Controller Synthesis Time: {elapsed_time:.6f} seconds")

    # reinitialize the initial condition to the final state reached using the controller
    car_controller.setInitialState(y_initial)
    car_dynamics.setInitialStates(y_initial)
    car_dynamics.setG0(y_initial)
    
    # Get the new dynamics and enter the corresponding initial conditions
    inputDynamics = car_dynamics.getG0()

    # Set the new final desired position for your controller
    y_final = y_initial + y_car
    car_controller.setY(y_final)
    car_controller.setG0(inputDynamics)
    car_controller.initializeController()
    print(f"Controller Synthesis Total Time: {elapsed_time:.6f} seconds")
    print(f"Length of inputs as the simulation continues: ", len(inputs[0]), "\n")

