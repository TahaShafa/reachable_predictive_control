import sys
sys.path.append("/home/taha-shafa/Projects/RPC/build/pybullet")
import pybullet_module
import pybullet as p
import pybullet_data
import time
import numpy as np
import threading
from queue import Queue

# Initialize PyBullet
def setup_pybullet():
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load a simple car model (use a box for simplicity)
    car_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.05])
    return car_id

# Update car position in PyBullet and make the camera follow the car
def update_car_in_pybullet(car_id, state):
    """
    Updates the car's position and orientation in PyBullet and adjusts the camera to follow the car.

    Args:
        car_id: ID of the car in PyBullet.
        state: Current state of the car [x, y, theta, v].
    """
    # Extract state variables
    x, y, theta, _ = state

    # Update car's position and orientation
    position = [x, y, 0.05]  # Keep z fixed for a 2D simulation
    orientation = p.getQuaternionFromEuler([0, 0, theta])
    p.resetBasePositionAndOrientation(car_id, position, orientation)

    # Camera parameters
    camera_distance = 2.0  # Distance from the car
    camera_yaw = -90.0       # Horizontal rotation
    camera_pitch = -30.0   # Vertical rotation

    # Set the camera to follow the car
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=position,
    )

    # Compute view and projection matrices for the camera (optional for advanced rendering)
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=position,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100
    )

    # You can use the matrices (view_matrix and projection_matrix) if needed for rendering
    return view_matrix, projection_matrix

def kinematic_bicycle_model(state, get_inputs, car_id, state_queue, L=1.0):
    while True:
        # Get the full array of inputs from the producer
        inputs = get_inputs()
        delta_array, a_array = inputs[1], inputs[0]
        num_inputs = len(delta_array)
        real_time_dt = 0.25 / 120
        # print(f"The entered inputs: ", inputs[1],inputs[0])
        # print(f"Number of inputs", num_inputs)

        for i in range(num_inputs):
            delta = delta_array[i]
            a = a_array[i]

            # Unpack state
            x, y, theta, v = state

            # Ensure scalars
            x, y, theta, v = float(x), float(y), float(theta), float(v)

            # Calculate derivatives
            x_dot = v * np.cos(theta)
            y_dot = v * np.sin(theta)
            theta_dot = v / L * np.tan(delta)
            v_dot = a

            # Update state
            x += x_dot * real_time_dt
            y += y_dot * real_time_dt
            theta += theta_dot * real_time_dt
            v += v_dot * real_time_dt

            # Update state array
            state[0], state[1], state[2], state[3] = x, y, theta, v

            # Push the updated state into the state queue
            state_queue.put(state.copy())

            # Update car position in PyBullet
            update_car_in_pybullet(car_id, state)

            # Simulate real-time behavior
            time.sleep(real_time_dt)


# def kinematic_bicycle_model(state, elapsed_time, get_inputs, car_id, L=1.0):
#     """
#     Updates the state of the car in PyBullet using the kinematic bicycle model
#     and applies the control inputs in real-time.
#     """
#     # Wait for inputs to become available
#     while True:
#         inputs = get_inputs()
#         if inputs is not None:
#             break
#         print("Waiting for inputs...")
#         time.sleep(0.01)  # Wait briefly before trying again
#
#     # Compute real-time step size
#     delta_array, a_array = inputs[1], inputs[0]
#     num_inputs = len(delta_array)
#     dt = elapsed_time / num_inputs 
#     start_time = time.time()
#     for i in range(num_inputs):
#
#         if inputs is not None:
#             start_time = time.time()
#             delta = delta_array[i]
#             a = a_array[i]
#
#             # Unpack state
#             x, y, theta, v = state
#
#             # Ensure scalars
#             x, y, theta, v = float(x), float(y), float(theta), float(v)
#
#             # Calculate derivatives
#             x_dot = v * np.cos(theta)
#             y_dot = v * np.sin(theta)
#             theta_dot = v / L * np.tan(delta)
#             v_dot = a
#
#             # Update state
#             x += x_dot * dt
#             y += y_dot * dt
#             theta += theta_dot * dt
#             v += v_dot * dt
#
#             # Update state array
#             state[0], state[1], state[2], state[3] = x, y, theta, v
#
#             # Debug output
#             print(f"Applied input {i + 1}/{num_inputs}: delta={delta}, a={a}")
#             print(f"Updated state: x={x:.2f}, y={y:.2f}, theta={theta:.2f}, v={v:.2f}")
#
#             # Update car position in PyBullet (if applicable)
#             update_car_in_pybullet(car_id, state)
#
#             # Simulate real-time behavior
#             time.sleep(dt)  # Wait for the next control input
#         else:
#             print("No inputs available. Waiting...")
#             time.sleep(0.01)  # No inputs yet, wait briefly
#
#     elapsed_time = time.time() - start_time
#     print("Total Elapsed Time In Python", elapsed_time, "\n")

import queue
from queue import Queue
from collections import deque
state_dequeue = deque(maxlen=100)

# Input producer thread
def input_producer(input_queue, state_queue, car_controller, car_dynamics, y_car, view_plot, save_plot):
    total_input_length = 0.0
    for i in range(5):
    # while True:
        try:
            # Get the latest state from state queue
            current_state = state_queue.get_nowait()
            print(f"Current state in the producer: {current_state}")
        except queue.Empty:
            # No state available yet
            current_state = None

        # if current_state is None:
        #     print(f"The current state is none\n")
        #
        # if current_state is not None:
        #     print(f"The current velocity: ", current_state[3], "\n")
        #
        #
        # if current_state is not None and current_state[3] > 0.05:
        #     # Check if the 4th state element exceeds the threshold
        #     inputs = np.zeros((2,1))
        #     print(f"The current velocity: ", current_state[3], "\n")
        # else:
            # Default program logic (runs if state is None or within the threshold)
        current_input_length = total_input_length
        start_time = time.time()
        y_initial, inputs = car_controller.synthesizeControl(view_plot, save_plot)
        elapsed_time = start_time - time.time()
        total_input_length = len(inputs[0])
        current_input_length = total_input_length - current_input_length
        print(f"Number of Inputs from input_producer", current_input_length)
        
        # Put the inputs into the queue
        input_queue.put(inputs)
        # Check the size of the queue
        print(f"Number of elements in the input_queue: {input_queue.qsize()}")
        
        # Reinitialize the initial condition to final state reached using the controller
        car_controller.setInitialState(y_initial)
        car_dynamics.setInitialStates(y_initial)
        car_dynamics.setG0(y_initial)
        
        # Get the new dynamics and enter the corresponding initial conditions
        inputDynamics = car_dynamics.getG0()
            
        if i < 9:
            # Set new final desired position for your controller to navigate to
            y_final = y_initial + y_car
            car_controller.setY(y_final)
            car_controller.setG0(inputDynamics)
            car_controller.initializeController()

        print(f"This loop is still running")
    
    if not input_queue.full():
        while not state_queue.empty():
            current_state = state_queue.get_nowait()
    time.sleep(1/120)
    
    print(f"The current velocity: ", current_state[3], "\n")
    print(f"The current angle: ", current_state[2], "\n")
    print(f"The current x-position: ", current_state[0], "\n")

    
    
    # for j in range(100):  
    #
    # # for j in range(100):
    # #     try:
    # #         # Always get the most recent state
    # #         while not state_queue.empty():
    # #             current_state = state_queue.get_nowait()
    # #
    # #         print(f"The current velocity: {current_state[3]}")
    # #
    # #         # Push zero inputs into the queue
    # #         inputs = np.zeros((2, 120))
    # #         input_queue.put(inputs)
    # #
    # #         # Print queue sizes for debugging
    # #         print(f"Number of states in the queue: {state_queue.qsize()}")
    # #         print(f"Number of inputs in the input_queue: {input_queue.qsize()}")
    #
    #     if not input_queue.full():
    #         while not state_queue.empty():
    #             current_state = state_queue.get_nowait()
    #             # print(f"The current state: ", current_state, "\n")
    #             #inputs = np.zeros((2,120))
    #             #input_queue.put(inputs)
    #
    #         #inputs = np.zeros((2,120))
    #         time.sleep(1/120)
    #         print(f"The current velocity: ", current_state[3], "\n")
    #         print(f"The current angle: ", current_state[2], "\n")
    #         print(f"The current x-position: ", current_state[0], "\n")
    #         theta = current_state[2]
    #         if abs(current_state[2]) > 0.02:
    #             if initial_adjustment:
    #                 for i in range(2):
    #                     if current_state[2] > 0.0:
    #                         y_car = np.array([-0.05, 0])
    #                     else:
    #                         y_car = np.array([0.05, 0])
    #                     y_final = y_initial + y_car
    #                     print(f"y_final for manipulating theta: ", y_final)
    #                     car_controller.setInitialState(y_initial)
    #                     car_dynamics.setInitialStates(y_initial)
    #                     car_dynamics.setG0(y_initial)
    #
    #                     # Get the new dynamics and enter the corresponding initial conditions
    #                     inputDynamics = car_dynamics.getG0()
    #                     car_controller.setY(y_final)
    #                     car_controller.setG0(inputDynamics)
    #                     car_controller.initializeController()
    #                     y_initial, inputs = car_controller.synthesizeControl(view_plot, save_plot)
    #                     print(f"Current Input Length for heading adjustment: ", len(inputs[0]))
    #                     input_queue.put(inputs)
    #                 initial_adjustment = False
    #             else:
    #                 inputs = np.zeros((2,120))
    #                 input_queue.put(inputs)
    #                 print(f"We ended up here in the loop forever\n")
    #         else:
    #             inputs = np.zeros((2,220))
    #             input_queue.put(inputs)
    #             print(f"We ended up here\n")
    #
    #     if input_queue.full():
    #         print(f"The input queue got full I guess\n")
    #
    #         # print(f"Number of states in the queue: {state_queue.qsize()}")





# Input consumer
def get_inputs(queue):
    if not queue.empty():
        inputs = queue.get()
        print(f"Retrieved inputs from queue: shape={inputs.shape}")
        return inputs
    return None  # No inputs available

from threading import Thread

# Main simulation setup
def main():
    # PyBullet setup
    car_id = setup_pybullet()

    # Kinematic model initial state
    initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, theta, v]
    state = np.copy(initial_state)

    # Control input queue
    input_queue = Queue()
    state_queue = Queue()

    # Define the car dynamics
    length = 1.0
    Lf = 1.0
    Lg = 1.0
    carInitialState = np.array([0.0, 0.0])
    car_dynamics = pybullet_module.CarDynamics(carInitialState, Lf, Lg, length)
    car_proxyDynamics = car_dynamics.getProxyDynamics() #Proxy dynamics for RPC
    car_trueDynamics = car_dynamics.getTrueDynamics() #True dynamics for simulation
    G0_car = car_dynamics.getG0()

    # Car controller arguments
    stateDimension = 2
    inputDimension = 2
    k = 5.0 
    epsilon = 0.1
    t_final = 0.2
    delta_t = 0.0015

    # Define the final desired reachable state
    y_car = np.array([0.0, 1.0])
    y_car = y_car / np.linalg.norm(y_car) # Normalize the vector
    y_car = y_car * t_final # Scale by t_final

    car_controller = pybullet_module.ControllerSynthesis(car_proxyDynamics, car_trueDynamics, carInitialState, y_car, G0_car, stateDimension, inputDimension, t_final, epsilon, k, delta_t)

    # Signal if we want to plot or save the data
    view_plot = False
    save_plot = False

    # Start input producer thread
    producer_thread = Thread(
        target=input_producer,
        args=(input_queue, state_queue, car_controller, car_dynamics, y_car, view_plot, save_plot),
        daemon=True
    )
    producer_thread.start()


    # Start the kinematic model simulation
    try:
        kinematic_bicycle_model(state, lambda: input_queue.get(), car_id, state_queue)    
    except KeyboardInterrupt:
        p.disconnect()

# Run the main simulation
if __name__ == "__main__":
    main()
