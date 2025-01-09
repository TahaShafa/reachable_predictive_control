import sys
sys.path.append("/home/taha-shafa/Projects/RPC/build/pybullet")
import pybullet_module
import pybullet as p
import pybullet_data
import cv2
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from io import BytesIO
from PIL import Image

def create_road_segment(start_pos, size=(10, 1, 0.1), color=[0, 0, 0, 1], orientation='horizontal'):
    """
    Create a road segment starting from `start_pos`.
    `orientation` can be 'horizontal' (along x-axis) or 'vertical' (along y-axis).
    """
    if orientation == 'horizontal':
        # Place the road along the x-axis
        center_pos = [start_pos[0] + size[0] / 2, start_pos[1], start_pos[2]]
        next_start_pos = [start_pos[0] + size[0], start_pos[1], start_pos[2]]
    elif orientation == 'vertical':
        # Place the road along the y-axis
        center_pos = [start_pos[0], start_pos[1] - size[1] / 2, start_pos[2]]
        next_start_pos = [start_pos[0], start_pos[1] - size[1], start_pos[2]]
    else:
        raise ValueError("Invalid orientation. Use 'horizontal' or 'vertical'.")

    # Create the road segment
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size[0] / 2, size[1] / 2, size[2] / 2])
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[size[0] / 2, size[1] / 2, size[2] / 2], rgbaColor=color)
    body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape,
                             baseVisualShapeIndex=visual_shape, basePosition=center_pos)
    return body, next_start_pos

# Path to the text file
file_path = "inputs_velocity.txt"
file_path_fast = "inputs_velocity_recorded_two.txt"
file_path_turn_right = "inputs_theta_positive_recorded_02.txt"
file_path_turn_right_fast = "inputs_theta_positive_recorded_04.txt"
file_path_turn_left = "inputs_theta_negative_recorded_02.txt"
file_path_turn_left_fast = "inputs_theta_negative_recorded_04.txt"

# Read the file
data = np.loadtxt(file_path)
data_fast = np.loadtxt(file_path_fast)
# Somewhere here I got my rights and lefts mixed up
data_turn_left = np.loadtxt(file_path_turn_right)
data_turn_left_fast = np.loadtxt(file_path_turn_right_fast)
data_turn_right = np.loadtxt(file_path_turn_left)
data_turn_right_fast = np.loadtxt(file_path_turn_left_fast)

# Extract columns into inputs 
input_velocity_acceleration = data[:, 0]  # Left column
input_velocity_steering = data[:, 1]  # Right column
input_velocity_acceleration_fast = data_fast[:, 0]
input_velocity_steering_fast = data_fast[:, 1]
input_steering_right_u1 = data_turn_right[:, 0]
input_steering_right_u2 = data_turn_right[:, 1]
input_steering_left_u1 = data_turn_left[:, 0]
input_steering_left_u2 = data_turn_left[:, 1]

# Initialize Matplotlib figure
plt.ion()
fig, ax = plt.subplots(figsize=(2, 2), dpi=100)  # Small figure for overlay
ax.set_title("Velocities")
ax.set_xlabel("Time")
ax.set_xlim(0, 2)  # Adjust x-axis range
ax.set_ylim(0, 4)  # Adjust y-axis range
ax.set_ylabel("Value")
line_velocity, = ax.plot([], [], label="Velocity ($m/s$)", color="red")
line_heading, = ax.plot([], [], label="Steer Angle (radians)", color="blue")
line_grs, = ax.plot([], [], label="Guaranteed Steering", color="black")
# Customize legend
ax.legend(
    loc="upper left",  # Place the legend in the upper left
    fontsize="small",  # Smaller font size
    bbox_to_anchor=(1, 1)  # Move the legend outside the plot
)

plt.tight_layout()

# Initialize a second Matplotlib figure for inputs
fig_inputs, ax_inputs = plt.subplots(figsize=(2, 2), dpi=100)
ax_inputs.set_title("Inputs")
ax_inputs.set_xlabel("Time")
ax_inputs.set_xlim(0, 2)  # Adjust x-axis range
ax_inputs.set_ylim(-2, 2)  # Adjust y-axis range for inputs
ax_inputs.set_ylabel("Value")
line_acceleration, = ax_inputs.plot([], [], label="Acceleration ($m/s^2$)", color="green")
line_steering, = ax_inputs.plot([], [], label="Steering rate (rad/s)", color="orange")
ax_inputs.legend(loc="upper left", fontsize="small", bbox_to_anchor=(1, 1))
plt.tight_layout()

# Data storage for plot
times = []
headings = []
velocities = []
grs_array = []

# Update plot function
def update_plot(t, heading, velocity, grs):
    times.append(t)
    headings.append(heading)
    velocities.append(velocity)
    grs_array.append(grs)

    line_velocity.set_data(times, velocities)
    line_heading.set_data(times, headings)
    line_grs.set_data(times, grs_array)

    ax.set_xlim(max(0, t - 20), t + 1)  # Slide the window as time progresses
    ax.figure.canvas.draw()
    ax.figure.canvas.flush_events()

# Convert Matplotlib plot to image

def plot_to_image():
    buffer = BytesIO()
    fig.savefig(buffer, format="png", bbox_inches="tight")  # Explicitly use `fig` for the state plot
    buffer.seek(0)
    img = Image.open(buffer)
    img = img.convert("RGBA")
    buffer.close()
    return img


# Helper function to overlay plot on a frame

def overlay_plot(frame):
    plot_img = plot_to_image()
    plot_np = np.array(plot_img)
    plot_h, plot_w, _ = plot_np.shape

    # Resize and position the plot in the bottom-right corner
    resized_width = 250  # Desired width
    resized_height = int(resized_width * (plot_h / plot_w))  # Preserve aspect ratio
    plot_np_resized = cv2.resize(plot_np, (resized_width, resized_height))

    # Get video frame dimensions
    frame_h, frame_w, _ = frame.shape

    # Define the position for bottom-right corner
    x_offset = frame_w - resized_width - 10  # 10-pixel padding from the right
    y_offset = frame_h - resized_height - 10  # 10-pixel padding from the bottom

    # Overlay the resized plot on the video frame
    frame[y_offset:y_offset + resized_height, x_offset:x_offset + resized_width] = plot_np_resized[..., :3]

    return frame


# Data storage for input plot
times_inputs = []
accelerations = []
steerings = []

# Update function for input plot
def update_input_plot(t, acceleration, steering):
    times_inputs.append(t)
    accelerations.append(acceleration)
    steerings.append(steering)
    line_acceleration.set_data(times_inputs, accelerations)
    line_steering.set_data(times_inputs, steerings)
    ax_inputs.set_xlim(max(0, t - 20), t + 1)  # Slide the window as time progresses
    ax_inputs.figure.canvas.draw()
    ax_inputs.figure.canvas.flush_events()

# Convert second Matplotlib plot to image

def input_plot_to_image():
    buffer = BytesIO()
    fig_inputs.savefig(buffer, format="png", bbox_inches="tight")  # Explicitly use `fig_inputs` for the input plot
    buffer.seek(0)
    img = Image.open(buffer)
    img = img.convert("RGBA")
    buffer.close()
    return img


# Helper function to overlay the second plot on a frame

def overlay_input_plot(frame):
    plot_img = input_plot_to_image()
    plot_np = np.array(plot_img)
    plot_h, plot_w, _ = plot_np.shape

    # Resize and position the plot in the bottom-left corner
    resized_width = 250  # Desired width
    resized_height = int(resized_width * (plot_h / plot_w))  # Preserve aspect ratio
    plot_np_resized = cv2.resize(plot_np, (resized_width, resized_height))

    # Get video frame dimensions
    frame_h, frame_w, _ = frame.shape

    # Define the position for bottom-left corner
    x_offset = 10  # 10-pixel padding from the left
    y_offset = frame_h - resized_height - 10  # 10-pixel padding from the bottom

    # Overlay the resized plot on the video frame
    frame[y_offset:y_offset + resized_height, x_offset:x_offset + resized_width] = plot_np_resized[..., :3]

    return frame

# Connect to PyBullet GUI
p.connect(p.GUI)

# Add default PyBullet data path
data_path = pybullet_data.getDataPath()
p.setAdditionalSearchPath(data_path)
print(f"pybullet data path: {data_path}")

# Set gravity
p.setGravity(0, 0, -9.8)

# Create the first straight road
straight_road = []
start_pos = [0, 0, 0]
for _ in range(5):  # Create 5 straight road segments
    road_segment, start_pos = create_road_segment(
        start_pos, 
        size=(2.2, 1, 0.1),  # Size is 1
        color=[0.1, 0.1, 0.1, 1], 
        orientation='horizontal'
    )
    straight_road.append(road_segment)

# Create a 90-degree left turn on the road
turn_start_pos = [start_pos[0], start_pos[1] +4.5, start_pos[2]]  # Align the first vertical road segment correctly
for _ in range(5):  # Create 5 vertical road segments
    road_segment, turn_start_pos = create_road_segment(
        turn_start_pos, 
        size=(2, 1, 0.1),  
        color=[0.1, 0.1, 0.1, 1], 
        orientation='vertical'
    )
    straight_road.append(road_segment)

# Create a straight horizontal segment to the right of the left-turn road
right_turn_start_pos = [turn_start_pos[0] - 1.0, turn_start_pos[1] + 5.5, turn_start_pos[2]]  # Correct alignment for horizontal segment
for _ in range(5):  # Create 5 horizontal road segments
    road_segment, right_turn_start_pos = create_road_segment(
        right_turn_start_pos, 
        size=(2.2, 1, 0.1),  
        color=[0.1, 0.1, 0.1, 1], 
        orientation='horizontal'
    )
    straight_road.append(road_segment)


# Load URDF files for simulation
# Define the rotation quaternion for a 180-degree rotation about the z-axis
car_orientation = p.getQuaternionFromEuler([0, 0, 0])
# Load racecar
car_id = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.05], baseOrientation = car_orientation)

# Load the plane URDF
plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

# Change the plane's color to red
red_color = [1, 0, 0, 1]  # RGBA for red
p.changeVisualShape(plane_id, -1, rgbaColor=red_color)

position, orientation = p.getBasePositionAndOrientation(car_id)
euler_angles = p.getEulerFromQuaternion(orientation)
print("Default orientation (quaternion):", orientation)
print("Default orientation (Euler angles):", euler_angles)

# Print joint details
print("Joint Information:")
for joint_index in range(p.getNumJoints(car_id)):
    print(f"Joint {joint_index}: {p.getJointInfo(car_id, joint_index)}")

# Dynamically find joint indices
steering_joints = []
rear_wheel_joints = []
for joint_index in range(p.getNumJoints(car_id)):
    joint_name = p.getJointInfo(car_id, joint_index)[1].decode('utf-8')
    if 'steering' in joint_name:
        steering_joints.append(joint_index)
    if 'rear_wheel' in joint_name:
        rear_wheel_joints.append(joint_index)

print(f"Steering Joints: {steering_joints}")
print(f"Rear Wheel Joints: {rear_wheel_joints}")

# Camera settings
width, height = 640, 480
fov = 60
aspect = width / height
near, far = 0.1, 100

# Initialize OpenCV video writer
video_filename = "car_simulation.mp4"
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))

# Camera parameters
camera_distance_behind = 1  # Distance behind the car
camera_height = 2  # Height of the camera above the car
camera_pitch = -15  # Downward tilt in degrees (negative means downward)

# Convert pitch angle to radians
pitch_radians = np.radians(camera_pitch)

# Helper function to capture and write frames
def capture_frame():
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[
            car_position[0] - camera_distance_behind * np.cos(pitch_radians),
            car_position[1],
            car_position[2] + camera_height * np.sin(-pitch_radians),
        ],
        cameraTargetPosition=car_position,
        cameraUpVector=[0, 0, 1],
    )
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    frame_data = p.getCameraImage(width, height, view_matrix, projection_matrix)
    frame = np.reshape(frame_data[2], (height, width, 4))  # RGBA image
    frame_bgr = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)

    # Update the state plot
    update_plot(time_elapsed, state[2], state[3], grs)

    # Update the input plot
    update_input_plot(time_elapsed, acceleration, steering)

    # Overlay the state plot on the frame
    frame_with_overlay = overlay_plot(frame_bgr)

    # Overlay the input plot on the frame
    frame_with_overlay = overlay_input_plot(frame_with_overlay)

    # Write the frame to the video
    video_writer.write(frame_with_overlay)


# Car parameters
L = 1.0  # Wheelbase
Lf = 1.0
Lg = 1.0
dt = 0.26 / 120  # Simulation timestep (standard pybullet timestep)

# Initial state [x, y, theta, v]
state = np.array([0, 0, 0, 1.0])  # x-position, y-position, yaw angle, velocity
carInitialState = np.array([0.0, 0.0])

# Load the car dynamics
car_dynamics = pybullet_module.CarDynamics(carInitialState, Lf, Lg, L)
car_proxyDynamics = car_dynamics.getProxyDynamics()
car_trueDynamics = car_dynamics.getTrueDynamics()
G0_car = car_dynamics.getG0()

# Load the RPC settings
stateDimension = 2
inputDimension = 2
k = 5.0
epsilon = 0.1
t_final = 0.2
delta_t = 0.0015

car_controller = pybullet_module.ControllerSynthesis(car_proxyDynamics, car_trueDynamics, carInitialState, np.array([0.0, 0.2]), G0_car, stateDimension, inputDimension, t_final, epsilon, k, delta_t)

# Controller inputs
target_velocity = 2  # Desired velocity (m/s)
target_steering_angle = 0.0  # Desired steering angle (rad)

# Dynamics function (Kinematic Bicycle Model)
def kinematic_bicycle_model(state, delta, a, dt):
    x, y, theta, v = state
    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = v / L * np.tan(delta)
    v_dot = a
    # Update state
    x += x_dot * dt
    y += y_dot * dt
    theta += theta_dot * dt
    v += v_dot * dt

    # Apply state to PyBullet for visualization
    car_position = [x, y, 0.05]
    car_orientation = p.getQuaternionFromEuler([0, 0, theta])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)

    # Step simulation
    p.stepSimulation()
    return np.array([x, y, theta, v])

# Camera parameters
camera_distance = 3
camera_yaw = -45  # Horizontal angle around the car
camera_pitch = -30  # Vertical angle

# def compute_control_inputs(y_car, view_plot = False, save_plot = False):
#     # current_input_length = total_input_length
#     # start_time = time.time()
#     y_reached, inputs = car_controller.synthesizeControl(view_plot, save_plot)
#     # elapsed_time = start_time - time.time()
#     # total_input_length = len(inputs[0])
#     # current_input_length = total_input_length - current_input_length
#     # print(f"Number of Inputs from input_producer", current_input_length)
#
#     # Put the inputs into the queue
#     # input_queue.put(inputs)
#     # Check the size of the queue
#     # print(f"Number of elements in the input_queue: {input_queue.qsize()}")
#
#     # Reinitialize the initial condition to final state reached using the controller
#     car_controller.setInitialState(y_reached)
#     car_dynamics.setInitialStates(y_reached)
#     car_dynamics.setG0(y_reached)
#
#     # Get the new dynamics and enter the corresponding initial conditions
#     inputDynamics = car_dynamics.getG0()
#
#     # Set new final desired position for your controller to navigate to
#     y_target = y_reached + y_car
#     car_controller.setY(y_target)
#     car_controller.setG0(inputDynamics)
#     car_controller.initializeController()
#
#     return y_reached, inputs

# Simulation loop with plot overlay
time_elapsed = 0  # Track simulation time
step = 0
keep_track = 0
input_length = len(input_velocity_steering_fast)
grs = 0.2
last_recorded_velocity = 1.0
case = 1
while True:
    # Get the current input values
    acceleration = input_velocity_acceleration_fast[step]
    steering = input_velocity_steering_fast[step]

    # Update state using the kinematic bicycle model
    state = kinematic_bicycle_model(state, steering, acceleration, dt)
    
    if (state[3] - last_recorded_velocity >= 0.2):
        grs = state[3] * 0.2 + state[2]
        last_recorded_velocity = state[3]
    # if (keep_track % 300 == 0):
    #     if case == 0:
    #         grs = grs + state[3] * 0.2
    #     if case == 1:
    #         grs = grs + state[3] * 0.17
    #     if case == 2:
    #         grs = grs + state[3] * 0.13
    #     if case == 3:
    #         grs = grs + state[3] * 0.08
    #     if case == 4:
    #         grs = grs + state[3] * 0.05 
    #     if case == 5:
    #         grs = grs + state[3] * 0.025
    #     if case == 6:
    #         grs = grs + state[3] * 0.015
    #     if case == 7:
    #         grs = grs + state[3] * 0.01
    #     case += 1


    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    # Step simulation
    p.stepSimulation()

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    print("x-position: ", state[0])
    print("y-position: ", state[1])
    print("current velocity: ", state[3])
    print("current heading (degrees): ", state[2] * 180/np.pi, "\n")
    print("grs: ", grs, "\n")

    # Update the camera to follow the car
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=car_position,
    )

    # Compute view and projection matrices for the camera
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=car_position,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100
    )

    # Step simulation
    p.stepSimulation()
    capture_frame()

    time.sleep(1/120)
    time_elapsed += dt
    step += 1
    keep_track += 1
    # print(f"Keep_track: ", keep_track)

    if (step == input_length - 1):
        step = 0

    if (state[0] > 10):
        break

check_run = 1
for i in range(3):
    for step in range(len(input_steering_left_u1)):
        # Get the current input values
        acceleration = input_steering_left_u1[step]
        steering = input_steering_left_u2[step]

        # Update state using the kinematic bicycle model
        state = kinematic_bicycle_model(state, steering, acceleration, dt)

        if (check_run % 70 == 0.0):
            grs = state[3] * 0.2 + state[2]
            last_recorded_velocity = time_elapsed


        # Apply state to PyBullet for visualization
        car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
        car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
        p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)

        # Step simulation
        p.stepSimulation()

        # Apply state to PyBullet for visualization
        car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
        car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
        p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
        print("x-position: ", state[0])
        print("y-position: ", state[1])
        print("current velocity: ", state[3])
        print("current heading (degrees): ", state[2] * 180/np.pi, "\n")
        print("grs: ", grs, "\n")

        # Update the camera to follow the car
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=car_position,
        )

        # Compute view and projection matrices for the camera
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=car_position,
            distance=camera_distance,
            yaw=camera_yaw,
            pitch=camera_pitch,
            roll=0,
            upAxisIndex=2
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100
        )

        # Step simulation
        p.stepSimulation()
        capture_frame()

        time.sleep(1/120)
        time_elapsed += dt
        # print(f"time_elapsed: ", time_elapsed)
        check_run += 1.0
        print(f"check_run: ", check_run)

        if (state[2] * 180/np.pi > 90.0):
            break;

grs = 1.9541464182726618
flag = False
# Go straight after taking a left-hand turn
input_length = len(input_velocity_acceleration_fast)
step = 0
while True:
    if (state[3] < 3.5):
        # Get the current input values
        acceleration = input_velocity_acceleration_fast[step]
        steering = input_velocity_steering_fast[step]
    else:
        flag = True
        acceleration = -0.5
        steering = 0.0

    # Update state using the kinematic bicycle model
    state = kinematic_bicycle_model(state, steering, acceleration, dt)
    if flag == False:
        grs = 1.9541464182726618
    else:
        grs = state[2] - (1.9541464182726618 - state[2])


    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    # Step simulation
    p.stepSimulation()

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    print("x-position: ", state[0])
    print("y-position: ", state[1])
    print("current velocity: ", state[3])
    print("current heading (degrees): ", state[2] * 180/np.pi, "\n")

    # Update the camera to follow the car
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=car_position,
    )

    # Compute view and projection matrices for the camera
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=car_position,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100
    )

    # Step simulation
    p.stepSimulation()
    capture_frame()

    time.sleep(1/120)
    time_elapsed += dt
    step += 1

    if (step == input_length - 1):
        step = 0

    if (state[1] > 3.6):
        break

check_run = 1
for i in range(3):
    for step in range(len(input_steering_right_u1)):
        # Get the current input values
        acceleration = input_steering_right_u1[step]    
        steering = input_steering_right_u2[step]

        # Update state using the kinematic bicycle model
        state = kinematic_bicycle_model(state, steering, acceleration, dt)

        if (check_run % 70 == 0.0):
            if (state[2] - state[3] * 0.2 >= 0.0):
                grs = state[2] - state[3] * 0.2
            else:
                grs = 0.0


        # Apply state to PyBullet for visualization
        car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
        car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
        p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)

        # Step simulation
        p.stepSimulation()

        # Apply state to PyBullet for visualization
        car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
        car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
        p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
        print("x-position: ", state[0])
        print("y-position: ", state[1])
        print("current velocity: ", state[3])
        print("current heading (degrees): ", state[2] * 180/np.pi, "\n")
        # Update the camera to follow the car
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=car_position,
        )

        # Compute view and projection matrices for the camera
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=car_position,
            distance=camera_distance,
            yaw=camera_yaw,
            pitch=camera_pitch,
            roll=0,
            upAxisIndex=2
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100
        )

        # Step simulation
        p.stepSimulation()
        capture_frame()

        time.sleep(1/150)
        time_elapsed += dt
        check_run += 1

        if (state[2] < 0.0):
            break;

# Go straight after taking a left-hand turn
input_length = len(input_velocity_acceleration_fast)
step = 0
while True:
    if (state[3] < 3.5):
        # Get the current input values
        acceleration = input_velocity_acceleration_fast[step]
        steering = input_velocity_steering_fast[step]
    else:
        if (step % 3 == 0):
            acceleration = -0.1
        elif (step % 3 - 1 == 0):
            acceleration = -0.2
        else:
            acceleration = -0.3
        steering = input_velocity_steering_fast[step]

    # Update state using the kinematic bicycle model
    state = kinematic_bicycle_model(state, steering, acceleration, dt)
    grs = state[3] * 0.2 + state[2]

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    # Step simulation
    p.stepSimulation()

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)
    print("x-position: ", state[0])
    print("y-position: ", state[1])
    print("current velocity: ", state[3])
    print("current heading (degrees): ", state[2] * 180/np.pi, "\n")

    # Update the camera to follow the car
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=car_position,
    )

    # Compute view and projection matrices for the camera
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=car_position,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100
    )

    # Step simulation
    p.stepSimulation()
    capture_frame()

    time.sleep(1/120)
    time_elapsed += dt
    step += 1

    if (step == input_length - 1):
        step = 0

    if (state[0] > 20):
        break

# Wait for user input to close the simulation
input("Press Enter to close the simulation...")
# Release the video writer and close PyBullet
video_writer.release()
p.disconnect()

print(f"Video saved as {video_filename}")


