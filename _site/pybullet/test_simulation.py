import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet GUI
p.connect(p.GUI)

# Add default PyBullet data path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.8)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Load racecar
car_id = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.05])

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

# Car parameters
L = 1  # Wheelbase
dt = 1 / 120  # Simulation timestep (standard pybullet timestep)

# Initial state [x, y, theta, v]
state = np.array([0, 0, 0, 0])  # x-position, y-position, yaw angle, velocity

# Controller inputs
target_velocity = 5  # Desired velocity (m/s)
target_steering_angle = 0.2  # Desired steering angle (rad)

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
    return np.array([x, y, theta, v])

# Camera parameters
camera_distance = 3
camera_yaw = -45  # Horizontal angle around the car
camera_pitch = -30  # Vertical angle

# Simulation loop
for _ in range(1000):
    # # Set steering
    # p.setJointMotorControl2(car_id, steering_joints[0], p.POSITION_CONTROL, targetPosition=0.2) # front-left wheel
    # p.setJointMotorControl2(car_id, steering_joints[1], p.POSITION_CONTROL, targetPosition=0.2) # front-right wheel
    #
    # # Set wheel velocity
    # p.setJointMotorControl2(car_id, rear_wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=10) # rear-left wheel
    # p.setJointMotorControl2(car_id, rear_wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=10) # rear-right wheel

    # Define control inputs (you can replace these with a feedback controller)
    current_velocity = state[3]
    acceleration = 1.0 if current_velocity < target_velocity else -1.0  # Simple throttle control
    steering_angle = target_steering_angle

    # Update state using dynamics
    state = kinematic_bicycle_model(state, steering_angle, acceleration, dt)

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.2]
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)

    # Step simulation
    p.stepSimulation()

    # Apply state to PyBullet for visualization
    car_position = [state[0], state[1], 0.05]  # Adjust z to ground level
    car_orientation = p.getQuaternionFromEuler([0, 0, state[2]])
    p.resetBasePositionAndOrientation(car_id, car_position, car_orientation)

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


    # # Capture an image from the car's perspective
    # img = p.getCameraImage(640, 480, view_matrix, projection_matrix)
    #
    # # Capture an image from the car's perspective
    # img = p.getCameraImage(640, 480, view_matrix, projection_matrix)

    time.sleep(1/120)

# Wait for user input to close the simulation
input("Press Enter to close the simulation...")
p.disconnect()


