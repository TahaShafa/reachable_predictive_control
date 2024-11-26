import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf")
cube = p.loadURDF("cube.urdf", [0, 0, 0.5])

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)

p.disconnect()
