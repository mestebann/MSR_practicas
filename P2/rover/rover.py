import pybullet as p
import pybullet_data
import time

# CONFIGURACIÓN
physicsClient = p.connect(p.GUI)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) 

planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 10]
euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("urdf/rover.urdf", startPos, startOrientation)

num_joints = p.getNumJoints(robotId)
print("Número de joints:" + str(num_joints))

joints = [0, 1, 2, 3]
for j in range(num_joints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))
    print("Link - %s" % (p.getJointInfo(robotId, j)[12]))

input("Press enter to start motion...")

try:
    while True:
        p.setJointMotorControlArray(robotId, joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 4)
        
except KeyboardInterrupt:
    pass

p.disconnect()