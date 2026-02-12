import pybullet as p
import pybullet_data
import time

rampa_urdf = "rampa.urdf"
husky_urdf  = "husky/husky.urdf"
plataforma_urdf = "plataforma.urdf"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) 

planeId = p.loadURDF("plane.urdf")

husky_startPosition = [0,0,1]
husky_euler_angles = [0,0,-1.57]
husky_startOrientation = p.getQuaternionFromEuler(husky_euler_angles)
rampa = p.loadURDF(husky_urdf, husky_startPosition, husky_startOrientation)

rampa_startPosition = [0,10,1]
rampa_euler_angles = [0,0,-3.15]

rampa_startOrientation = p.getQuaternionFromEuler(rampa_euler_angles)
rampa = p.loadURDF(rampa_urdf, rampa_startPosition, rampa_startOrientation)

plataforma_startPosition = [0,20,1]
plataforma_euler_angles = [0,0,-3.15]
plataforma_startOrientation = p.getQuaternionFromEuler(plataforma_euler_angles)
plataforma = p.loadURDF(plataforma_urdf, plataforma_startPosition, plataforma_startOrientation)


try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
    pass

p.disconnect()
