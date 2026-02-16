import pybullet as p
import pybullet_data
import time

# FICHEROS URDF
ramp_urdf = "rampa.urdf"
husky_urdf  = "husky/husky.urdf"
platform_urdf = "plataforma.urdf"
barrier_urdf = "barrera.urdf"

# CONFIGURACIÓN
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) 

planeId = p.loadURDF("plane.urdf")

# CONFIGURACION HUSKY
husky_startPosition = [0,0,1]
husky_euler_angles = [0,0,1.57]
husky_startOrientation = p.getQuaternionFromEuler(husky_euler_angles)
husky = p.loadURDF(husky_urdf, husky_startPosition, husky_startOrientation)

# CONFIGURACION RAMPA
ramp_startPosition = [0,10,1]
ramp_euler_angles = [0,0,-3.15]
ramp_startOrientation = p.getQuaternionFromEuler(ramp_euler_angles)
ramp = p.loadURDF(ramp_urdf, ramp_startPosition, ramp_startOrientation)

#CONFIGURACION BARRERA
barrier_startPosition = [-1.5,17,0.5]
barrier_euler_angles = [0,0,-1.5708]
barrier_startOrientation = p.getQuaternionFromEuler(barrier_euler_angles)
barrier = p.loadURDF(barrier_urdf, barrier_startPosition, barrier_startOrientation, useFixedBase = True)

# CONFIGURACION PLATAFORMA
platform_startPosition = [0,20,1]
platform_euler_angles = [0,0,-3.15]
platform_startOrientation = p.getQuaternionFromEuler(platform_euler_angles)
platform = p.loadURDF(platform_urdf, platform_startPosition, platform_startOrientation)

# JOINTS HUSKY
husky_numJoints = p.getNumJoints(husky)
husky_jointIndices = list(range(husky_numJoints))

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        
        # MOTORES HUSKY
        p.setJointMotorControlArray(husky, husky_jointIndices, p.VELOCITY_CONTROL, targetVelocities = [10]*husky_numJoints)
        
except KeyboardInterrupt:
    pass

p.disconnect()
