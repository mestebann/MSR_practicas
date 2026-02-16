import pybullet as p
import pybullet_data
import time
import csv

# FICHEROS URDF
ramp_urdf = "rampa.urdf"
husky_urdf  = "husky/husky.urdf"
platform_urdf = "plataforma.urdf"
barrier_urdf = "barrera.urdf"

# CONFIGURACIÓN
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)

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

# VARIABLES DE REGISTRO
datos = []
y_anterior = None
inicio_tiempo = time.time()

# BUCLE PRINCIPAL
while True:

    # MOTORES HUSKY
    p.setJointMotorControlArray(husky, husky_jointIndices, p.VELOCITY_CONTROL, targetVelocities = [10]*husky_numJoints)

    # Obtener posición y velocidad base
    pos, orn = p.getBasePositionAndOrientation(husky)
    vel_lin, vel_ang = p.getBaseVelocity(husky)

    y_actual = pos[1]
    velocidad_y = vel_lin[1]

    # Inicializar posición en y
    if y_anterior is None:
        y_anterior = y_actual

    # Guardar datos cada 0.01 metros
    if abs(y_actual - y_anterior) >= 0.01:

        tiempo_actual = time.time() - inicio_tiempo

        # Velocidad ruedas y fuerza
        velocidades_ruedas = []
        fuerzas_ruedas = []

        for j in husky_jointIndices:
            joint_state = p.getJointState(husky, j)
            velocidades_ruedas.append(joint_state[1])
            fuerzas_ruedas.append(joint_state[3])

        datos.append([tiempo_actual, y_actual, velocidad_y, sum(velocidades_ruedas)/len(velocidades_ruedas), sum(fuerzas_ruedas)/len(fuerzas_ruedas)])

        y_anterior = y_actual

    if y_actual >= 20:
        break

# GUARDAR CSV
with open("resultados.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["tiempo", "posicion_y", "velocidad_y", "velocidad_media_ruedas", "fuerza_media_ruedas"])
    writer.writerows(datos)

p.disconnect()

print("Datos guardados en resultados.csv")
