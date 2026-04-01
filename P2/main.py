import pybullet as p
import pybullet_data
import time
import numpy as np

# ================ CONFIGURACIÓN ========================================

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")

POS_ROVER   = [0, 0, 1.6]
POS_CUBO    = [0, 4, 0.25]
orientacion = p.getQuaternionFromEuler([0, 0, 0])

rover = p.loadURDF("./rover/urdf/rover.urdf", POS_ROVER, orientacion)
cubo  = p.loadURDF("./cubo/urdf/cubo.urdf",  POS_CUBO,  orientacion)

n = p.getNumJoints(rover)
print(f"Número de joints: {n}")
for j in range(n):
    info = p.getJointInfo(rover, j)
    print(f"  {info[0]} - {info[1].decode()}")


# ============ ÍNDICES DE JOINTS ========================================

RUEDAS      = [8, 9, 10, 11]
GRIPPERS    = [5, 6]
BRAZO       = [0, 1, 2, 3, 4]
IK_JOINTS   = [0, 1, 2, 3, 4, 5, 6]
NUM_IK      = len(IK_JOINTS)
END_EFFECTOR = 4   # end-effector: Brazo5_link


# ============ PARÁMETROS DE CONTROL ====================================
 
AVANCE          = [5, 5, 5, 5]
PARADO          = [0, 0, 0, 0]
DIST_FRENADO    = 3.25   # metros antes del cubo donde frena
STEP_SIM        = 0.005  # segundos por paso de simulación
MUESTREO_CSV    = 0.01   # segundos entre muestras de G-parcial
MAX_MUESTRAS    = 3000

# Orientaciones para la IK
ori_pick    = p.getQuaternionFromEuler([0, 0, 0])
ori_place   = p.getQuaternionFromEuler([0, 0, 0])

# Waypoints del manipulador
wp_pick     = [POS_CUBO[0],       POS_CUBO[1],        0.3  ]
wp_deposit  = [POS_CUBO[0] + 0.5, POS_CUBO[1] - 4.5,  2.75 ]

# ================ VARIABLES DE SIMULACIÓN ================================

secuencia       = 1
velocidad       = AVANCE[:]
ik_encendida    = False
dedo            = 0       # 0 = abierto, 0.4 = cerrado
objetivo_ik     = wp_pick
ori_ik          = ori_pick
ganancia_vel    = 1

t_agarre        = 0
t_deposito      = 0
t_fin           = 0

registro        = np.zeros((MAX_MUESTRAS, 3))
num_reg         = 0
t0              = time.time()
t_ultimo_reg    = 0

# ==== BUCLE DE SIMULACIÓN ============================================

try:
    while True:
        p.stepSimulation()
        time.sleep(STEP_SIM)

        pos_rover, _ = p.getBasePositionAndOrientation(rover)
        ahora = time.time()

        # == LÓGICA DE SECUENCIA =======================================

        if secuencia == 1:
            # Conducir hacia el cubo hasta la distancia de frenado
            if (POS_CUBO[1] - pos_rover[1]) <= DIST_FRENADO:
                secuencia       = 2
                velocidad       = PARADO[:]
                ik_encendida    = True
                objetivo_ik     = wp_pick
                ori_ik          = ori_pick
                ganancia_vel    = 1

        elif secuencia == 2:
            # Bajar el brazo hasta la altura del cubo
            z_end_effector = p.getLinkState(rover, END_EFFECTOR)[0][2]
            if z_end_effector < 1.0:
                secuencia   = 3
                t_agarre    = ahora
                dedo        = 0.4   # cerrar gripper

        elif secuencia == 3:
            # Mantener gripper cerrado 1 segundo
            if (ahora - t_agarre) > 1:
                secuencia   = 4
                objetivo_ik = wp_deposit
                ori_ik      = ori_place
                ganancia_vel = 1
                t_deposito  = ahora

        elif secuencia == 4:
            # Mover al compartimento, soltar y terminar
            if (ahora - t_deposito) > 6:
                dedo = 0   # abrir gripper

            if (ahora - t_deposito) > 7:
                secuencia       = 5
                ik_encendida    = False
                t_fin           = ahora

        elif secuencia == 5:
            # Volver todos los joints a posición de reposo
            for joint in BRAZO:
                p.setJointMotorControl2(
                    bodyIndex=rover, jointIndex=joint,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0, targetVelocity=0,
                    force=180, positionGain=0.01,
                    velocityGain=ganancia_vel
                )
            if (ahora - t_fin) > 5:
                print("Secuencia pick & place finalizada.")
                break

        # === CINEMÁTICA INVERSA ===================================

        if ik_encendida:
            poses = p.calculateInverseKinematics(
                rover, END_EFFECTOR, objetivo_ik,
                targetOrientation=ori_ik
            )
            for i, joint in enumerate(BRAZO):
                p.setJointMotorControl2(
                    bodyIndex=rover, jointIndex=joint,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=poses[i],
                    targetVelocity=0, force=180,
                    positionGain=0.005, velocityGain=ganancia_vel
                )

        # === GRIPPER ===============================================

        p.setJointMotorControlArray(
            rover, GRIPPERS, p.POSITION_CONTROL,
            targetPositions=[dedo, -dedo],
            forces=[300, 300]
        )

        # === RUEDAS ================================================

        p.setJointMotorControlArray(
            rover, RUEDAS, p.VELOCITY_CONTROL,
            targetVelocities=velocidad,
            forces=[40, 40, 40, 40]
        )

        # === REGISTRO G-PARCIAL ====================================

        if (ahora - t_ultimo_reg) >= MUESTREO_CSV and num_reg < MAX_MUESTRAS:
            g = 0.0
            if t_agarre > 0:
                for jnt in IK_JOINTS:
                    g += abs(p.getJointState(rover, jnt)[3])

            registro[num_reg] = [ahora - t0, NUM_IK, g]
            t_ultimo_reg = ahora
            num_reg += 1

except KeyboardInterrupt:
    print("Simulación detenida manualmente.")

# ==== EXPORTAR CSV =================================================

np.savetxt(
    "./Gparcial.csv",
    registro[:num_reg],
    delimiter=",",
    fmt="%.3f",
    header="tiempo(s),Numero_joints,G-parcial(N)",
    comments=""
)
print(f"Archivo Gparcial.csv guardado con {num_reg} muestras.")

p.disconnect()
