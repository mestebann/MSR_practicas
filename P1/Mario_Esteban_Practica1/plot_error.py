import numpy as np
import matplotlib.pyplot as plt

# Cargar datos
datos1 = np.loadtxt("Fase3.csv", delimiter = ",", skiprows = 1)
datos2 = np.loadtxt("Fase4.csv", delimiter = ",", skiprows = 1)

posicion_y_1 = datos1[:,1]
velocidad_y_1 = datos1[:,2]

posicion_y_2 = datos2[:,1]
velocidad_y_2 = datos2[:,2]

# CALCULO DE ERROR
v_ref = 2.0

# Error Fase 3
error_f3 = velocidad_y_1 - v_ref
MAE_f3 = np.mean(np.abs(error_f3))
MSE_f3 = np.mean(error_f3**2)
RMSE_f3 = np.sqrt(MSE_f3)

# Error Fase 4
error_f4 = velocidad_y_2 - v_ref
MAE_f4 = np.mean(np.abs(error_f4))
MSE_f4 = np.mean(error_f4**2)
RMSE_f4 = np.sqrt(MSE_f4)

print("FASE 3:")
print("MAE  :", MAE_f3)
print("MSE  :", MSE_f3)
print("RMSE :", RMSE_f3)

print("\nFASE 4:")
print("MAE  :", MAE_f4)
print("MSE  :", MSE_f4)
print("RMSE :", RMSE_f4)

plt.figure(figsize=(8,5))
plt.plot(posicion_y_1, error_f3, label="Error Fase 3")
plt.plot(posicion_y_2, error_f4, label="Error Fase 4")
plt.axhline(0, linestyle="--")
plt.xlabel("Posición Y (m)")
plt.ylabel("Error respecto a 2 m/s")
plt.title("Comparación de error respecto a velocidad deseada")
plt.grid(True)
plt.legend()
plt.show()
