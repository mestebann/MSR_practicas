import numpy as np
import matplotlib.pyplot as plt

# Cargar datos
datos1 = np.loadtxt("resultados1.csv", delimiter = ",", skiprows = 1)
datos2 = np.loadtxt("resultados2.csv", delimiter = ",", skiprows = 1)

posicion_y_1 = datos1[:,1]
velocidad_y_1 = datos1[:,2]

posicion_y_2 = datos2[:,1]
velocidad_y_2 = datos2[:,2]

# Graficar
plt.figure(figsize = (8, 5))
plt.plot(posicion_y_1, velocidad_y_1, label = "Without friction")
plt.plot(posicion_y_2, velocidad_y_2, label = "With friction")

plt.xlabel("Posición Y (m)")
plt.ylabel("Velocidad Y (m/s)")
plt.title("Velocidad del robot en función de la posición")

plt.grid(True)
plt.legend()
plt.show()
