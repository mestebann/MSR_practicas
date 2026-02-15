import numpy as np
import matplotlib.pyplot as plt

# Cargar datos
datos = np.loadtxt("resultados.csv", delimiter = ",", skiprows = 1)

posicion_y = datos[:,1]
velocidad_y = datos[:,2]

# Graficar
plt.figure()
plt.plot(posicion_y, velocidad_y)

plt.xlabel("Posición Y (m)")
plt.ylabel("Velocidad Y (m/s)")
plt.title("Velocidad del robot en función de la posición")

plt.grid(True)
plt.show()
