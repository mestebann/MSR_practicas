import numpy as np
import matplotlib.pyplot as plt

datos = np.loadtxt("./Gparcial.csv", delimiter=",", skiprows=1)

tiempo = datos[:, 0]
g_parcial = datos[:, 2]

g_total = np.sum(g_parcial)
desviacion_estandar = np.std(g_parcial)

# Graficar
plt.figure()
plt.plot(tiempo, g_parcial)

plt.xlabel("Tiempo (s)")
plt.ylabel("Fuerza total brazo (N)")

# G-Total y desviación estándar en el título, como exige el enunciado
plt.title(f"Tiempo vs G-parcial  |  G-Total = {g_total:.3f} N  |  Std = {desviacion_estandar:.3f} N")


plt.grid(True)
plt.show()