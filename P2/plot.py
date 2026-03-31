import numpy as np
import matplotlib.pyplot as plt

datos = np.loadtxt("./Gparcial.csv", delimiter=",", skiprows=1)

time = datos[:, 0]
gparcial = datos[:, 2]

# Graficar
plt.figure()
plt.plot(time, gparcial)

plt.xlabel("Tiempo (s)")
plt.ylabel("Fuerza total brazo (N)")
plt.title("tiempo vs G-parcial")

text = '\n'.join((
    f'G-TOTAL = {np.sum(gparcial):.3f}',
    f'Desviacion Estándar={np.std(gparcial):.3f}',
))

plt.text(0.98, 0.02, text, transform=plt.gca().transAxes,
        verticalalignment='bottom', horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.grid(True)
plt.show()