import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# Vertici del parallelepipedo (origine + dimensioni lungo x, y, z)
origin = np.array([0, 0, 0])
lengths = np.array([2, 1, 1.5])  # lunghezze lungo x, y, z

# Calcolo i 8 vertici
x, y, z = origin
l, w, h = lengths

vertices = np.array([
    [x, y, z],
    [x + l, y, z],
    [x + l, y + w, z],
    [x, y + w, z],
    [x, y, z + h],
    [x + l, y, z + h],
    [x + l, y + w, z + h],
    [x, y + w, z + h]
])

# Definizione delle 6 facce
faces = [
    [vertices[0], vertices[1], vertices[2], vertices[3]],  # base
    [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
    [vertices[0], vertices[1], vertices[5], vertices[4]],  # lato x
    [vertices[1], vertices[2], vertices[6], vertices[5]],  # lato y
    [vertices[2], vertices[3], vertices[7], vertices[6]],  # lato opposto x
    [vertices[3], vertices[0], vertices[4], vertices[7]]   # lato opposto y
]

# Disegno
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.add_collection3d(Poly3DCollection(faces, facecolors='lightblue', linewidths=1, edgecolors='k', alpha=0.7))

# Set dei limiti degli assi
ax.set_xlim([x - 1, x + l + 1])
ax.set_ylim([y - 1, y + w + 1])
ax.set_zlim([z - 1, z + h + 1])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
