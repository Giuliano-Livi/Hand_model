import numpy as np

point_h1 = np.array([0.087, 0.027, -0.025])
point_h6 = np.array([0.07, 0.07, -0.025])

axis_vector = point_h1 - point_h6  # vector from h1 to h6
unit_axis = axis_vector / np.linalg.norm(axis_vector)  # normalized vector

print("Axis vector:", axis_vector)
print("Normalized axis:", unit_axis)