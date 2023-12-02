from cydibodi.cylinder_to_cylinder import CylinderToCylinderDistance
from cydibodi.data_classes import Cylinder
import numpy as np


if __name__ == "__main__":
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]])
    cylinderA = Cylinder(rotation_matrix, np.array([1, 1, 1]), np.array([0, 0, 0]))
    cylinderB = Cylinder(rotation_matrix, np.array([1, 1, 1]), np.array([0, 0, 1]))
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB)

    shortest_distance = cylinderToCyilinderDistance.shortest_distance()
    print(shortest_distance)