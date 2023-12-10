from cydibodi.cylinder_to_cylinder import CylinderToCylinderDistance, OPTIMAL_CIRCLE_COMBO
from cydibodi.data_classes import Cylinder
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]])
    cylinderA = Cylinder(rotation_matrix, np.array([1, 1, 1]), np.array([0, 0, 0]))
    cylinderB = Cylinder(rotation_matrix, np.array([1, 1, 1]), np.array([0, 0, 1]))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, ax)

    shortest_distance = cylinderToCyilinderDistance.shortest_distance_circle_to_circle()

    angles = 90
    rotation_matrix1 = np.array([
        [1, 0, 0],
        [0, np.cos(np.radians(angles)), -np.sin(np.radians(angles))],
        [0, np.sin(np.radians(angles)), np.cos(np.radians(angles))]
    ])

    cylinderA = Cylinder(rotation_matrix, np.array([1, 1, 1]), np.array([0, 0, 0]))
    cylinderB = Cylinder(rotation_matrix1, np.array([1, 1, 1]), np.array([0, 1, 4]))
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, ax)

    # visualize the cylinders using plt
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)

    pointsA_top = [cylinderToCyilinderDistance.get_point_cylinder_circle_top(cylinderA, angle)  for angle in np.linspace(0, 2*np.pi, 10)]
    pointsA_bottom = [cylinderToCyilinderDistance.get_point_cylinder_circle_bottom(cylinderA, angle) for angle in np.linspace(0, 2*np.pi, 10)]
    pointsB_top = [cylinderToCyilinderDistance.get_point_cylinder_circle_top(cylinderB, angle) for angle in np.linspace(0, 2*np.pi, 10)]
    pointsB_bottom = [cylinderToCyilinderDistance.get_point_cylinder_circle_bottom(cylinderB, angle) for angle in np.linspace(0, 2*np.pi, 10)]

    x_coords = [x[0] for x in pointsA_top]
    y_coords = [x[1] for x in pointsA_top]
    z_coords = [x[2] for x in pointsA_top]

    ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

    x_coords = [x[0] for x in pointsB_top]
    y_coords = [x[1] for x in pointsB_top]
    z_coords = [x[2] for x in pointsB_top]

    ax.scatter(x_coords, y_coords, z_coords, c='b', marker='o')

    x_coords = [x[0] for x in pointsA_bottom]
    y_coords = [x[1] for x in pointsA_bottom]
    z_coords = [x[2] for x in pointsA_bottom]

    ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

    x_coords = [x[0] for x in pointsB_bottom]
    y_coords = [x[1] for x in pointsB_bottom]
    z_coords = [x[2] for x in pointsB_bottom]

    ax.scatter(x_coords, y_coords, z_coords, c='b', marker='o')

    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, ax)
    shortest_distance, pointA, pointB = cylinderToCyilinderDistance.shortest_distance()


    print("Shortest distance: ", shortest_distance)
    print("Point A: ", pointA)
    print("Point B: ", pointB)

    ax.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], [pointA[2], pointB[2]], c='g')
    ax.scatter([pointA[0]], [pointA[1]], [pointA[2]], marker='o')

    # show axis
    ax.quiver(0, 0, 0, 1, 0, 0, length=1, normalize=True, color='k')
    ax.quiver(0, 0, 0, 0, 1, 0, length=1, normalize=True, color='r')
    ax.quiver(0, 0, 0, 0, 0, 1, length=1, normalize=True, color='b')

    import time
    current_time = time.time()
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, ax)
    shortest_distance, pointA, pointB = cylinderToCyilinderDistance.shortest_distance()
    print("Time to compute shortest distance: ", time.time() - current_time)
    #plt.show()
