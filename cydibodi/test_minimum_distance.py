from cydibodi.cylinder_to_cylinder import CylinderToCylinderDistance, OPTIMAL_CIRCLE_COMBO
from cydibodi.data_classes import Cylinder
import numpy as np
import matplotlib.pyplot as plt
from cydibodi.create_figure import Figure3D

if __name__ == "__main__":
    angles = 0
    rotation_matrixA = np.array([
        [1, 0, 0],
        [0, np.cos(np.radians(angles)), -np.sin(np.radians(angles))],
        [0, np.sin(np.radians(angles)), np.cos(np.radians(angles))]
    ])
    radiusA = 1
    scalingA = np.array([radiusA, radiusA, 1])
    translationA = np.array([-2, 0, -2])

    fig = plt.figure()
    figure3D = Figure3D(fig)

    angles = 0
    rotation_matrix1 = np.array([
        [1, 0, 0],
        [0, np.cos(np.radians(angles)), -np.sin(np.radians(angles))],
        [0, np.sin(np.radians(angles)), np.cos(np.radians(angles))]
    ])
    radiusB = 1
    scalingB = np.array([radiusB, radiusB, 1])
    tranlationB = np.array([0, -5, 4])

    cylinderA = Cylinder(rotation_matrixA, scalingA, translationA)
    cylinderB = Cylinder(rotation_matrix1, scalingB, tranlationB)
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, figure3D.ax)

    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, figure3D.ax)
    shortest_distance, pointA, pointB = cylinderToCyilinderDistance.shortest_distance()

    print("Shortest distance: ", shortest_distance)
    print("Point A: ", pointA)
    print("Point B: ", pointB)

    figure3D.draw_cylinder(cylinderToCyilinderDistance)

    figure3D.ax.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], [pointA[2], pointB[2]], c='g')
    figure3D.ax.scatter([pointA[0]], [pointA[1]], [pointA[2]], marker='o')

    figure3D.show_axis()

    import time
    current_time = time.time()
    cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, figure3D.ax)
    shortest_distance, pointA, pointB = cylinderToCyilinderDistance.shortest_distance()
    print("Time to compute shortest distance: ", time.time() - current_time)
    plt.show()

    # x[0] = angleA
    # x[1] = scalingA [-1, 1]
    # x[2] = angleB
    # x[3] = scalingB [-1, 1]
    initial_x_list = [
            [0, 0, 0, 0],
            [0, 0.5, 0, 0.5],
            [-0.8, 0.5, -0.8, 0.5],
            [0.8, 0.5, 0.8, 0.5],
            [0, -0.5, 0, -0.5],
            [-0.8, -0.5, -0.8, -0.5],
            [0.8, -0.5, 0.8, -0.5],
            [0, 0.5, 0, -0.5],
            [0, -0.5, 0, 0.5],
            [-1.6, 0.5, 1.6, -0.5],
            [1.6, 0.5, 1.6, -0.5],
            [1.6, 0.5, -1.6, -0.5],
            [-1.6, 0.5, -1.6, -0.5],
            [1.6, 0.5, -1.6, 0.5],
            [0.3, 1, 0.3, 1],
            [0.3, -1, 0.3, 1],
            [0.3, -1, 0.3, -1],
            [-0.3, 1, -0.3, -1],
            [1, 1, -1, -1],
            [-1, -1, -0.3, -1]
            ]

    cost_values = []
    for initial_x in initial_x_list:
        cylinderToCyilinderDistance = CylinderToCylinderDistance(cylinderA, cylinderB, figure3D.ax)
        shortest_distance, pointA, pointB = cylinderToCyilinderDistance.shortest_distance(initial_guess=initial_x)
        cost_values.append(cylinderToCyilinderDistance.cost)

    #plot the cost values
    plt.figure()
    current_time = time.time()
    for cost in cost_values:
        x_axis = np.linspace(0, len(cost)-1, len(cost))
        plt.ylim(0, 10)
        plt.xlabel("Iteration")
        plt.ylabel("Cost")
        plt.plot(x_axis, cost)
    print(f"Time to compute shortest distance for {len(cost_values)} distances: ", time.time() - current_time)
    plt.show()