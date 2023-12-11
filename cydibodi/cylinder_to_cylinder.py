from cydibodi.data_classes import Cylinder
import numpy as np
from scipy import optimize
import enum

class OPTIMAL_CIRCLE_COMBO(enum.Enum):
    TOP_TOP = 0
    TOP_BOTTOM = 1
    BOTTOM_TOP = 2
    BOTTOM_BOTTOM = 3


class CylinderToCylinderDistance:
    def __init__(self, cylinderA: Cylinder, cylinderB: Cylinder, ax):
        self.cylinderA = cylinderA
        self.cylinderB = cylinderB
        self.ax = ax

        self.T_A = self.getT(self.cylinderA.R, self.cylinderA.translation, self.cylinderA.scaling[0], self.cylinderA.scaling[2])
        self.T_B = self.getT(self.cylinderB.R, self.cylinderB.translation, self.cylinderB.scaling[0], self.cylinderB.scaling[2])

        self.cost = []

    def getT(self, R, translation, radius, height):
        R_homogeneous = np.vstack((R, np.array([0, 0, 0])))
        R_homogeneous = np.hstack((R_homogeneous, np.array([[0], [0], [0], [1]])))

        translation_matrix = np.array([
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1]
        ])

        scaling_matrix = np.array([
            [radius, 0, 0, 0],
            [0, radius, 0, 0],
            [0, 0, height, 0],
            [0, 0, 0, 1]
        ])

        return translation_matrix @ scaling_matrix @ R_homogeneous

    def shortest_distance(self, initial_guess=None, method='SLSQP'):
        shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B = self.shortest_distance_circular_to_circular(initial_guess=initial_guess, method=method)
        return shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B

    def shortest_distance_circular_to_circular(self, initial_guess=None, method='SLSQP'):
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0]

        # x[0] = angle for points on circle for line1
        # x[1] = length of line1 vector
        # x[2] = angle for points on circle for line2
        # x[3] = length of line2 vector
        constraints = [
            {'type': 'ineq', 'fun': lambda x: 1 - x[1]},
            {'type': 'ineq', 'fun': lambda x: 1 + x[1]},
            {'type': 'ineq', 'fun': lambda x: 1 - x[3]},
            {'type': 'ineq', 'fun': lambda x: 1 + x[3]}
        ]

        result = optimize.minimize(self.objective_function_circular_to_circular, initial_guess, method=method, constraints=constraints)

        optimal_angle_for_lineA, optimal_height_scaling_A, optimal_angle_for_lineB, optimal_height_scaling_B = result.x[0], result.x[1], result.x[2], result.x[3]

        optimal_point_A = self.T_A @ np.array([np.cos(optimal_angle_for_lineA), np.sin(optimal_angle_for_lineA), optimal_height_scaling_A, 1])
        optimal_point_B = self.T_B @ np.array([np.cos(optimal_angle_for_lineB), np.sin(optimal_angle_for_lineB), optimal_height_scaling_B, 1])

        return result.fun, optimal_point_A, optimal_point_B

    def objective_function_circular_to_circular(self, x):
        pointA = self.T_A @ np.array([np.cos(x[0]), np.sin(x[0]), x[1], 1])
        pointB = self.T_B @ np.array([np.cos(x[2]), np.sin(x[2]), x[3], 1])

        min_vector = np.array(pointA - pointB)
        cost_function = np.linalg.norm(min_vector)

        self.cost.append(cost_function)

        return cost_function

    def get_point_cylinder_circle(self, angle, cylinder_instance, side):
        if cylinder_instance == 'A' and side == 'top':
            return self.T_A @ np.array([np.cos(angle), np.sin(angle), 1, 1])
        elif cylinder_instance == 'B' and side == 'top':
            return self.T_B @ np.array([np.cos(angle), np.sin(angle), 1, 1])
        elif cylinder_instance == 'A' and side == 'bottom':
            return self.T_A @ np.array([np.cos(angle), np.sin(angle), -1, 1])
        elif cylinder_instance == 'B' and side == 'bottom':
            return self.T_B @ np.array([np.cos(angle), np.sin(angle), -1, 1])
        else:
            raise ValueError("Invalid cylinder instance or side")
