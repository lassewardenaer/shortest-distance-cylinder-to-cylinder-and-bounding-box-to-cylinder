from cydibodi.data_classes import Cylinder
import numpy as np
from scipy import optimize
import enum

class Circle(enum.Enum):
    TOP = 0
    BOTTOM = 1

class CylinderToCylinderDistance:
    def __init__(self, cylinderA: Cylinder, cylinderB: Cylinder):
        self.cylinderA = cylinderA
        self.cylinderB = cylinderB

        self.cylinderA_trasformation_matrix = self.getT(self.cylinderA)
        self.cylinderB_trasformation_matrix = self.getT(self.cylinderB)

    def getT(self, cylinder: Cylinder):
        R_homogeneous = np.vstack((cylinder.R, np.array([0, 0, 0])))
        R_homogeneous = np.hstack((R_homogeneous, np.array([[0], [0], [0], [1]])))

        translation_matrix = np.array([
            [1, 0, 0, cylinder.translation[0]],
            [0, 1, 0, cylinder.translation[1]],
            [0, 0, 1, cylinder.translation[2]],
            [0, 0, 0, 1]
        ])

        scaling_matrix = np.array([
            [cylinder.scaling[0], 0, 0, 0],
            [0, cylinder.scaling[1], 0, 0],
            [0, 0, cylinder.scaling[2], 0],
            [0, 0, 0, 1]
        ])

        return translation_matrix @ scaling_matrix @ R_homogeneous

    def shortest_distance_circle_to_circle(self):

        # Reasonable initial guess
        initial_guess = [0, 0, 0, 0]

        # Minimize the distance for each combination of top and bottom circles
        distances = [optimize.minimize(fun, initial_guess, args=(self.cylinderA, self.cylinderB), method='SLSQP')
                     for fun in [self.objective_function_circle_A1_to_B1,
                                 self.objective_function_circle_A1_to_B2,
                                 self.objective_function_circle_A2_to_B1,
                                 self.objective_function_circle_A2_to_B2]]

        shortest_distance = min(dist.fun for dist in distances)
        index = np.argmin([dist.fun for dist in distances])

        optimal_values = distances[index].x
        optimal_radiusA, optimal_angleA, optimal_radiusB, optimal_angleB = optimal_values[0], optimal_values[1], optimal_values[2], optimal_values[3]

        return shortest_distance, optimal_radiusA, optimal_radiusB, optimal_angleA, optimal_angleB, index

    def get_point_cylinder_circle_top(self, cylinder: Cylinder, angle: float):
        radius = cylinder.scaling[0]
        circle_vector = cylinder.R @ np.array([np.cos(angle), np.sin(angle), 0]) * radius
        center_circle_top = cylinder.translation + cylinder.R @ np.array([0, 0, 1]) * cylinder.scaling[2]
        point_circle_top = center_circle_top + circle_vector
        return point_circle_top

    def get_point_cylinder_circle_bottom(self, cylinder: Cylinder, angle: float):
        radius = cylinder.scaling[0]
        circle_vector = cylinder.R @ np.array([np.cos(angle), np.sin(angle), 0]) * radius
        center_circle_bottom = cylinder.translation - cylinder.R @ np.array([0, 0, 1]) * cylinder.scaling[2]
        point_circle_bottom = center_circle_bottom + circle_vector
        return point_circle_bottom

    def objective_function_circle_A1_to_B1(self, x, cylinderA, cylinderB):
        point_circle_A_top = self.get_point_cylinder_circle_top(cylinderA, x[0])
        point_circle_B_top = self.get_point_cylinder_circle_top(cylinderB, x[1])
        print(np.linalg.norm(point_circle_A_top - point_circle_B_top))
        return np.linalg.norm(point_circle_A_top - point_circle_B_top)

    def objective_function_circle_A1_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A_top = self.get_point_cylinder_circle_top(cylinderA, x[0])
        point_circle_B_bottom = self.get_point_cylinder_circle_bottom(cylinderB, x[1])
        return np.linalg.norm(point_circle_A_top - point_circle_B_bottom)

    def objective_function_circle_A2_to_B1(self, x, cylinderA, cylinderB):
        point_circle_A_bottom = self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        point_circle_B_top = self.get_point_cylinder_circle_top(cylinderB, x[1])
        return np.linalg.norm(point_circle_B_top - point_circle_A_bottom)

    def objective_function_circle_A2_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A_bottom = self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        point_circle_B_bottom = self.get_point_cylinder_circle_bottom(cylinderB, x[1])
        return np.linalg.norm(point_circle_A_bottom - point_circle_B_bottom)
