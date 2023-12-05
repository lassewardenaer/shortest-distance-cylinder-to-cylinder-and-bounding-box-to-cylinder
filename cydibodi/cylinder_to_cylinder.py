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

        initial_guess = [0, 0]

        distances = [optimize.minimize(fun, initial_guess, args=(self.cylinderA, self.cylinderB), method='SLSQP')
                     for fun in [self.objective_function_circle_A1_to_B1,
                                 self.objective_function_circle_A1_to_B2,
                                 self.objective_function_circle_A2_to_B1,
                                 self.objective_function_circle_A2_to_B2]]

        shortest_distance = min(dist.fun for dist in distances)
        index = np.argmin([dist.fun for dist in distances])
        optimal_circle_combintaions = [OPTIMAL_CIRCLE_COMBO.TOP_TOP, OPTIMAL_CIRCLE_COMBO.TOP_BOTTOM,
                                       OPTIMAL_CIRCLE_COMBO.BOTTOM_TOP, OPTIMAL_CIRCLE_COMBO.BOTTOM_BOTTOM]

        optimal_values = distances[index].x
        optimal_angleA, optimal_angleB = optimal_values[0], optimal_values[1]

        return shortest_distance, optimal_angleA, optimal_angleB, optimal_circle_combintaions[index]

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
        return np.linalg.norm(point_circle_A_top - point_circle_B_top)**2

    def objective_function_circle_A1_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A_top = self.get_point_cylinder_circle_top(cylinderA, x[0])
        point_circle_B_bottom = self.get_point_cylinder_circle_bottom(cylinderB, x[1])
        return np.linalg.norm(point_circle_A_top - point_circle_B_bottom)**2

    def objective_function_circle_A2_to_B1(self, x, cylinderA, cylinderB):
        point_circle_A_bottom = self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        point_circle_B_top = self.get_point_cylinder_circle_top(cylinderB, x[1])
        return np.linalg.norm(point_circle_A_bottom - point_circle_B_top)**2

    def objective_function_circle_A2_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A_bottom = self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        point_circle_B_bottom = self.get_point_cylinder_circle_bottom(cylinderB, x[1])
        return np.linalg.norm(point_circle_A_bottom - point_circle_B_bottom)**2
