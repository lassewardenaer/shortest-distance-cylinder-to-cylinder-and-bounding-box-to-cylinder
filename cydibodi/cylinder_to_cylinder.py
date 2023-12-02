from cydibodi.data_classes import Cylinder
import numpy as np
from scipy import optimize

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

    def shortest_distance(self):

        radius = self.cylinderA.scaling[0]
        constraints_circles = [
            {'type': 'ineq', 'fun': lambda x: x[0] - radius},
            {'type': 'ineq', 'fun': lambda x: x[2] - radius},
        ]

        distance_circleA1_to_circleB1 = optimize.minimize(self.objective_function_circle_A1_to_B1, [0, 0, 0, 0], args=(self.cylinderA, self.cylinderB), constraints=constraints_circles).fun
        distance_circleA1_to_circleB2 = optimize.minimize(self.objective_function_circle_A1_to_B2, [0, 0, 0, 0], args=(self.cylinderA, self.cylinderB), constraints=constraints_circles).fun
        distance_circleA2_to_circleB1 = optimize.minimize(self.objective_function_circle_A2_to_B1, [0, 0, 0, 0], args=(self.cylinderA, self.cylinderB), constraints=constraints_circles).fun
        distance_circleA2_to_circleB2 = optimize.minimize(self.objective_function_circle_A2_to_B2, [0, 0, 0, 0], args=(self.cylinderA, self.cylinderB), constraints=constraints_circles).fun

        shortest_distance = np.min([
            distance_circleA1_to_circleB1,
            distance_circleA1_to_circleB2,
            distance_circleA2_to_circleB1,
            distance_circleA2_to_circleB2
        ])

        return shortest_distance

    def get_point_cylinder_circles(self, cylinder: Cylinder, radius: float, angle: float):
        center_circle_A = cylinder.translation + cylinder.R @ np.array([0, 0, 1]) * cylinder.scaling[2]
        center_circle_B = cylinder.translation - cylinder.R @ np.array([0, 0, 1]) * cylinder.scaling[2]

        circle_point = cylinder.R @ np.array([np.cos(angle), np.sin(angle), 0]) * radius
        point_circle_A = center_circle_A + circle_point
        point_circle_B = center_circle_B + circle_point

        return point_circle_A, point_circle_B

    def objective_function_circle_A1_to_B1(self, x, cylinderA, cylinderB):
        point_circle_A1, _ = self.get_point_cylinder_circles(cylinderA, x[0], x[1])
        point_circle_B1, _ = self.get_point_cylinder_circles(cylinderB, x[2], x[3])

        return np.linalg.norm(point_circle_A1 - point_circle_B1)

    def objective_function_circle_A1_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A1, _ = self.get_point_cylinder_circles(cylinderA, x[0], x[1])
        _, point_circle_B2 = self.get_point_cylinder_circles(cylinderB, x[2], x[3])

        return np.linalg.norm(point_circle_A1 - point_circle_B2)

    def objective_function_circle_A2_to_B1(self, x, cylinderA, cylinderB):
        _, point_circle_A2 = self.get_point_cylinder_circles(cylinderA, x[0], x[1])
        point_circle_B1, _ = self.get_point_cylinder_circles(cylinderB, x[2], x[3])

        return np.linalg.norm(point_circle_A2 - point_circle_B1)

    def objective_function_circle_A2_to_B2(self, x, cylinderA, cylinderB):
        point_circle_A1, point_circle_A2 = self.get_point_cylinder_circles(cylinderA, x[0], x[1])
        point_circle_B1, point_circle_B2 = self.get_point_cylinder_circles(cylinderB, x[2], x[3])

        return np.linalg.norm(point_circle_A2 - point_circle_B2)

    def radius_constraint(self, x, max_radius: float, min_radius: float):
        return x[0] - max_radius, x[0] - min_radius, x[2] - max_radius, x[2] - min_radius