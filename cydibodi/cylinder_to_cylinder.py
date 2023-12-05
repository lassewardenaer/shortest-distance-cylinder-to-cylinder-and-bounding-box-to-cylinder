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

    def shortest_distance(self):
        shortest_distance_circle_to_circle, optimal_cc_angleA, optimal_cc_angleB, optimal_cc_circle_combintaions = self.shortest_distance_circle_to_circle()
        shortest_distance_line_to_point, closest_point_on_line, closest_point_in_point_to_line = self.shortest_distance_all_line_to_point()
        if shortest_distance_circle_to_circle < shortest_distance_line_to_point:
            pointA, pointB = self.get_optimal_points_on_circles(self.cylinderA, self.cylinderB, optimal_cc_angleA, optimal_cc_angleB, optimal_cc_circle_combintaions)
            return shortest_distance_circle_to_circle, pointA, pointB
        else:
            return shortest_distance_line_to_point, closest_point_on_line, closest_point_in_point_to_line

    def line_point_exseeds_the_line_segment(self, point, line_point1, line_point2):
        point = np.array(point)
        line_point1 = np.array(line_point1)
        line_point2 = np.array(line_point2)

        line_vec = line_point2 - line_point1
        point_vec1 = point - line_point1
        point_vec2 = point - line_point2

        norm_line_vec = np.linalg.norm(line_vec)
        norm_point_vec1 = np.linalg.norm(point_vec1)
        norm_point_vec2 = np.linalg.norm(point_vec2)
        if norm_point_vec1 > norm_line_vec or norm_point_vec2 > norm_line_vec:
            return True
        return False

    def shortest_distance_all_line_to_point(self):
        _, optimal_cc_angleA, optimal_cc_angleB, optimal_cc_circle_combintaions = self.shortest_distance_circle_to_circle()
        pointA_top = self.get_point_cylinder_circle_top(self.cylinderA, optimal_cc_angleA)
        pointA_bottom = self.get_point_cylinder_circle_bottom(self.cylinderA, optimal_cc_angleA)
        pointB_top = self.get_point_cylinder_circle_top(self.cylinderB, optimal_cc_angleB)
        pointB_bottom = self.get_point_cylinder_circle_bottom(self.cylinderB, optimal_cc_angleB)

        if optimal_cc_circle_combintaions == OPTIMAL_CIRCLE_COMBO.TOP_TOP:
            shortest_distance_line_A_to_point_B, closest_point_on_line_A = self.shortest_distance_line_point(pointB_top, pointA_top, pointA_bottom)
            shortest_distance_line_B_to_point_A, closest_point_on_line_B = self.shortest_distance_line_point(pointA_top, pointB_top, pointB_bottom)
            if shortest_distance_line_B_to_point_A < shortest_distance_line_A_to_point_B:
                return shortest_distance_line_B_to_point_A, closest_point_on_line_B, pointA_top
            else:
                return shortest_distance_line_A_to_point_B, closest_point_on_line_A, pointB_top
        elif optimal_cc_circle_combintaions == OPTIMAL_CIRCLE_COMBO.TOP_BOTTOM:
            shortest_distance_line_A_to_point_B, closest_point_on_line_A = self.shortest_distance_line_point(pointB_bottom, pointA_top, pointA_bottom)
            shortest_distance_line_B_to_point_A, closest_point_on_line_B = self.shortest_distance_line_point(pointA_top, pointB_top, pointB_bottom)
            if shortest_distance_line_B_to_point_A < shortest_distance_line_A_to_point_B:
                return shortest_distance_line_B_to_point_A, closest_point_on_line_B, pointA_top
            else:
                return shortest_distance_line_A_to_point_B, closest_point_on_line_A, pointB_bottom
        elif optimal_cc_circle_combintaions == OPTIMAL_CIRCLE_COMBO.BOTTOM_TOP:
            shortest_distance_line_A_to_point_B, closest_point_on_line_A = self.shortest_distance_line_point(pointB_top, pointA_top, pointA_bottom)
            shortest_distance_line_B_to_point_A, closest_point_on_line_B = self.shortest_distance_line_point(pointA_bottom, pointB_top, pointB_bottom)
            if shortest_distance_line_B_to_point_A < shortest_distance_line_A_to_point_B:
                return shortest_distance_line_B_to_point_A, closest_point_on_line_B, pointA_bottom
            else:
                return shortest_distance_line_A_to_point_B, closest_point_on_line_A, pointB_top
        elif optimal_cc_circle_combintaions == OPTIMAL_CIRCLE_COMBO.BOTTOM_BOTTOM:
            shortest_distance_line_A_to_point_B, closest_point_on_line_A = self.shortest_distance_line_point(pointB_bottom, pointA_top, pointA_bottom)
            shortest_distance_line_B_to_point_A, closest_point_on_line_B = self.shortest_distance_line_point(pointA_bottom, pointB_top, pointB_bottom)
            if shortest_distance_line_B_to_point_A < shortest_distance_line_A_to_point_B:
                return shortest_distance_line_B_to_point_A, closest_point_on_line_B, pointA_bottom
            else:
                return shortest_distance_line_A_to_point_B, closest_point_on_line_A, pointB_bottom


    def shortest_distance_line_point(self, point, line_point1, line_point2):
        point = np.array(point)
        line_point1 = np.array(line_point1)
        line_point2 = np.array(line_point2)

        line_vec = line_point2 - line_point1

        point_vec = point - line_point1

        line_unit_vec = line_vec / np.linalg.norm(line_vec)
        projected_length = np.dot(point_vec, line_unit_vec)
        closest_point_on_line = line_point1 + projected_length * line_unit_vec

        distance = np.linalg.norm(point - closest_point_on_line)

        if self.line_point_exseeds_the_line_segment(closest_point_on_line, line_point1, line_point2):
            if np.linalg.norm(point - line_point1) < np.linalg.norm(point - line_point2):
                return np.linalg.norm(point - line_point1), line_point1
            else:
                return np.linalg.norm(point - line_point2), line_point2
        return distance, closest_point_on_line

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

    def get_optimal_points_on_circles(self, cylinderA, cylinderB, angleA, angleB, optimal_circle_combo):
        if optimal_circle_combo == OPTIMAL_CIRCLE_COMBO.TOP_TOP:
            pointA = self.get_point_cylinder_circle_top(cylinderA, angleA)
            pointB = self.get_point_cylinder_circle_top(cylinderB, angleB)
            return pointA, pointB
        elif optimal_circle_combo == OPTIMAL_CIRCLE_COMBO.TOP_BOTTOM:
            pointA = self.get_point_cylinder_circle_top(cylinderA, angleA)
            pointB = self.get_point_cylinder_circle_bottom(cylinderB, angleB)
            return pointA, pointB
        elif optimal_circle_combo == OPTIMAL_CIRCLE_COMBO.BOTTOM_TOP:
            pointA = self.get_point_cylinder_circle_bottom(cylinderA, angleA)
            pointB = self.get_point_cylinder_circle_top(cylinderB, angleB)
            return pointA, pointB
        elif optimal_circle_combo == OPTIMAL_CIRCLE_COMBO.BOTTOM_BOTTOM:
            pointA = self.get_point_cylinder_circle_bottom(cylinderA, angleA)
            pointB = self.get_point_cylinder_circle_bottom(cylinderB, angleB)
            return pointA, pointB

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
