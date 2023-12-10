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

    def shortest_distance(self):
        #shortest_distance_circle_to_circle, optimal_cc_angleA, optimal_cc_angleB, optimal_cc_circle_combintaions = self.shortest_distance_circle_to_circle()
        #shortest_distance_line_to_point, optimal_point_on_line, optimal_angle_for_line, optimal_point_on_circle = self.shortest_distance_all_line_to_circle_candidates()
        shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B = self.shortest_distance_circular_to_circular()
        return shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B

        # if shortest_distance_circle_to_circle < shortest_distance_line_to_point:
        #     print("Circle to circle")
        #     pointA, pointB = self.get_optimal_points_on_circles(self.cylinderA, self.cylinderB, optimal_cc_angleA, optimal_cc_angleB, optimal_cc_circle_combintaions)
        #     return shortest_distance_circle_to_circle, pointA, pointB
        # else:
        #     print("Circle point to line point")
        #     return shortest_distance_line_to_point, optimal_point_on_line, optimal_point_on_circle

    def shortest_distance_circular_to_circular(self):
        initial_guess = [3, 0.5, 0, 0.5]

        # x[0] = angle for points on circle for line1
        # x[1] = length of line1 vector
        # x[2] = angle for points on circle for line2
        # x[3] = length of line2 vector
        constraints = [
            {'type': 'ineq', 'fun': lambda x: x[1] - self.cylinderB.scaling[2]},
            {'type': 'ineq', 'fun': lambda x: self.cylinderA.scaling[2] - x[1]},
            {'type': 'ineq', 'fun': lambda x: x[3] - self.cylinderB.scaling[2]},
            {'type': 'ineq', 'fun': lambda x: self.cylinderB.scaling[2] - x[3]},
        ]

        result = optimize.minimize(self.objective_function_circular_to_circular, initial_guess, method='SLSQP', constraints=constraints)

        optimal_angle_for_lineA, optimal_height_scaling_A, optimal_angle_for_lineB, optimal_height_scaling_B = result.x[0], result.x[1], result.x[2], result.x[3]

        optimal_point_A = self.T_A @ np.array([np.cos(optimal_angle_for_lineA), np.sin(optimal_angle_for_lineA), optimal_height_scaling_A, 1])
        optimal_point_B = self.T_B @ np.array([np.cos(optimal_angle_for_lineB), np.sin(optimal_angle_for_lineB), optimal_height_scaling_B, 1])

        return np.sqrt(result.fun), optimal_point_A, optimal_point_B

    def objective_function_circular_to_circular(self, x):
        pointA = self.T_A @ np.array([np.cos(x[0]), np.sin(x[0]), x[1], 1])
        pointB = self.T_B @ np.array([np.cos(x[2]), np.sin(x[2]), x[3], 1])

        min_vector = np.array(pointA - pointB)
        cost_function = (min_vector.dot(min_vector.T))

        return cost_function

    def shortest_distance_all_line_to_circle_candidates(self):
        line_cylinderB_to_circleA = self.shortest_distance_line_to_circle(line_cylinder=self.cylinderB, circle_cylinder=self.cylinderA, objective_functions=[self.objective_function_circle_A_top_to_lineB, self.objective_function_circle_A_bottom_to_lineB])
        line_cylinderA_to_circleB = self.shortest_distance_line_to_circle(line_cylinder=self.cylinderA, circle_cylinder=self.cylinderB, objective_functions=[self.objective_function_circle_B_top_to_lineA, self.objective_function_circle_B_bottom_to_lineA])

        line_cylinderB_to_circleA_distance = line_cylinderB_to_circleA[0]
        line_cylinderA_to_circleB_distance = line_cylinderA_to_circleB[0]

        if line_cylinderB_to_circleA_distance < line_cylinderA_to_circleB_distance:
            shortest_distance_line_to_circle = line_cylinderB_to_circleA_distance
            optimal_point_on_line = line_cylinderB_to_circleA[1]
            optimal_angle_for_line = line_cylinderB_to_circleA[2]
            optimal_point_on_circle = line_cylinderB_to_circleA[3]
            return shortest_distance_line_to_circle, optimal_point_on_line, optimal_angle_for_line, optimal_point_on_circle
        else:
            shortest_distance_line_to_circle = line_cylinderA_to_circleB_distance
            optimal_point_on_line = line_cylinderA_to_circleB[1]
            optimal_angle_for_line = line_cylinderA_to_circleB[2]
            optimal_point_on_circle = line_cylinderA_to_circleB[3]
            return shortest_distance_line_to_circle, optimal_point_on_line, optimal_angle_for_line, optimal_point_on_circle


    def shortest_distance_line_to_circle(self, line_cylinder: Cylinder, circle_cylinder: Cylinder, objective_functions):
        initial_guess = [0, 0.5, 0]

        # x[0] = angle for points on circle for line
        # x[1] = length of line vector
        # x[2] = angle for circle
        constraints = [
            {'type': 'ineq', 'fun': lambda x: x[1]},
            {'type': 'ineq', 'fun': lambda x: 2 * line_cylinder.scaling[2] - x[1]},
        ]

        distances = [optimize.minimize(fun, initial_guess, args=(self.cylinderA, self.cylinderB), method='SLSQP', constraints=constraints)
                     for fun in objective_functions]

        shortest_distances = [dist.fun for dist in distances]
        shortest_distance = min(shortest_distances)
        index = np.argmin(shortest_distances)

        optimal_values = distances[index].x
        optimal_angle_for_line, optimal_line_vector_length, optimal_circle_angle = optimal_values[0], optimal_values[1], optimal_values[2]

        line = self.get_point_cylinder_circle_top(line_cylinder, optimal_angle_for_line) - self.get_point_cylinder_circle_bottom(line_cylinder, optimal_angle_for_line)
        unit_line = line / np.linalg.norm(line)
        line_vector = unit_line * optimal_line_vector_length
        optimal_line_point = self.get_point_cylinder_circle_bottom(line_cylinder, optimal_angle_for_line) + line_vector
        if index == 0:
            optimal_circle_point = self.get_point_cylinder_circle_top(circle_cylinder, optimal_circle_angle)
        if index == 1:
            optimal_circle_point = self.get_point_cylinder_circle_bottom(circle_cylinder, optimal_circle_angle)

        return np.sqrt(shortest_distance), optimal_line_point, optimal_angle_for_line, optimal_circle_point

    def objective_function_circle_A_top_to_lineB(self, x, cylinderA, cylinderB):
        line = self.get_point_cylinder_circle_top(cylinderB, x[0]) - self.get_point_cylinder_circle_bottom(cylinderB, x[0])
        unit_line = line / np.linalg.norm(line)
        line_vector = unit_line * x[1]
        line_point_2 = self.get_point_cylinder_circle_bottom(cylinderB, x[0]) + line_vector
        circle_point = self.get_point_cylinder_circle_top(cylinderA, x[2])
        min_vector = np.array(circle_point - line_point_2)
        cost_function = (min_vector.dot(min_vector.T)) ** 2
        return cost_function

    def objective_function_circle_A_bottom_to_lineB(self, x, cylinderA, cylinderB):
        line = self.get_point_cylinder_circle_top(cylinderB, x[0]) - self.get_point_cylinder_circle_bottom(cylinderB, x[0])
        unit_line = line / np.linalg.norm(line)
        line_vector = unit_line * x[1]
        line_point_2 = self.get_point_cylinder_circle_bottom(cylinderB, x[0]) + line_vector
        circle_point = self.get_point_cylinder_circle_bottom(cylinderA, x[2])
        min_vector = np.array(circle_point - line_point_2)
        cost_function = (min_vector.dot(min_vector.T)) ** 2
        return cost_function

    def objective_function_circle_B_top_to_lineA(self, x, cylinderA, cylinderB):
        line = self.get_point_cylinder_circle_top(cylinderA, x[0]) - self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        unit_line = line / np.linalg.norm(line)
        line_vector = unit_line * x[1]
        line_point_2 = self.get_point_cylinder_circle_bottom(cylinderA, x[0]) + line_vector
        circle_point = self.get_point_cylinder_circle_top(cylinderB, x[2])
        min_vector = np.array(circle_point - line_point_2)
        cost_function = (min_vector.dot(min_vector.T)) ** 2
        return cost_function

    def objective_function_circle_B_bottom_to_lineA(self, x, cylinderA, cylinderB):
        line = self.get_point_cylinder_circle_top(cylinderA, x[0]) - self.get_point_cylinder_circle_bottom(cylinderA, x[0])
        unit_line = line / np.linalg.norm(line)
        line_vector = unit_line * x[1]
        line_point_2 = self.get_point_cylinder_circle_bottom(cylinderA, x[0]) + line_vector
        circle_point = self.get_point_cylinder_circle_bottom(cylinderB, x[2])
        min_vector = np.array(circle_point - line_point_2)
        cost_function = (min_vector.dot(min_vector.T)) ** 2
        return cost_function

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
