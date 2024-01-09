from cydibodi.data_classes import Cylinder
import numpy as np
from scipy import optimize
from utilities import GeometryUtilities

class CylinderToCylinderDistance:
    """
    Calculates the shortest distance between two cylinders in 3D space.

    Args:
        cylinderA (Cylinder): The first cylinder.
        cylinderB (Cylinder): The second cylinder.
        ax: The axis along which the cylinders are aligned.

    Attributes:
        cylinderA (Cylinder): The first cylinder.
        cylinderB (Cylinder): The second cylinder.
        ax: Axis for the plot.
        T_A: Transformation matrix for cylinder A.
        T_B: Transformation matrix for cylinder B.
        cost (list): List to store the cost function values.

    Methods:
        shortest_distance(initial_guess=None, method='SLSQP'): Calculates the shortest distance between the cylinders.
        shortest_distance_circular_to_circular(initial_guess=None, method='SLSQP'): Calculates the shortest distance between the circular sections of the cylinders.
        objective_function_circular_to_circular(x): Objective function for the optimization problem.
        get_point_cylinder_circle(angle, cylinder_instance, side): Calculates a point on the circular section of a cylinder.

    """

    def __init__(self, cylinderA: Cylinder, cylinderB: Cylinder, ax):
        self.cylinderA = cylinderA
        self.cylinderB = cylinderB
        self.ax = ax

        self.T_A = GeometryUtilities.getT_cylinder(self.cylinderA.R, self.cylinderA.translation, self.cylinderA.scaling[0], self.cylinderA.scaling[2])
        self.T_B = GeometryUtilities.getT_cylinder(self.cylinderB.R, self.cylinderB.translation, self.cylinderB.scaling[0], self.cylinderB.scaling[2])

        self.cost = []

    def shortest_distance(self, initial_guess=None, method='SLSQP'):
        """
        Calculates the shortest distance between the cylinders.

        Args:
            initial_guess: Initial guess for the optimization problem.
            method: Optimization method to use.

        Returns:
            shortest_distance_circular_to_circular (float): The shortest distance between the circular sections of the cylinders.
            optimal_point_A (numpy.ndarray): The optimal point on cylinder A.
            optimal_point_B (numpy.ndarray): The optimal point on cylinder B.

        """
        shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B = self.shortest_distance_circular_to_circular(initial_guess=initial_guess, method=method)
        return shortest_distance_circular_to_circular, optimal_point_A, optimal_point_B

    def shortest_distance_circular_to_circular(self, initial_guess=None, method='SLSQP'):
        """
        Calculates the shortest distance between the circular sections of the cylinders.

        Args:
            initial_guess: Initial guess for the optimization problem.
            method: Optimization method to use.

        Returns:
            shortest_distance (float): The shortest distance between the circular sections of the cylinders.
            optimal_point_A (numpy.ndarray): The optimal point on cylinder A.
            optimal_point_B (numpy.ndarray): The optimal point on cylinder B.

        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0, 0.5]

        # x[0] = angle for points on circle for line1
        # x[1] = length of line1 vector
        # x[2] = angle for points on circle for line2
        # x[3] = length of line2 vector
        # x[4] = radius
        constraints = [
            {'type': 'ineq', 'fun': lambda x: 1 - x[1]},
            {'type': 'ineq', 'fun': lambda x: 1 + x[1]},
            {'type': 'ineq', 'fun': lambda x: 1 - x[3]},
            {'type': 'ineq', 'fun': lambda x: 1 + x[3]},
            {'type': 'ineq', 'fun': lambda x: 1 - x[4]},
            {'type': 'ineq', 'fun': lambda x: 1 + x[4]},
        ]

        result = optimize.minimize(self.objective_function_circular_to_circular, initial_guess, method=method, constraints=constraints)

        optimal_angle_for_lineA = result.x[0]
        optimal_height_scaling_A = result.x[1]
        optimal_angle_for_lineB = result.x[2]
        optimal_height_scaling_B = result.x[3]
        optimal_radius = result.x[4]
        print("radius: ", optimal_radius)

        optimal_point_A = self.T_A @ np.array([np.cos(optimal_angle_for_lineA)*optimal_radius, np.sin(optimal_angle_for_lineA)*optimal_radius, optimal_height_scaling_A, 1])
        optimal_point_B = self.T_B @ np.array([np.cos(optimal_angle_for_lineB)*optimal_radius, np.sin(optimal_angle_for_lineB)*optimal_radius, optimal_height_scaling_B, 1])

        return result.fun, optimal_point_A, optimal_point_B

    def objective_function_circular_to_circular(self, x):
        """
        Objective function for the optimization problem.

        Args:
            x: Optimization variables.

        Returns:
            cost_function (float): The value of the cost function.

        """
        pointA = self.T_A @ np.array([np.cos(x[0])*x[4], np.sin(x[0])*x[4], x[1], 1])
        pointB = self.T_B @ np.array([np.cos(x[2])*x[4], np.sin(x[2])*x[4], x[3], 1])

        min_vector = np.array(pointA - pointB)
        cost_function = np.linalg.norm(min_vector)

        self.cost.append(cost_function)

        return cost_function

    def get_point_cylinder_circle(self, angle, cylinder_instance, side):
        """
        Calculates a point on the circular section of a cylinder.

        Args:
            angle: Angle on the circular section.
            cylinder_instance: Instance of the cylinder ('A' or 'B').
            side: Side of the cylinder ('top' or 'bottom').

        Returns:
            Point on the circular section of the cylinder.

        Raises:
            ValueError: If the cylinder instance or side is invalid.

        """
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
