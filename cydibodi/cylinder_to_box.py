from data_classes import Cylinder, Box, Line
import numpy as np
from utilities import Utilities

class CylinderToBoxDistance:

    def __init__(self, cylinder: Cylinder, box: Box, simplify_cylinder: bool=False, number_of_lines: int=4) -> None:
        """
        Attributes:
            cylinder (Cylinder): The cylinder.
            box (Box): The box.
            simplify_cylinder (bool): Whether to simplify the cylinder or not.
            number_of_lines (int): Number of lines to use when simplifying the cylinder.
        """
        self.cylinder = cylinder
        self.box = box

        self.simplify_cylinder = simplify_cylinder
        self.number_of_lines = number_of_lines

        self.T_cylinder = Utilities.getT_cylinder(self.cylinder.R, self.cylinder.translation, self.cylinder.scaling[0], self.cylinder.scaling[2])
        self.T_box = Utilities.getT(self.box.translation, self.box.scaling)

    def shortest_distance(self) -> float:
        """
        Calculates the shortest distance between the cylinder and the box.

        Returns:
            The shortest distance between the cylinder and the box.

        """
        if self.simplify_cylinder:
            return self.shortest_distance_simplified_cyinder_to_box()
        else:
            return self.shortest_distance_cylinder_to_box()

    def shortest_distance_simplified_cyinder_to_box(self) -> float:
        """
        Calculates the shortest distance between the simplified cylinder and the box.

        Returns:
            The shortest distance between the simplified cylinder and the box.

        """
        pass

    def shortest_distance_cylinder_to_box(self) -> float:
        """
        Calculates the shortest distance between the cylinder and the box.

        Returns:
            The shortest distance between the cylinder and the box.

        """
        pass

    def line_to_line_distance(self, line1: Line, line2: Line) -> float:
        """
        Calculates the shortest distance between two lines.

        Args:
            line1: The first line.
            line2: The second line.

        Returns:
            The shortest distance between the two lines.

        """
        direction1 = (line1.pointB - line1.pointA)
        direction2 = (line2.pointB - line2.pointA)

        perpendicular_vec = np.cross(direction1, direction2)

        if np.linalg.norm(perpendicular_vec) == 0:
            return (np.Inf, np.array([np.Inf, np.Inf, np.Inf]), np.array([np.Inf, np.Inf, np.Inf]))

        n1 = np.cross(direction1, perpendicular_vec)
        n2 = np.cross(direction2, perpendicular_vec)

        point1 = line1.pointB + np.dot((line2.pointB - line1.pointB), n2) / np.dot(direction1, n2) * direction1
        point2 = line2.pointB + np.dot((line1.pointB - line2.pointB), n1) / np.dot(direction2, n1) * direction2

        distance = np.linalg.norm(point1 - point2)

        return zip(distance, point1, point2)

    def get_box_points(self, T_box) -> np.ndarray:
        """
        Calculates the points of the box.

        Input:
            T: Transformation matrix.

        Returns:
            The points of the box.

        """
        unit_cube_corners = np.array([
            [-1, -1, -1, 1],
            [1, -1, -1, 1],
            [-1, 1, -1, 1],
            [1, 1, -1, 1],
            [-1, -1, 1, 1],
            [1, -1, 1, 1],
            [-1, 1, 1, 1],
            [1, 1, 1, 1]
        ])
        transformed_corners = [np.dot(T_box, corner)[:3] for corner in unit_cube_corners]
        return transformed_corners

