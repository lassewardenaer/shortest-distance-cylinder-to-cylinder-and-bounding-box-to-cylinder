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

