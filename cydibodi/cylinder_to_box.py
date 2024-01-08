from data_classes import Cylinder, Box
import numpy as np

class CylinderToBoxDistance:

    def __init__(self, cylinder: Cylinder, box: Box) -> None:
        self.cylinder = cylinder
        self.box = box

    def shortest_distance(self) -> float:
        """
        Calculates the shortest distance between the cylinder and the box.

        Returns:
            The shortest distance between the cylinder and the box.

        """
        pass

    def get_box_points(self, T) -> np.ndarray:
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
        transformed_corners = [np.dot(T, corner)[:3] for corner in unit_cube_corners]
        return transformed_corners